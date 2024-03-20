import copy
import random
import cv2
import sys
import matplotlib.pyplot as plt
import numpy as np
import os
import time
import shutil
import json
import matplotlib.image as mpimg
from PIL import Image

NUM_IMAGES_ALREADY_GENERATED = 0
NUM_IMAGES_TO_GENERATE = 100
IOU_THRESHOLD = 0.15
VISUALIZE = False
MAX_OBJECTS = 10
IMAGE_HEIGHT = 540
IMAGE_WIDTH = 960
np.set_printoptions(threshold=sys.maxsize)


def object_boxes_to_corner_format(bboxes):
    bboxes_corner_format = list()

    for bbox in bboxes:
        x_min = round((bbox[0] - bbox[2] / 2) * IMAGE_WIDTH)
        x_max = round((bbox[0] + bbox[2] / 2) * IMAGE_WIDTH)
        y_min = round((bbox[1] - bbox[3] / 2) * IMAGE_HEIGHT)
        y_max = round((bbox[1] + bbox[3] / 2) * IMAGE_HEIGHT)
        bboxes_corner_format.append([x_min, y_min, x_max, y_max, bbox[-1]])

    return bboxes_corner_format


def object_boxes_yolo_format(bboxes, classes):
    bboxes_yolo_format = list()
    class_dict = {"brown": 1, "small-white": 2, "medium-white": 3, "large-white": 4, "banana": 5, "catfood": 6,
                  "ketchup": 7, "mouthwash": 8, "showergel": 9, "skittles": 10, "stainremover": 11, "toothpaste": 12,
                  "trex": 13}
    for index in range(len(bboxes)):
        x_min, y_min, x_max, y_max = bboxes[index]
        object_class = class_dict[classes[index]]
        x_center = (x_max + x_min) / 2
        y_center = (y_max + y_min) / 2
        width = x_max - x_min
        # print(width)
        height = y_max - y_min

        bboxes_yolo_format.append(
            [x_center / IMAGE_WIDTH, y_center / IMAGE_HEIGHT, width / IMAGE_WIDTH, height / IMAGE_HEIGHT,
             object_class - 1])

    return bboxes_yolo_format


def create_all_objects_mask(all_object_masks, bboxes_yolo_format):
    masks_new = list()
    for index in range(len(all_object_masks) - 1, -1, -1):
        object_binary_mask = np.transpose(get_object_positive_mask(all_object_masks[index], if_one_channel=True),
                                          axes=[1, 2, 0])
        for i in range(len(masks_new)):
            upper_object_mask = np.transpose(get_object_positive_mask(masks_new[i], if_one_channel=True),
                                             axes=[1, 2, 0])
            object_binary_mask = (1 - upper_object_mask) * object_binary_mask
        masks_new.append(object_binary_mask)

    bboxes_centroid = list()
    bboxes_class = list()
    for index in range(len(bboxes_yolo_format) - 1, -1, -1):
        bboxes_class.append(bboxes_yolo_format[index][-1])
        bboxes_centroid.append((bboxes_yolo_format[index][0], bboxes_yolo_format[index][1]))
    bboxes_size_list_sorted = sorted(copy.deepcopy(bboxes_centroid), key=lambda x: x[0], reverse=False)

    for index in range(len(bboxes_size_list_sorted) - 1):
        if (bboxes_size_list_sorted[index + 1][0] - bboxes_size_list_sorted[index][0]) < 0.05:
            if bboxes_size_list_sorted[index + 1][1] < bboxes_size_list_sorted[index][1]:
                saved = bboxes_size_list_sorted[index]
                bboxes_size_list_sorted[index] = bboxes_size_list_sorted[index + 1]
                bboxes_size_list_sorted[index + 1] = saved

    bboxes_class_sorted = list()

    mask_tensor = np.zeros(shape=(IMAGE_HEIGHT, IMAGE_WIDTH, len(masks_new)))

    for index in range(len(bboxes_size_list_sorted)):
        index_in_non_sorted = bboxes_centroid.index(bboxes_size_list_sorted[index])
        bboxes_class_sorted.append(bboxes_class[index_in_non_sorted])
        mask_tensor[..., index:index + 1] = masks_new[index_in_non_sorted]

    return mask_tensor, bboxes_class_sorted


def reshape(image, w, h):
    reshaped_image = cv2.resize(image, (w, h), interpolation=cv2.INTER_NEAREST)
    return reshaped_image


def get_object_white_mask(background):
    ones = 255 * np.ones((background.shape[0], background.shape[1]))

    R_channel = background[..., 0] == ones
    G_channel = background[..., 1] == ones
    B_channel = background[..., 2] == ones
    mask = np.array([R_channel * G_channel * B_channel])

    return mask


def get_object_negative_mask(background):
    ones = np.zeros((background.shape[0], background.shape[1]))

    R_channel = background[..., 0] == ones
    G_channel = background[..., 1] == ones
    B_channel = background[..., 2] == ones
    mask = np.array([R_channel * G_channel * B_channel])

    return mask


def get_object_positive_mask(background, if_one_channel=False):
    ones = np.zeros((background.shape[0], background.shape[1]))

    if not if_one_channel:
        R_channel = background[..., 0] > ones
        G_channel = background[..., 1] > ones
        B_channel = background[..., 2] > ones
        mask = np.array([R_channel * G_channel * B_channel])

        return mask
    else:
        R_channel = background[..., 0] > ones
        mask = np.array([R_channel])

        return mask


def get_every_object_contour(each_object_separately):
    contours_to_return = list()
    for object_index_1 in range(len(each_object_separately)):
        lower_object_mask = get_object_positive_mask(each_object_separately[object_index_1])
        for object_index_2 in range(object_index_1 + 1, len(each_object_separately)):
            mask = get_object_negative_mask(each_object_separately[object_index_2])
            lower_object_mask *= mask
        lower_object_image = 255 - each_object_separately[object_index_1] * \
                             (np.transpose(np.repeat(lower_object_mask, [3], axis=0), axes=[1, 2, 0]))

        contours_to_return.append(find_mask_contours(lower_object_image))

    return contours_to_return


def create_json_dict(objects_classes, objects_contours, image_number):
    shapes_list = list()
    for index_object in range(len(objects_contours)):
        shapes_list.append({
            "label": objects_classes[index_object],
            "points": [objects_contours[index_object][index_point][0].tolist()
                       for index_point in range(len(objects_contours[index_object]))],
            "group_id": "null",
            "shape_type": "polygon",
            "flags": {},
        })
    dict = {
        "version": "5.0.1",
        "flags": {},
        "shapes": shapes_list,
        "image_path": "image_{}.png".format(image_number),
        "imageData": None,
        "imageHeight": IMAGE_HEIGHT,
        "imageWidth": IMAGE_WIDTH
    }

    return dict


def recompute_contour_coords(object_contour, x_min, y_min):
    recomputed_contour = list()
    for point in object_contour:
        recomputed_contour.append([[point[0][0] + x_min, point[0][1] + y_min]])
    return np.array(recomputed_contour)


def find_mask_contours(mask):
    mask = np.asarray(mask, dtype=np.uint8)
    mask = np.asarray(cv2.cvtColor(mask, cv2.COLOR_RGB2GRAY), dtype=np.uint8)

    ret, mask = cv2.threshold(mask, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 1:
        first_contour = contours[0]
        for index_contour in range(1, len(contours)):
            first_contour = np.concatenate((first_contour, contours[index_contour]), axis=0)
        return first_contour
    else:
        return contours[0]


def find_mask_biggest_contour(mask):
    mask = np.asarray(mask, dtype=np.uint8)
    mask = np.asarray(cv2.cvtColor(mask, cv2.COLOR_RGB2GRAY), dtype=np.uint8)

    ret, mask = cv2.threshold(mask, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours_area = list()
    for contour in contours:
        contours_area.append(cv2.contourArea(contour))

    object_contour = contours[contours_area.index(max(contours_area))]

    return object_contour


def remove_above_white(image):
    ones = np.ones((image.shape[0], image.shape[1]))

    R_channel = image[..., 0] > 255 * ones
    G_channel = image[..., 1] > 255 * ones
    B_channel = image[..., 2] > 255 * ones
    mask = np.array([R_channel * G_channel * B_channel])

    image = np.array([255]) * np.transpose(np.repeat(mask, [3], axis=0), axes=[1, 2, 0]) + \
            image * (1 - np.transpose(np.repeat(mask, [3], axis=0), axes=[1, 2, 0]))

    return image


def remove_black(image, color_threshold=(10, 10, 10)):
    ones = np.ones((image.shape[0], image.shape[1]))
    R_channel = image[..., 0] <= color_threshold[0] * ones
    G_channel = image[..., 1] <= color_threshold[1] * ones
    B_channel = image[..., 2] <= color_threshold[2] * ones
    black_mask = np.array([R_channel * G_channel * B_channel])
    base_image = np.ones(image.shape) * (np.array([255, 255, 255]))

    image_after_black_mask = base_image * np.transpose(np.repeat(black_mask, [3], axis=0), axes=[1, 2, 0]) + \
                             image * (1 - np.transpose(np.repeat(black_mask, [3], axis=0), axes=[1, 2, 0]))
    return image_after_black_mask


def insert_object(background, object_to_insert, x_coord, y_coord):
    ones = np.ones((object_to_insert.shape[0], object_to_insert.shape[1]))
    intensity_threshold = 254
    R_channel_ = object_to_insert[..., 0] <= intensity_threshold * ones
    G_channel_ = object_to_insert[..., 1] <= intensity_threshold * ones
    B_channel_ = object_to_insert[..., 2] <= intensity_threshold * ones

    inserted_object_mask = np.array([R_channel_ * G_channel_ * B_channel_])

    R_channel_ = background[y_coord:y_coord + object_to_insert.shape[0], x_coord:x_coord + object_to_insert.shape[1], 0] \
                 <= intensity_threshold * ones
    G_channel_ = background[y_coord:y_coord + object_to_insert.shape[0], x_coord:x_coord + object_to_insert.shape[1], 1] \
                 <= intensity_threshold * ones
    B_channel_ = background[y_coord:y_coord + object_to_insert.shape[0], x_coord:x_coord + object_to_insert.shape[1], 2] \
                 <= intensity_threshold * ones

    area_to_insert_mask = np.array([R_channel_ * G_channel_ * B_channel_])

    object_insert_mask = np.transpose(np.repeat(area_to_insert_mask * inserted_object_mask, [3], axis=0),
                                      axes=[1, 2, 0])

    background[y_coord:y_coord + object_to_insert.shape[0], x_coord:x_coord + object_to_insert.shape[1], :] -= \
        background[y_coord:y_coord + object_to_insert.shape[0], x_coord:x_coord + object_to_insert.shape[1], :] * \
        object_insert_mask

    background[y_coord:y_coord + object_to_insert.shape[0], x_coord:x_coord + object_to_insert.shape[1], :] += \
        object_to_insert

    R_channel = background[y_coord:y_coord + object_to_insert.shape[0], x_coord:x_coord + object_to_insert.shape[1],
                0] > 250 * ones
    G_channel = background[y_coord:y_coord + object_to_insert.shape[0], x_coord:x_coord + object_to_insert.shape[1],
                1] > 250 * ones
    B_channel = background[y_coord:y_coord + object_to_insert.shape[0], x_coord:x_coord + object_to_insert.shape[1],
                2] > 250 * ones
    mask = np.array([R_channel * G_channel * B_channel])

    background[y_coord:y_coord + object_to_insert.shape[0], x_coord:x_coord + object_to_insert.shape[1], :] = \
        background[y_coord:y_coord + object_to_insert.shape[0], x_coord:x_coord + object_to_insert.shape[1], :] - \
        np.array([255]) * np.transpose(np.repeat(mask, [3], axis=0), axes=[1, 2, 0])

    return background


def insert_mask(background, mask_to_insert, x_coord, y_coord):
    background[y_coord:y_coord + mask_to_insert.shape[0], x_coord:x_coord + mask_to_insert.shape[1],
    ...] = mask_to_insert
    return background


def get_object_bbox(contour):
    bbox_midpoint_format = cv2.boundingRect(contour)
    bbox_corner_format = [bbox_midpoint_format[0], bbox_midpoint_format[1],
                          bbox_midpoint_format[0] + bbox_midpoint_format[2],
                          bbox_midpoint_format[1] + bbox_midpoint_format[3]]
    return np.intp(np.asarray(bbox_corner_format, dtype=int))


def compute_iou(bbox_1, bbox_2):
    x_min = max(bbox_1[0], bbox_2[0])
    y_min = max(bbox_1[1], bbox_2[1])
    x_max = min(bbox_1[2], bbox_2[2])
    y_max = min(bbox_1[3], bbox_2[3])

    height = (y_max - y_min) if (y_max - y_min) > 0 else 0
    width = (x_max - x_min) if (x_max - x_min) > 0 else 0

    intersection = height * width

    union = np.abs((bbox_1[2] - bbox_1[0]) * (bbox_1[3] - bbox_1[1])) + \
            np.abs((bbox_2[2] - bbox_2[0]) * (bbox_2[3] - bbox_2[1])) - \
            intersection

    return intersection / (union + 0.00001)


def check_collisions(previous_objects_bboxes, new_object_bbox):
    for prev_bbox in previous_objects_bboxes:
        iou = compute_iou(prev_bbox, new_object_bbox)
        if iou > IOU_THRESHOLD:
            return False
    return True


def apply_rotation(image, angle):
    image = np.array(image, dtype=np.uint8)
    image = Image.fromarray(image)
    image = image.rotate(angle, fillcolor=(255, 255, 255), expand=True)

    return np.asarray(image)


def add_background(basic_background, real_background):
    intensity_threshold = 254

    zeros = np.ones((basic_background.shape[0], basic_background.shape[1]))
    R_channel = basic_background[..., 0] <= intensity_threshold * zeros
    G_channel = basic_background[..., 1] <= intensity_threshold * zeros
    B_channel = basic_background[..., 2] <= intensity_threshold * zeros
    black_mask = np.array([R_channel * G_channel * B_channel])

    image_no_black = basic_background * (np.transpose(np.repeat(black_mask, [3], axis=0), axes=[1, 2, 0])) + \
                     real_background * (1 - np.transpose(np.repeat(black_mask, [3], axis=0), axes=[1,
                                                                                                   2, 0]))
    return image_no_black


def generate_image(object_mask_applied, object_name_list, object_mask_list):
    basic_background = 255 * np.ones(shape=(IMAGE_HEIGHT, IMAGE_WIDTH, 3))

    real_background = reshape(cv2.imread("background.png"), w=IMAGE_WIDTH, h=IMAGE_HEIGHT)

    number_objects = random.randint(1, MAX_OBJECTS)

    object_class_list = list()
    each_object_mask = list()
    objects_bboxes = list()
    actual_num_objects = number_objects
    for obj_num in range(number_objects):
        time_begin = time.time()
        while True:
            if time.time() - time_begin > 2:
                actual_num_objects -= 1
                break
            alternative_background = 255 * np.ones(shape=(IMAGE_HEIGHT, IMAGE_WIDTH, 3))
            object_type = random.randint(0, len(object_mask_applied) - 1)

            angle = random.random() * 360
            rotated_object = apply_rotation(object_mask_applied[object_type], angle)
            rotated_mask = apply_rotation(object_mask_list[object_type], angle)

            if rotated_object.shape[0] > IMAGE_HEIGHT - 50:
                actual_num_objects -= 1
                break

            x_coord = random.randint(0, IMAGE_WIDTH - 20 - rotated_object.shape[1])
            y_coord = random.randint(20, IMAGE_HEIGHT - 20 - rotated_object.shape[0])

            alternative_background = 255 - insert_mask(alternative_background, rotated_mask, x_coord, y_coord)

            object_contour = find_mask_biggest_contour(255 - alternative_background)
            object_bbox = get_object_bbox(object_contour)

            if check_collisions(objects_bboxes, object_bbox):
                object_class_list.append(object_name_list[object_type])
                objects_bboxes.append(object_bbox)
                each_object_mask.append(alternative_background)
                basic_background = insert_object(basic_background, rotated_object, x_coord, y_coord)
                break

    object_contour_list = get_every_object_contour(each_object_mask)

    image = add_background(basic_background, real_background)
    image = np.asarray(image, dtype=np.uint8)
    # segmentation_mask = create_all_objects_mask(all_object_masks=each_object_mask, object_classes=object_class_list)

    # image = add_noise_blur_brightness(image)

    bboxes_yolo = object_boxes_yolo_format(objects_bboxes, classes=object_class_list)
    segmentation_mask, classes = create_all_objects_mask(all_object_masks=each_object_mask,
                                                         bboxes_yolo_format=bboxes_yolo)

    image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT))
    mask = cv2.resize(segmentation_mask, (IMAGE_WIDTH, IMAGE_HEIGHT))

    if VISUALIZE:
        bboxes_corner = object_boxes_to_corner_format(bboxes_yolo)
        image_vis = np.copy(image)
        for num_object in range(actual_num_objects):
            color = (random.randint(100, 255), random.randint(100, 255), random.randint(100, 255))

            image_vis = cv2.drawContours(image_vis, object_contour_list[num_object], -1, color, 2)
            image_vis = cv2.rectangle(image_vis, (bboxes_corner[num_object][0], bboxes_corner[num_object][1]),
                                      (bboxes_corner[num_object][2], bboxes_corner[num_object][3]), color, 2)
        cv2.imshow("image", image_vis)
        cv2.waitKey(0)

    # json_dict = create_json_dict(objects_classes=object_class_list, objects_contours=object_contour_list, image_number=image_number)
    # json_object = json.dumps(json_dict, indent=4)

    return image, mask, bboxes_yolo, classes


def mask_to_polygon(mask):
    # Ensure the mask is of data type np.uint8
    mask = mask.astype(np.uint8)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea)

    polygon_contour = []
    for i in range(0, len(largest_contour), 5):
        point = largest_contour[i]
        x, y = point[0]
        # Normalize coordinates to values between 0 and 1
        normalized_x = x / mask.shape[1]
        normalized_y = y / mask.shape[0]
        polygon_contour.append(str(round(normalized_x, 3)))
        polygon_contour.append(str(round(normalized_y, 3)))
        # polygon_contour.append((round(normalized_x, 3), round(normalized_y, 3)))
    return polygon_contour


def generate_dataset(custom_num_images=NUM_IMAGES_TO_GENERATE, segmentation=False):
    

    def generate_to_folder(subdir_name, n_images=NUM_IMAGES_TO_GENERATE):
        os.mkdir(DATASET_DIRECTORY + "images/" + subdir_name)
        os.mkdir(DATASET_DIRECTORY + "masks/" + subdir_name)
        os.mkdir(DATASET_DIRECTORY + "labels/" + subdir_name)
        for image_num in range(n_images):
            image, mask, bbox, classes = generate_image(object_mask_applied_list, object_name_list, object_mask_list)
            mask = np.array(mask, dtype=bool)
            cv2.imwrite(DATASET_DIRECTORY + "images/" + subdir_name + "/image_{}.png".format(image_num), image)
            with open(DATASET_DIRECTORY + "masks/" + subdir_name + "/mask_{}.npy".format(image_num), "wb") as f:
                np.save(f, mask)

            txt_file = open(DATASET_DIRECTORY + "labels/" + subdir_name + "/image_{}.txt".format(image_num), "w")
            for obj_index in range(len(bbox)):
                ############# Segmentation ###############
                if segmentation:
                    try:
                        polygon = mask_to_polygon(mask[:,:,obj_index])
                    except IndexError:
                        polygon = mask_to_polygon(mask)
                    line_segmentation = str(classes[obj_index]) + ' ' + ' '.join(polygon) + '\n'
                    txt_file.write(line_segmentation)
                ##########################################
                else:
                    line = str(bbox[obj_index][4]) + " " + str(bbox[obj_index][0]) + " " + str(
                        bbox[obj_index][1]) + " " + str(
                        bbox[obj_index][2]) + " " + str(bbox[obj_index][3]) + "\n"
                    txt_file.write(line)
            txt_file.close()
            print("IMAGE NUMBER {} IS SAVED".format(image_num))

    PATH_PICTURES = "objects_pictures_ordered/"
    PATH_MASKS = "objects_masks_ordered/"
    pictures_names_list = sorted(os.listdir(PATH_PICTURES))
    object_mask_applied_list = list()
    object_mask_list = list()
    object_name_list = list()

    DATASET_DIRECTORY = "GeneratedDataset/"

    try:
        os.mkdir(DATASET_DIRECTORY)
        os.mkdir(DATASET_DIRECTORY + "images")
        os.mkdir(DATASET_DIRECTORY + "masks")
        os.mkdir(DATASET_DIRECTORY + "labels")
    except FileExistsError:
        shutil.rmtree(DATASET_DIRECTORY)
        os.mkdir(DATASET_DIRECTORY)
        os.mkdir(DATASET_DIRECTORY + "images")
        os.mkdir(DATASET_DIRECTORY + "masks")
        os.mkdir(DATASET_DIRECTORY + "labels")

    for pict_name in pictures_names_list:
        mask_name = pict_name.split("_")[0] + "_" + pict_name.split("_")[1] + "_mask.png"

        object_pict = cv2.imread(PATH_PICTURES + pict_name)
        object_mask = cv2.imread(PATH_MASKS + mask_name) / 255
        reshape_ratio = int(1080 / IMAGE_HEIGHT)

        reshaped_object_pict = reshape(object_pict, w=int(object_pict.shape[1] / reshape_ratio),
                                       h=int(object_pict.shape[0] / reshape_ratio))
        reshaped_object_mask = reshape(object_mask, w=int(object_mask.shape[1] / reshape_ratio),
                                       h=int(object_mask.shape[0] / reshape_ratio))
        object_name_list.append(pict_name.split("_")[0])
        object_mask_list.append(255 - reshaped_object_mask * 255)

        if "trex" in mask_name:
            object_mask_applied_list.append(
                remove_black(reshaped_object_pict * reshaped_object_mask, color_threshold=(5, 5, 5)))
        else:
            object_mask_applied_list.append(remove_black(reshaped_object_pict * reshaped_object_mask))

    print('Generating training images')
    generate_to_folder("train", custom_num_images)
    print('Generating validation images')
    generate_to_folder("val", int(0.2 * custom_num_images))


if __name__ == "__main__":
    generate_dataset()