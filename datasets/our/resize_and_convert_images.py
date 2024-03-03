import os
import cv2

input_folder = 'orig'
output_folder = 'converted'

# Make sure the output folder exists
if not os.path.exists(output_folder):
    os.makedirs(output_folder)


# Traverse all the png files in input folder
for filename in os.listdir(input_folder):
    if filename.endswith('.png'):

        file_path = os.path.join(input_folder, filename)
        output_filename = os.path.splitext(filename)[0] + '.jpg'
        output_path = os.path.join(output_folder, output_filename)

        image = cv2.imread(file_path)

        # Crop out the conveyor belt
        image = image[63:-93, :-50]

        print(f'{file_path} -> {output_path}')
        cv2.imwrite(output_path, image)


