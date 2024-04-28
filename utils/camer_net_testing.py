import os
import time 


from ultralytics import YOLO
import cv2
from ultralytics.utils.plotting import Annotator
from pathlib import Path

from cv_pick_place.robot_cell.detection.realsense_depth import DepthCamera



camera = DepthCamera(config_path=Path("C:/Users/Testbed/CIIRC_DTU_3WEEK_NEW/cv_pick_place/config/D435_camera_config.json"))

# Get frames from camera



def run(frame,model):
    ouput = model(frame)
    return ouput

            

if __name__ == "__main__":
    model_to_use=model_to_use = f'runs/detect/train/weights/best.pt'

    try:
        model = YOLO(model_to_use)
        
    except FileNotFoundError:
        print("Model not found, YoloV8n")


    img_array =[]
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (1750, 25)
    fontScale = 1
    fontColor = (0, 0, 255)
    thickness = 2
    lineType = 2
    prev_frame_time= 0
    while True:
        success,frame_timestamp,depth_frame,rgb_frame,colorized_depth = camera.get_frames()
        if not success:
            continue
        img_frame = rgb_frame.copy()
        img_frame = cv2.resize(img_frame,(640,320))
        results = run(img_frame,model)
        annotated_frame = results[0].plot()
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        fps = str(int(fps))
        cv2.putText(annotated_frame, 'FPS: ' + fps,
                    bottomLeftCornerOfText,
                    font,
                    fontScale,
                    fontColor,
                    thickness,
                    lineType)

        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        img_array.append(annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        


    #     img_frame = rgb_frame.copy()
    #     img_frame = cv2.resize(img_frame,(640,320))
    #     data= run(img_frame,model)
    #     for r in data: 
            
    #         boxes = r.boxes
    #         annotator = Annotator(img_frame)
        
    #         for box in boxes:
                
    #             b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
    #             c = box.cls
    #             annotator.box_label(b, model.names[int(c)])
          
    #     img = annotator.result()  
    #     cv2.imshow('YOLO V8 Detection', img)     
    #     if cv2.waitKey(1) & 0xFF == ord(' '):
    #         break

        
    # cv2.destroyAllWindows()




# success, frame = cap.read()

#     if success:
#         # Run YOLOv8 inference on the frame
#         results = model(frame)

#         # Visualize the results on the frame
#         annotated_frame = results[0].plot()
        
#         # time when we finish processing for this frame 
#         new_frame_time = time.time()

#         # fps will be number of frame processed in given time frame 
#         # since their will be the most time error of 0.001 second 
#         # we will be subtracting it to get more accurate result 
#         fps = 1 / (new_frame_time - prev_frame_time)
#         prev_frame_time = new_frame_time

#         # converting the fps into integer 
#         fps = str(int(fps))

#         # putting the FPS count on the frame 
#         cv2.putText(annotated_frame, 'FPS: ' + fps,
#                     bottomLeftCornerOfText,
#                     font,
#                     fontScale,
#                     fontColor,
#                     thickness,
#                     lineType)

#         # Display the annotated frame
#         cv2.imshow("YOLOv8 Inference", annotated_frame)
#         img_array.append(annotated_frame)

#         # Break the loop if 'q' is pressed
#         if cv2.waitKey(1) & 0xFF == ord("q"):
#             break
#     else:
#         # Break the loop if the end of the video is reached
#         break