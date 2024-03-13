import cv2
import os


def generate_video( image_folder = 'real_footage/Run1/color', video_name = 'real_footage/Run1/run1.mp4')->None:
   
    """
    Generates video for validation
    """
    
    images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    images.sort()
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape
    
    fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    
    video = cv2.VideoWriter(video_name, fourcc, 20.0, (width, height))
    
    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))
    
    video.release()