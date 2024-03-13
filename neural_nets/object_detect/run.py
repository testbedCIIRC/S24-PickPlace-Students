import os

from ultralytics import YOLO
import cv2
from ultralytics.utils.plotting import Annotator
from pathlib import Path



def choose_model(runs_dir:str)->str:
    """
     return latest run from directory
    """
    dirs= list(os.walk(runs_dir))
    
    return dirs[0][1][-1]

def run(rgb_image,model):
     
    cv2.imshow("test",rgb_image)
    ouput = model(rgb_image)
    
    return ouput

        


if __name__ == "__main__":
    run()
        
