
import time 
import os
import platform
from pathlib import Path
import shutil

from ultralytics import YOLO
import torch


from dataset.dataset_check import dataset_check
from dataset.GenerateSyntheticDataset import generate_dataset

# device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

EPOCHS = 30
GENERATE_NR_IMAGES = 10000
GENERATE_NW_DS=True






def train_detect(mode = "Segment",yaml_data = Path("cv_pick_place/neural_nets/data.yaml")):
        # Load already trained weights
        
        try:
             os.path.isfile(yaml_data)
        except FileNotFoundError:
            print(f"Yaml file is missing {yaml_data}")
        ###########
        if(dataset_check(generate_dataset=GENERATE_NW_DS)):
            generate_dataset(GENERATE_NR_IMAGES,segmentation=True)
        
        if(mode == "Detect"):
            model_to_use = f'runs/detect/train1/weights/best.pt'  # use 'yolov8n.pt' to start fresh
            try:
                model = YOLO(model_to_use)
            except FileNotFoundError:
                print("Model not found, retraining from scratch with YoloV8n")
                model = YOLO('yolov8n.pt')
        if(mode == "Segment"):
            model_to_use = f'runs/seg/train1/weights/best.pt'  # use 'yolov8n.pt' to start fresh
            try:
                model = YOLO(model_to_use)
            except FileNotFoundError:
                print("Model not found, retraining from scratch with YoloV8n")
                model = YOLO('yolov8n-seg.pt')
             
       
        
        
        #check if using cuda
        if torch.cuda.is_available():
            print("Using GPU")
            model.cuda()
            
        
        start = time.time()
        # if curr_os == 'Windows':
        #     print('Using Windows')
        results = model.train(data=yaml_data, epochs=EPOCHS)
        # else:
        #     print('Using Linux')
        #     results = model.train(data='generated_dataset.yaml', epochs=EPOCHS)
        end = time.time()
        
        # Save the results
        
        # results['Epochs'] = EPOCHS
        # results['Time training'] = end - start
        print("Training finished")

    