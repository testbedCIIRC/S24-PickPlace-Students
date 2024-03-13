
import os 
import shutil


DATASET_ROOT_PATH=os.path.join("cv_pick_place/neural_nets/Dataset","GeneratedDataset")



def dataset_check(generate_dataset = True,generate_video = False) ->bool:
    """
    params:
    generate_new :  to generate new dataset 
    generate video : generates video for validation 
    
    """

    
    if(generate_dataset):
        if(os.path.isdir(DATASET_ROOT_PATH)):
             print("Deleting old dataset...")
        else:
            print("No data set found ")
            
            return True
        #remove the old dataset
        for filename in os.listdir(DATASET_ROOT_PATH):
            file_path = os.path.join(DATASET_ROOT_PATH, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)

            except Exception as e:
                print('Failed to delete %s. Reason: %s' % (file_path, e))
                return False
        
        return True
    if(generate_video):
        return True
    else:
        return False 

    
    

   
