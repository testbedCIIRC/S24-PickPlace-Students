TRAIN =True
from object_detect.run import run
from object_detect.train import train_detect


def main():
    if(TRAIN==True):
        train_detect()



if __name__ == '__main__':
    main()