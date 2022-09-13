import cv2
from cv2 import CascadeClassifier
import numpy as np




eyedetect=cv2.CascadeClassifier("D:\\vs\\Hackathon2.0\\haarcascade_eye.xml")


def findeye(frame):

    detectionobj=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
    eyes_anotation=eyedetect.detectMultiScale(detectionobj,1.1,10)
    return eyes_anotation


