import math
from pyparsing import And
import detection
import cv2
from cv2 import boundingRect
import numpy as np
import serial
import time


time.sleep(1)

rad=0

cap = cv2.VideoCapture(0)
while(1):

    _, img = cap.read()
    scaling_factor = 1
    img=cv2.flip(img,1)

    img = cv2.resize(img, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_AREA)

    eyes_ano=detection.findeye(img)

    gray = cv2.cvtColor(~img, cv2.COLOR_BGR2GRAY)

    for x,y,w,h in eyes_ano:
        cv2.circle(img,(x+(w//2),y+(h//2)),(h+w)//2,(255,0,0),1)
    ret, thresh = cv2.threshold(gray,125, 255, cv2.THRESH_BINARY)
    #cv2.imshow("thresh",thresh)



    count=0
   
    for x,y,w,h in eyes_ano:
        count+=1
        cv2.circle(img,(x+(w//2),y+(h//2)),(h+w)//4,(255,0,0),1)
        #cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),5)
        
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200


        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1500

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        # Create a detector with the parameters
        # OLD: detector = cv2.SimpleBlobDetector(params)
        detector = cv2.SimpleBlobDetector_create(params)


        # Detect blobs.
        keypoints = detector.detect(img)




        img = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                



    var=count

   
        
   
    cv2.imshow("eyes",img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
    


cv2.destroyAllWindows()
cap.release()


