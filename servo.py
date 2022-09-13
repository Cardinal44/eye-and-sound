import serial
import detection
import cv2
import numpy as np
import time
import winsound

usbport = 'COM6'  # usb from arduino


arduino = serial.Serial(usbport, 9600, timeout=0)

def recognize(dist):
    time.sleep(1)

    rad = 0

    cap = cv2.VideoCapture(0)
    while(float(dist) < 5):
        _, img = cap.read()
        scaling_factor = 1
        img = cv2.flip(img, 1)

        img = cv2.resize(img, None, fx=scaling_factor,
                            fy=scaling_factor, interpolation=cv2.INTER_AREA)

        eyes_ano = detection.findeye(img)

        gray = cv2.cvtColor(~img, cv2.COLOR_BGR2GRAY)

        kernel = np.ones((5, 5), np.uint8)
        #gray = cv2.dilate(gray,kernel,iterations = 1)
        #gray = cv2.erode(gray,kernel,iterations=1)

        for x, y, w, h in eyes_ano:
            cv2.circle(img, (x+(w//2), y+(h//2)),
                        (h+w)//2, (255, 0, 0), 1)

        ret, thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)
        thresh = cv2.dilate(thresh, kernel, iterations=1)
        thresh = cv2.erode(thresh, kernel, iterations=1)
        cv2.imshow("thresh", thresh)

        count = 0

        for x, y, w, h in eyes_ano:
            count += 1
            cv2.circle(img, (x+(w//2), y+(h//2)),
                        (h+w)//4, (255, 0, 0), 1)
            # cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),5)
            contours, hierarchy = cv2.findContours(
                thresh[y:y+h, x:x+w], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if len(contours) != 0:
                contour = max(contours, key=cv2.contourArea)
                rec = cv2.boundingRect(contour)
                x1, y1, w1, h1 = rec
                if (50 < cv2.contourArea(contour)) and (w1/h1 < 1.5):
                    # cv2.rectangle(img,(x1+x,y1+y),(x1+x+w1,y1+y+h1),(0,255,255),2)
                    radius = int(0.1 * (w1 + h1))
                    rad += radius
                    cv2.circle(img, (x1+x+(w1//2), y1+y + \
                                (h1//2)), radius, (0, 0, 255), 3)
                    count += 1

        if count < 2:
            duration = 200

            # milliseconds
            freq = 1000  # Hz
            winsound.Beep(freq, duration)

        var = count

        cv2.imshow("eyes", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    cap.release()

distances = []
while (True):
    if (arduino.inWaiting() > 0):
        myData = arduino.readline()
        dist = (myData.decode().strip('\r\n'))
        print(dist)
        recognize(dist)
