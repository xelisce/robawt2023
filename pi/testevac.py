import cv2
from MultiThread import WebcamStream
import numpy as np

evac_stream = WebcamStream(stream_id=2)
evac_stream.start()
evac_org = evac_stream.read()
width, height_org = evac_org.shape[1], evac_org.shape[0]

crop_h = 183

height = height_org - crop_h
u_black = 55

u_sat_thresh = np.array([0, 0, 0], np.uint8)
l_sat_thresh = np.array([180, 100, 255], np.uint8)

#* IN LOOP

while True:
    if evac_stream.stopped:
        break

    evac_org = evac_stream.read()

    #print("og shape:", evac_org.shape[1], evac_org.shape[0])
    evac_hsv = cv2.cvtColor(evac_org, cv2.COLOR_BGR2HSV)
    evac_gray = cv2.cvtColor(evac_org, cv2.COLOR_BGR2GRAY)
    evac_max = np.amax(evac_org, axis=2) #to get max "intensity/saturation" of every pixel (RGB)

    # evac_sat = evac_hsv[:, :, 1]

    evac_sat_mask = cv2.inRange(evac_hsv, u_sat_thresh, l_sat_thresh)
    evac_gray = cv2.bitwise_and(evac_gray, evac_gray, mask=evac_sat_mask)
    evac_max = cv2.bitwise_and(evac_max, evac_max, mask=evac_sat_mask)

    evac_gray = evac_gray[crop_h:, :]
    circles = cv2.HoughCircles(evac_gray, cv2.HOUGH_GRADIENT, 1.2, 70, param1 = 200 , param2 =27, minRadius= 40, maxRadius=100)
    balls = []
    if circles is not None:
        for x, y, r in circles[0]:
            mask = np.zeros(evac_org.shape[:2], dtype=np.uint8)
            mask = cv2.circle(mask, (int(x),int(y)), int(r), 255, -1)
            balls.append({
                    "x": x,
                    "y": y,
                    "r": r,
                })
            
        # print(balls)

        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(evac_gray,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(evac_gray,(i [0],i[1]),2,(0,0,255),3)

    evac_max = evac_max[crop_h:, :]

    #^ DOM: To finetune the detection of circles(reduce false positives), can consider changing the following params:
    #^ (I still have no idea what dp does btw)
    #^ 1. min and max radius
    #^ 2. param1, which controls the sensitivity of the edge detection (gradient value); a higher value will reduce no. of circles detected
    #^ 3. param2, which controls the threshold for circle detection; a larger value will reduce no. of circles detected
    #^ Prev values were param1 = 200, param2 = 27
    #~ Detection of circles
    circles = cv2.HoughCircles(evac_max, cv2.HOUGH_GRADIENT, 1.2, 70, param1=273, param2=30, minRadius=40, maxRadius=100)
    balls = []
    if circles is not None:
        for x, y, r in circles[0]:
            mask_max = np.zeros(evac_max.shape[:2], dtype=np.uint8)
            mask_max = cv2.circle(mask_max, (int(x),int(y)), int(r), 255, -1)
            balls.append({
                    "x": x,
                    "y": y,
                    "r": r,
                })
            
        # print(balls)

        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(evac_max,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(evac_max,(i [0],i[1]),2,(0,0,255),3)
            
    # cv2.imshow('detected circles',evac_gray)

    combined = np.hstack((evac_gray, evac_max))
    combinedDown = cv2.pyrDown(combined)
    cv2.imshow("both", combinedDown)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

evac_stream.stop()
cv2.destroyAllWindows()