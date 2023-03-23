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

balls = []

dp = 1.2
min_dist = 70
param1 = 184
param2 = 28
min_radius = 40
max_radius = 170

def callback(val):
    global dp, min_dist, param1, param2, min_radius, max_radius
    # dp = int(cv2.getTrackbarPos('dp', 'controls'))/10
    min_dist = int(cv2.getTrackbarPos('min_dist', 'controls'))
    param1 = int(cv2.getTrackbarPos('param1', 'controls'))
    param2 = int(cv2.getTrackbarPos('param2', 'controls'))
    min_radius = int(cv2.getTrackbarPos('min_radius', 'controls'))
    max_radius = int(cv2.getTrackbarPos('max_radius', 'controls'))

cv2.namedWindow('controls', 2)
cv2.resizeWindow('controls', 550, 10)
# cv2.createTrackbar('dp*10', 'controls', 12, 20, callback)
cv2.createTrackbar('min_dist', 'controls', 70, 200, callback)
cv2.createTrackbar('param1', 'controls', 184, 500, callback)
cv2.createTrackbar('param2', 'controls', 28, 180, callback)
cv2.createTrackbar('min_radius', 'controls', 40, 255, callback)
cv2.createTrackbar('max_radius', 'controls', 170, 255, callback)
#old values 1.2, 70, 200, 27, 40, 100

#* IN LOOP

while True:
    if evac_stream.stopped:
        break

    evac_org = evac_stream.read()

    #print("og shape:", evac_org.shape[1], evac_org.shape[0])
    evac_hsv = cv2.cvtColor(evac_org, cv2.COLOR_BGR2HSV)
    evac_gray = cv2.cvtColor(evac_org, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("gray frame", evac_gray)
    evac_max = np.amax(evac_org, axis=2) #to get max "intensity/saturation" of every pixel (RGB)

    # evac_sat = evac_hsv[:, :, 1]

    evac_sat_mask = cv2.inRange(evac_hsv, u_sat_thresh, l_sat_thresh)
    evac_gray = cv2.bitwise_and(evac_gray, evac_gray, mask=evac_sat_mask)
    evac_max = cv2.bitwise_and(evac_max, evac_max, mask=evac_sat_mask)

    evac_gray = evac_gray[crop_h:, :]
    circles = cv2.HoughCircles(evac_gray, cv2.HOUGH_GRADIENT, dp, min_dist, param1 = param1 , param2=param2, minRadius= min_radius, maxRadius=max_radius)
    edge_gray = cv2.Canny(evac_gray, int(param1/2), param1)
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
    #^ (I still have no idea what dp does btw) it changes the resolution of the 3d space for the accumulator
    #^ 1. min and max radius
    #^ 2. param1, which controls the sensitivity of the edge detection (gradient value); a higher value will reduce no. of circles detected
    #^ 3. param2, which controls the threshold for circle detection; a larger value will reduce no. of circles detected
    #^ Prev values were param1 = 200, param2 = 27
    #~ Detection of circles
    circles = cv2.HoughCircles(evac_max, cv2.HOUGH_GRADIENT, dp, min_dist, param1 = param1 , param2=param2, minRadius= min_radius, maxRadius=max_radius)
    balls = []
    edge_max = cv2.Canny(evac_max, int(param1/2), param1)
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
    combined_edge = np.hstack((edge_gray, edge_max))
    combinedDown_edge = cv2.pyrDown(combined_edge)
    cv2.imshow("both", combinedDown)
    cv2.imshow("edges", combinedDown_edge)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

evac_stream.stop()
cv2.destroyAllWindows()