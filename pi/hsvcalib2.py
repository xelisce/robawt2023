# THIS FILE IS TO BE KEPT ON COMPUTER

import cv2
import numpy as np

raw = cv2.imread('hsvcalibimage.jpg')
frame_hsv = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)

cv2.namedWindow("raw frame", cv2.WINDOW_NORMAL)
cv2.namedWindow("masked frame", cv2.WINDOW_NORMAL)

#l_hsv = np.array([0, 0, 0], np.uint8)
#u_hsv = np.array([0, 0, 0], np.uint8)
l_red1 = np.array([0, 0, 0], np.uint8)
u_red1 = np.array([10, 0, 0], np.uint8)
l_red2 = np.array([170, 0, 0], np.uint8)
u_red2 = np.array([180, 0, 0], np.uint8) 

def callback(x):
    global l_hsv, u_hsv
    #Mask 1
    l_red1[0] = cv2.getTrackbarPos('L1 HUE', 'controls')
    l_red1[1] = cv2.getTrackbarPos('L1 SAT', 'controls')
    l_red1[2] = cv2.getTrackbarPos('L1 VAL', 'controls')
    u_red1[0] = cv2.getTrackbarPos('U1 HUE', 'controls')
    u_red1[1] = cv2.getTrackbarPos('U1 SAT', 'controls')
    u_red1[2] = cv2.getTrackbarPos('U1 VAL', 'controls')
    #Mask 2
    l_red2[0] = cv2.getTrackbarPos('L2 HUE', 'controls')
    l_red2[1] = cv2.getTrackbarPos('L2 SAT', 'controls')
    l_red2[2] = cv2.getTrackbarPos('L2 VAL', 'controls')
    u_red2[0] = cv2.getTrackbarPos('U2 HUE', 'controls')
    u_red2[1] = cv2.getTrackbarPos('U2 SAT', 'controls')
    u_red2[2] = cv2.getTrackbarPos('U2 VAL', 'controls')
    #mask = cv2.inRange(frame_hsv, l_hsv, u_hsv)

    _mask_red1 = cv2.inRange(frame_hsv, l_red1, u_red1)
    _mask_red2 = cv2.inRange(frame_hsv, l_red2, u_red2)
    mask_red = _mask_red1 + _mask_red2
    res = cv2.bitwise_and(frame_hsv, frame_hsv, mask=mask_red)
    cv2.imshow('raw frame', raw)
    cv2.imshow('masked frame', res)
 
cv2.namedWindow('controls', 2)
cv2.resizeWindow('controls', 550, 10) #why's the height 10 pixels lol

cv2.createTrackbar('L1 HUE', 'controls', 0, 180, callback)
cv2.createTrackbar('L1 SAT', 'controls', 80, 255, callback)
cv2.createTrackbar('L1 VAL', 'controls', 60, 255, callback)
cv2.createTrackbar('U1 HUE', 'controls', 15, 180, callback)
cv2.createTrackbar('U1 SAT', 'controls', 255, 255, callback)
cv2.createTrackbar('U1 VAL', 'controls', 200, 255, callback)

cv2.createTrackbar('L2 HUE', 'controls', 176, 180, callback)
cv2.createTrackbar('L2 SAT', 'controls', 0, 255, callback)
cv2.createTrackbar('L2 VAL', 'controls', 61, 255, callback)
cv2.createTrackbar('U2 HUE', 'controls', 180, 180, callback)
cv2.createTrackbar('U2 SAT', 'controls', 255, 255, callback)
cv2.createTrackbar('U2 VAL', 'controls', 255, 255, callback)


key = cv2.waitKey(0)

while key != ord('q'):
    key = cv2.waitKey(0)

cv2.destroyAllWindows()


