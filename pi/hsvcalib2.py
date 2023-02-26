# THIS FILE IS TO BE KEPT ON COMPUTER

import cv2
import numpy as np

raw = cv2.imread('hsvcalibimage.jpg')
frame_hsv = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)

cv2.namedWindow("raw frame", cv2.WINDOW_NORMAL)
cv2.namedWindow("masked frame", cv2.WINDOW_NORMAL)

l_hsv = np.array([0, 0, 0], np.uint8)
u_hsv = np.array([0, 0, 0], np.uint8)

def callback(x):
    global l_hsv, u_hsv
    l_hsv[0] = cv2.getTrackbarPos('L HUE', 'controls')
    l_hsv[1] = cv2.getTrackbarPos('L SAT', 'controls')
    l_hsv[2] = cv2.getTrackbarPos('L VAL', 'controls')
    u_hsv[0] = cv2.getTrackbarPos('U HUE', 'controls')
    u_hsv[1] = cv2.getTrackbarPos('U SAT', 'controls')
    u_hsv[2] = cv2.getTrackbarPos('U VAL', 'controls')
    mask = cv2.inRange(frame_hsv, l_hsv, u_hsv)
    res = cv2.bitwise_and(frame_hsv, frame_hsv, mask=mask)
    cv2.imshow('raw frame', raw)
    cv2.imshow('masked frame', res)
    

cv2.namedWindow('controls', 2)
cv2.resizeWindow('controls', 550, 10)

cv2.createTrackbar('L HUE', 'controls', 0, 180, callback)
cv2.createTrackbar('L SAT', 'controls', 0, 255, callback)
cv2.createTrackbar('L VAL', 'controls', 0, 255, callback)
cv2.createTrackbar('U HUE', 'controls', 179, 180, callback)
cv2.createTrackbar('U SAT', 'controls', 255, 255, callback)
cv2.createTrackbar('U VAL', 'controls', 255, 255, callback)

key = cv2.waitKey(0)

while key != ord('q'):
    key = cv2.waitKey(0)

cv2.destroyAllWindows()


