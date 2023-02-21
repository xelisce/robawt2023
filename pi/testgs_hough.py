import cv2
from MultiThread import WebcamStream
import numpy as np
import math

height = 480

crop_h_hsv_gs = 0
crop_h_bw_gs = 0

l_green = np.array([30, 90, 60], np.uint8)
u_green = np.array([85, 255, 255], np.uint8)

def green_squares(frame_bw_gs, frame_hsv_gs):
    edges = cv2.Canny(frame_bw_gs, 100, 210)
    lines = cv2.HoughLines(edges, 1, np.pi/180, 205)

    cdst = cv2.cvtColor(frame_bw_gs, cv2.COLOR_GRAY2BGR)

    print(lines)

    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    
    return cdst

frame_org = cv2.imread("hsvcalibimage.jpg")
frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

mask_green = cv2.inRange(frame_hsv, l_green, u_green)
if np.sum(mask_green) > 1000000:
    cdst = green_squares(frame_gray, frame_hsv)

cv2.imshow("frame_org", frame_org)
cv2.imshow("Detected Lines", cdst)

cv2.waitKey()

