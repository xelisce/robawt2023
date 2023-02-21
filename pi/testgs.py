import cv2
from MultiThread import WebcamStream
import numpy as np
import matplotlib as plt

cam_stream = WebcamStream(0)
cam_stream.start()
frame_org = cam_stream.read()
width, height = frame_org.shape[1], frame_org.shape[0]

crop_h_hsv_gs = 0
crop_h_bw_gs = 0

l_green = np.array([0, 0, 0], np.uint8)
u_green = np.array([0, 0, 0], np.uint8)

def green_squares(frame_bw_gs):
    edges = cv2.Canny(frame_bw_gs, 100, 200)

while True:
    if cam_stream.stopped:
        break

    frame_org = cam_stream.read()
    frame_crop4hsvgs = frame_org[crop_h_hsv_gs:height, :]
    frame_crop4bwgs = frame_org[crop_h_bw_gs:height]
    frame_bw_gs = cv2.cvtColor(frame_crop4bwgs, cv2.COLOR_BGR2GRAY) #see whether can crop after converting to save processing power
    frame_hsv_gs = cv2.cvtColor(frame_crop4hsvgs, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv_gs, l_green, u_green)

    green_sum = np.sum(mask_green)

    print(green_sum)

    if green_sum > 20000:
        green_squares(frame_bw_gs)
