import cv2
from MultiThread import WebcamStream
import serial
import numpy as np
# import struct
# from time import sleep
import math
import enum

#* CAMERA
bot_stream = WebcamStream(stream_id=0)
bot_stream.start()
bot_frame = bot_stream.read()
width, height_org = bot_frame.shape[1], bot_frame.shape[0]
print("Bottom camera width:", width, "Camera height:", height_org)

#* SERIAL
ser = serial.Serial("/dev/serial0", 9600)

#* COLOR CALIBRATION
# l_black = 0
u_black = 79

l_blue = np.array([96, 170, 80], np.uint8)
u_blue = np.array([106, 245, 191], np.uint8) #! blue needs more tuning to account for lighting 

l_green = np.array([30, 50, 60], np.uint8)
u_green = np.array([85, 255, 255], np.uint8)

l_red1 = np.array([0, 100, 80], np.uint8) 
u_red1 = np.array([15, 255, 255], np.uint8)
l_red2 = np.array([170, 100, 80], np.uint8) 
u_red2 = np.array([180, 255, 255], np.uint8)

#* CONSTANTS CALIBRATION
#! tune green squares to be mainly by height
gs_roi_h = 300 #the crop height for gs #! increase after tuning  
gs_bksampleoffset = 10 #to offset sample above green squares
gs_bksampleh = 40
gs_minbkpct = 0.35 #! to be tuned properly #DOM: this is gs_minimum_black_percentage
gs_minarea = 4000000 #^ consider making this scaled by pixels (/255)

while True:
    if bot_frame.stopped:
        break

    #* IMAGE SETUP

    bot_frame = bot_stream.read()
    frame_gray = cv2.cvtColor(bot_frame, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(bot_frame, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_green, u_green)
    mask_gs = mask_green[gs_roi_h:, :]
    # cv2.imshow("green square mask", frame_org[gs_roi_h:, :])
    # cv2.imshow("green square mask", mask_gs) #& debug green square mask
    gs_sum = np.sum(mask_gs)
    # print(gs_sum) #& debug green min area

    mask_black = cv2.inRange(frame_gray, 0, u_black) - mask_green
    # cv2.imshow('black frame', black_mask) #& debug black mask
    # print(np.sum(mask_black), curr.name)

    mask_red1 = cv2.inRange(frame_hsv, l_red1, u_red1)
    mask_red2 = cv2.inRange(frame_hsv, l_red2, u_red2)
    mask_red = mask_red1 + mask_red2
    # print(np.sum(mask_red)) #& debug red min area

    #* LINETRACK

    mask_black = mask_black[crp_:, :]