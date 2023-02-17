import cv2
from MultiThread import WebcamStream
import serial
import numpy as np
# import struct
# from time import sleep
import math

#* CAMERA
lt_stream = WebcamStream(stream_id=0)
lt_stream.start()
lt_frame = lt_stream.read()
width, height_org = lt_frame.shape[1], lt_frame.shape[0]
print("Line track camera width:", width, "Camera height:", height_org)

crop_h = 93
height = height_org - crop_h
# cam_x = width/2 #-1 to bias robot to the right
# cam_y = height - 1
# print(f"Line track image centred on ({cam_x}, {cam_y})")

#* SERIAL
ser = serial.Serial("/dev/ttyS0", 9600) #TODO

#* IMAGE PROCESSING
# l_black = 0
u_black = 90

x_com = np.tile(np.linspace(-1., 1., width), (height, 1)) #reps is (outside, inside)
y_com = np.array([[i] * width for i in np.linspace(1., 1/height, height)]) #1/height is just to save pixels
x_com_scale = ((1-y_com) ** 0.6)
x_com *= x_com_scale

#* BOT CONFIGURATIONS
speed = 0.5
kp = 1.5

while True:
    if lt_stream.stopped:
        break

    #* IMAGE SETUP
    frame_org = lt_stream.read()
    frame = frame_org[crop_h:height_org, :]
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    black_mask = cv2.inRange(frame_gray, 0, u_black)
    # cv2.imshow('black frame', black_mask)
    #? Maybe needed, not critical
    # black_mask = cv2.erode(black_mask, kernel) #TODO kernel
    # black_mask = cv2.dilate(black_mask, kernel)
    y_black = cv2.bitwise_and(y_com, y_com, mask = black_mask)
    x_black = cv2.bitwise_and(x_com, x_com, mask = black_mask)
    # cv2.imshow("yframe", y_black)
    # cv2.imshow("xframe", x_black)

    #* LINETRACK
    #? Consider accounting for line gap later on
    y_resultant = np.mean(y_black[y_black!=0])
    x_resultant = np.mean(x_black[x_black!=0])


    angle = 90 - (math.atan2(y_resultant, x_resultant) * 180/math.pi) if y_resultant != 0 else 0
    pidangle = angle * kp

    # print(y_resultant, x_resultant)

    if pidangle > 90:
        pidangle = 90
    elif pidangle < -90:
        pidangle = -90

    rotation = int(pidangle) + 90
    # print(rotation)

    #* SEND DATA
    to_pico = [rotation] #!choose speed = float or 0-100
    ser.write(to_pico) #TODO

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

lt_stream.stop()
cv2.destroyAllWindows()
