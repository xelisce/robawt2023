import cv2
from MultiThread import WebcamStream
import serial
import numpy as np
# import struct
# from time import sleep
import math
import enum

#* CAMERA
lt_stream = WebcamStream(stream_id = 0)
lt_stream.start()
lt_frame = lt_stream.read()
width, height_org = lt_frame.shape[1], lt_frame.shape[0]
print("Line track camera width:", width, "Camera height:", height_org)

crop_h_bw = 93
height = height_org - crop_h_bw
# cam_x = width/2 #-1 to bias robot to the right
# cam_y = height - 1
# print(f"Line track image centred on ({cam_x}, {cam_y})")

#* SERIAL
ser = serial.Serial("/dev/ttyS0", 9600)

#* COLOR CALIBRATION
# l_black = 0
u_black = 70

l_green = np.array([30, 50, 60], np.uint8)
u_green = np.array([85, 255, 255], np.uint8)

l_red1 = np.array([0, 100, 100], np.uint8) #! untuned red values
u_red1 = np.array([15, 255, 255], np.uint8)
l_red2 = np.array([170, 100, 100], np.uint8) 
u_red2 = np.array([180, 255, 255], np.uint8) #! 179 or 180?

#* CONSTANTS CALIBRATION
gs_roi_h = 300 #the crop height for gs #! increase after tuning 
gs_bksampleoffset = 10 #offset sample above green squares
gs_bksampleh = 40
gs_minbkpct = 0.35 #! to be tuned
gs_minarea = 4000000 #? consider making this scaled by pixels (/255)

rpm = 100
kp = 1.5

#* IMAGE PROCESSING
x_com = np.tile(np.linspace(-1., 1., width), (height, 1)) #reps is (outside, inside)
y_com = np.array([[i] * width for i in np.linspace(1., 1/height, height)]) #1/height is just to save pixels
x_com_scale = ((1-y_com) ** 0.6)
x_com *= x_com_scale

gs_erode_kernel = np.ones((3, 3), np.uint8)

#* LOGIC SETUP
class Task(enum.Enum):
    EMPTY = 0
    LEFT_GREEN = 1
    RIGHT_GREEN = 2
    DOUBLE_GREEN = 3

curr = Task.EMPTY

#* BEGIN OF LINETRACK LOOP CODE

while True:
    if lt_stream.stopped:
        break

    #* IMAGE SETUP

    frame_org = lt_stream.read()
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_green, u_green)
    mask_gs = mask_green[gs_roi_h:, :]
    cv2.imshow("green square mask", frame_org[gs_roi_h:, :])
    cv2.imshow("green square mask", mask_gs) #& debug green square mask
    gs_sum = np.sum(mask_gs)
    print(gs_sum) #& debug green min area

    mask_black = cv2.inRange(frame_gray, 0, u_black) - mask_green
    # cv2.imshow('black frame', black_mask) #& debug black mask

    # mask_red1 = cv2.inRange(frame_hsv, l_red1, u_red1)
    # mask_red2 = cv2.inRange(frame_hsv)
    # mask_red = mask_red1 + mask_red2

    #* RED LINE 
    #TODO

    #* GREEN SQUARES

    if gs_sum > gs_minarea:
        mask_gs = cv2.erode(mask_gs, gs_erode_kernel, iterations=1)
        mask_gs = cv2.dilate(mask_gs, gs_erode_kernel, iterations=1)
        cv2.imshow("after erosion and dilation", mask_gs) #& debug green square mask

        #~ Find x positions of green
        green_col = np.amax(mask_gs, axis=0)
        g_indices_h = np.where(green_col==255) #h for horiontal
        gs_left = g_indices_h[0][0]
        gs_right = g_indices_h[0][-1]
        # print("left:", gs_left, "right:", gs_right)
        # gs_indices_h

        #~ Find y position of green (Can cut processing time here by putting fixed constant instead)
        green_row = np.amax(mask_gs, axis=1)
        g_indices_v = np.where(green_row==255) #v for vertical
        gs_top = g_indices_v[0][0] + gs_roi_h
        # cv2.imshow("Green bounding box", frame_org[gs_top:, gs_left:gs_right]) #& debug green bounds

        #~ Test if below or above line
        gs_bkabove = mask_black[gs_top - gs_bksampleoffset - gs_bksampleh : gs_top - gs_bksampleoffset, gs_left : gs_right]
        gs_bkpct = np.sum(gs_bkabove) / 255 / gs_bksampleh / (gs_right - gs_left)
        # print("Percentage of black above green:", gs_bkpct) #& debug green's black
        # cv2.imshow("black area above green square", gs_bkabove) 
        # cv2.imshow("black area above green in orig frame", frame_org[gs_top - gs_bksampleoffset - gs_bksampleh : gs_top - gs_bksampleoffset, gs_left : gs_right])
        
        #~ GREEM SQUARE FOUND
        if gs_bkpct > gs_minbkpct:

            #~ Find x position of green
            gs_centre = (gs_left + gs_right) / 2

            #~ Find x position of black
            gs_bkbeside = mask_black[gs_roi_h:, :]
            blackM = cv2.moments(gs_bkbeside)
            cx_black = int(blackM["m10"]/blackM["m00"]) if np.sum(gs_bkbeside) else 0 #theoretically divide by zero error should never happen
            # cv2.imshow("Black beside green squares", gs_bkbeside) #& debug green type triggered
            print("Black x-centre:", cx_black)
            print("Green x-centre:", gs_centre)

            #~ Identify type of green square
            if cx_black > gs_left and cx_black < gs_right and gs_sum > 2 * gs_minarea:
                curr = Task.DOUBLE_GREEN
            elif cx_black > gs_centre:
                curr = Task.LEFT_GREEN
            elif cx_black < gs_centre:
                curr = Task.RIGHT_GREEN

            to_pico = [253, curr.value]
            # else:
                # print("ERROR: Green found but type indetermined")

    #* LINETRACK

    else:
        curr = Task.EMPTY
        mask_black = mask_black[crop_h_bw:, :]
        #? Maybe needed, not critical
        # black_mask = cv2.erode(black_mask, kernel) #? maybe try kernel
        # black_mask = cv2.dilate(black_mask, kernel)

        #~ Vectorizing the black components
        y_black = cv2.bitwise_and(y_com, y_com, mask = mask_black)
        x_black = cv2.bitwise_and(x_com, x_com, mask = mask_black)
        # cv2.imshow("yframe", y_black)
        # cv2.imshow("xframe", x_black)

        #? Consider accounting for line gap later on
        y_resultant = np.mean(y_black)
        x_resultant = np.mean(x_black)
        # print(y_resultant, x_resultant) #& debug resultant angle

        #~ Formatting data for transfer
        angle = 90 - (math.atan2(y_resultant, x_resultant) * 180/math.pi) if y_resultant != 0 else 0
        pidangle = angle * kp

        if pidangle > 90:
            pidangle = 90
        elif pidangle < -90:
            pidangle = -90
        rotation = int(pidangle) + 90
        # print(rotation)

        to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm,
                253, curr.value] # 0 to 200
                # 253, curr] # 0 to 3 currently

    print(curr)
        
    #* SEND DATA
    
    # ser.write(to_pico)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

lt_stream.stop()
cv2.destroyAllWindows()
