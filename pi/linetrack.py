import cv2
from MultiThread import WebcamStream
import serial
import numpy as np
# import struct
import time
import math
import enum

#* CAMERA
lt_stream = WebcamStream(stream_id = 0)
lt_stream.start()
lt_frame = lt_stream.read() 
width, height_org = lt_frame.shape[1], lt_frame.shape[0]
print("Line track camera width:", width, "Camera height:", height_org)

crop_h_bw = 93
crop_h_bw_gap = 110
height = height_org - crop_h_bw
# cam_x = width/2 #-1 to bias robot to the right
# cam_y = height - 1
# print(f"Line track image centred on ({cam_x}, {cam_y})")

#* SERIAL
ser = serial.Serial("/dev/serial0", 9600)

def receive_pico(ser) -> int:
    if(ser.in_waiting > 0):
        received_data = ser.read()
        data_left = ser.inWaiting()
        received_data += ser.read(data_left)
        return ord(received_data)
    else:
        return 0

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
gs_roi_h = 300 #the crop height for gs #! increase after tuning  
gs_bksampleoffset = 10 #to offset sample above green squares
gs_bksampleh = 40
gs_minbkpct = 0.35 #! to be tuned properly #DOM: this is gs_minimum_black_percentage
gs_minarea = 4000000 #^ consider making this scaled by pixels (/255)

b_minarea = 500000 #! ~DOM: to tune the values 

rpm_setpt = 40
rpm = 40
rotation = 0
kp = 1.4 #constant used for PID

#* IMAGE PROCESSING
x_com = np.tile(np.linspace(-1., 1., width), (height, 1)) #reps is (outside, inside)
# y_com = np.array([[i] * width for i in np.linspace(1., 1/height, height)]) #1/height is just to save pixels
# x_com_scale = ((1-y_com) ** 0.6)
y_com = np.array([[i] * width for i in np.linspace(1., 0, height)])
x_com_scale = ((1-y_com) ** 0.3)
x_gap_scale = y_com ** 0.4
x_gap_com = x_com * x_gap_scale
x_com *= x_com_scale
# ^ not scaled according to available picture!!

gap_mask = np.zeros([height, width], dtype="uint8")
peak_triangle_gap = 20
points = np.array([[0, 0], [width, 0], [int(width/2), int(height-peak_triangle_gap)]])
cv2.fillConvexPoly(gap_mask, points, 255)
gap_x_com = cv2.bitwise_and(x_com, x_com, mask=gap_mask)
gap_y_com = cv2.bitwise_and(y_com, y_com, mask=gap_mask)

gap_mask = np.zeros([height, width], dtype="uint8") #? DOM: Can we stick to one naming convention for mask variables, ie. mask_[object]
peak_triangle_gap = 20
points = np.array([[0, 0], [width, 0], [int(width/2), int(height-peak_triangle_gap)]])
cv2.fillConvexPoly(gap_mask, points, 255)
gap_x_com = cv2.bitwise_and(x_com, x_com, mask=gap_mask)  #? DOM: Also why's there a x_gap_com variable above and a gap_x_com var here :skull:
gap_y_com = cv2.bitwise_and(y_com, y_com, mask=gap_mask)

gs_erode_kernel = np.ones((3, 3), np.uint8)

#* LOGIC SETUP
class Task(enum.Enum):
    EMPTY = 0
    LEFT_GREEN = 1
    RIGHT_GREEN = 2
    DOUBLE_GREEN = 3
    RED = 4
    BEFORE_BLUE = 5  #Turning towards blue #^ TEMPORARY
    BLUE = 6


curr = Task.EMPTY
red_now = False
gs_now = False
gap_now = False
blue_now = False 
blue_once = False
see_line = 0

#~ Rescue kit
reversing_now = False #for rescue kit #^ DOM: i think i can remove this; hopefully

#* BEGIN OF LINETRACK LOOP CODE

while True:
    from_pico = receive_pico(ser)
    if lt_stream.stopped or from_pico == 1:
        break

    #* IMAGE SETUP

    frame_org = lt_stream.read()
    frame_org = cv2.flip(frame_org, 0)
    frame_org = cv2.flip(frame_org, 1)
    # cv2.imshow("correctly flipped image", frame_org) #& debug flipped image
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_green, u_green)
    mask_gs = mask_green[gs_roi_h:, :]
    # cv2.imshow("green square mask", frame_org[gs_roi_h:, :])
    # cv2.imshow("green square mask", mask_gs) #& debug green square mask
    gs_sum = np.sum(mask_gs)
    #print("Green sum:", gs_sum) #& debug green min area

    mask_black_org = cv2.inRange(frame_gray, 0, u_black) - mask_green
    # cv2.imshow('black frame', black_mask) #& debug black mask
    # print(np.sum(mask_black), curr.name)

    mask_red1 = cv2.inRange(frame_hsv, l_red1, u_red1)
    mask_red2 = cv2.inRange(frame_hsv, l_red2, u_red2)
    mask_red = mask_red1 + mask_red2
    red_sum = np.sum(mask_red)
    # print(np.sum(mask_red)) #& debug red min area

    mask_blue = cv2.inRange(frame_hsv, l_blue, u_blue) 
    mask_blue = mask_blue[100:, :]
    blue_sum = np.sum(mask_blue)
    #print(blue_sum) #& debug blue mask

    #* RED LINE 

    if red_sum > 17000000:
        red_now = True
        curr = Task.RED
        rpm = 0
    elif red_now == True:
        red_now = False
        rpm = rpm_setpt

    #* JUST FOUND GREEN SQUARE PREVIOUSLY
    
    #~ Turn while still seeing green
    elif gs_now and gs_sum < 20000:
        if curr.name == "DOUBLE_GREEN":
            if np.sum(mask_black_org) > 15000000:
                gs_now = False
        else:
            gs_now = False

    #* GREEN SQUARES

    #~ Test for sufficient green in frame
    elif not gs_now and gs_sum > gs_minarea:
        mask_gs = cv2.erode(mask_gs, gs_erode_kernel, iterations=1)
        mask_gs = cv2.dilate(mask_gs, gs_erode_kernel, iterations=1)
        # cv2.imshow("after erosion and dilation", mask_gs) #& debug green square mask

        #~ Find y position of green (Can cut processing time here by putting fixed constant instead)
        green_row = np.amax(mask_gs, axis=1)
        g_indices_v = np.where(green_row==255) #v for vertical
        gs_top = g_indices_v[0][0] + gs_roi_h
        gs_bot = g_indices_v[0][-1] + gs_roi_h
        # cv2.imshow("Green bounding box", frame_org[gs_top:, gs_left:gs_right]) #& debug green bounds

        if gs_bot > 470:
            #~ Find x positions of green
            green_col = np.amax(mask_gs, axis=0)
            g_indices_h = np.where(green_col==255) #h for horiontal
            gs_left = g_indices_h[0][0]
            gs_right = g_indices_h[0][-1]
            # print("left:", gs_left, "right:", gs_right)

            #~ Test if below or above line
            gs_bkabove = mask_black_org[gs_top - gs_bksampleoffset - gs_bksampleh : gs_top - gs_bksampleoffset, gs_left : gs_right]
            gs_bkpct = np.sum(gs_bkabove) / 255 / gs_bksampleh / (gs_right - gs_left)
            # print("Percentage of black above green:", gs_bkpct) #& debug green's black
            # cv2.imshow("black area above green square", gs_bkabove) 
            # cv2.imshow("black area above green in orig frame", frame_org[gs_top - gs_bksampleoffset - gs_bksampleh : gs_top - gs_bksampleoffset, gs_left : gs_right])
            
            #~ GREEN SQUARE FOUND
            if gs_bkpct > gs_minbkpct:

                #~ Find x position of green
                gs_centre = (gs_left + gs_right) / 2

                #~ Find x position of black
                gs_bkbeside = mask_black_org[gs_roi_h:, :]
                blackM = cv2.moments(gs_bkbeside)
                cx_black = int(blackM["m10"]/blackM["m00"]) if np.sum(gs_bkbeside) else 0 
                #? theoretically divide by zero error should never happen 
                #? XEL: oh idk why but it actually happens sometimes even tho it shouldnt this is to make sure it doesnt
                # cv2.imshow("Black beside green squares", gs_bkbeside) #& debug green type triggered
                # print("Black x-centre:", cx_black)
                # print("Green x-centre:", gs_centre)

                #~ Identify type of green square
                if cx_black > gs_left and cx_black < gs_right and gs_sum > 2 * gs_minarea:
                    curr = Task.DOUBLE_GREEN
                    gs_now = True
                elif cx_black > gs_centre:
                    curr = Task.LEFT_GREEN
                    gs_now = True
                elif cx_black < gs_centre:
                    curr = Task.RIGHT_GREEN
                    gs_now = True
                # else:
                    # print("ERROR: Green found but type indetermined")
                                

    #* DETECTION OF RESCUE KIT
    #^ Note from DOM: Will defo clean up the code once claw works haha yes

    # #~ If close enough to rescue kit: (temporary, will use lidar to detect proximity instead)
    # if not reversing_now and blue_sum >= 14000000: 
    #     blue_now = True
    #     reversing_now = True 
    #     curr = Task.BLUEFINAL  #^ DOM: please remove BLUEFINAL later thxxx

    #~ Rescue kit spotted: 
    if not reversing_now and blue_sum >= b_minarea:
        # print("Blue sum: ", blue_sum) #& debug blue sum
        blue_now = True
        if (curr.name != "BLUE" and curr.name != "BLUEFINAL"):
            curr = Task.BEFORE_BLUE

        #~ Find x and y positions of rescue kit
        blueM = cv2.moments(mask_blue)
        # cv2.imshow("blue mask", mask_blue) #& debug blue mask
        cx_blue = int(blueM["m10"] / blueM["m00"]) if np.sum(mask_blue) else 0
        cy_blue = int(blueM["m01"] / blueM["m00"]) if np.sum(mask_blue) else 0
        finalx = cx_blue - frame_org.shape[1]/2
        #finaly = frame_org.shape[0]/2 - cy_blue

        #~ if the block isn't centred
        if abs(finalx) >= 60: #^ DOM: not refined yet; consider doing abs(...) >= 100
            if cx_blue < frame_org.shape[1]/2: #if rescue kit is to the left of the frame's middle
                rotation = -45 #^ Fixed rotation of the bot; DOM: try 45 degrees instead? ie. only turn 1 wheel
                rpm = 20
            elif cx_blue > frame_org.shape[1]/2: #to the right
                rotation = 45
                rpm = 20

        #~ else if the block is (relatively) centred:
        else:
            curr = Task.BLUE

    #~ No longer sees the rescue kit
    elif blue_now and blue_sum < 2000: 
        blue_now = False
        reversing_now = False
        rpm = rpm_setpt

    if not gs_now and not red_now and not blue_now:
        
        #* OBSTACLE

        mask_seeline = mask_black_org[250:]
        # cv2.imshow('obstacle see line', mask_seeline)
        # black_line_sum = np.sum(mask_seeline)
        # print("black sum for obstacle:", black_line_sum) #& debug obstacle see line
        # if black_line_sum > 13000000:
        #     see_line = 1
        # else:
        #     see_line = 0
        # print("see line value", see_line)

        #* LINETRACK

        curr = Task.EMPTY
        # mask_gap = mask_black[crop_h_bw_gap:, :] 
        mask_black = mask_black_org[crop_h_bw:, :]
        black_kernel = np.ones((5, 5), np.uint8)
        mask_black = cv2.erode(mask_black, kernel=black_kernel)
        mask_black = cv2.dilate(mask_black, kernel=black_kernel)
        # cv2.imshow("Black lt mask", mask_gap)

        #~ Find y positions of black
        black_row = np.amax(mask_black, axis=1)
        white_indices = np.where(black_row==0)
        gap_start_y = white_indices[0][-1] if len(white_indices[0]) else 0

        if gap_start_y < height-peak_triangle_gap: #if no gap

            #~ Mask everything above the gap
            mask_black = mask_black[gap_start_y:height, :]
            # cv2.imshow("after gap accounted for", mask_black) #& debug black lt mask
            y_black_com = y_com[gap_start_y:height, :]
            x_black_com = x_com[gap_start_y:height, :]

            #~ Vectorizing the black components
            y_black = cv2.bitwise_and(y_black_com, y_black_com, mask = mask_black)
            x_black = cv2.bitwise_and(x_black_com, x_black_com, mask = mask_black)
            # cv2.imshow("yframe", y_black)
            # cv2.imshow("xframe", x_black)

        else:
            y_black = cv2.bitwise_and(gap_y_com, gap_y_com, mask = mask_black)
            x_black = cv2.bitwise_and(gap_x_com, gap_x_com, mask = mask_black)
            # print("Gap")

        #~ Line track values
        y_resultant = np.mean(y_black)
        x_resultant = np.mean(x_black)

        #~ Formatting data for transfer
        angle = math.atan2(x_resultant, y_resultant) * 180/math.pi if y_resultant != 0 else 0
        # print(y_resultant, x_resultant) #& debug resultant angle
        # print(angle)
        rotation = angle * kp
    
    #* SEND DATA TO PICO

    print("rpm:", rpm) #& debug sent variables
    print("rotation:", rotation)
    print("task:", curr)

    rotation = int(rotation)
    if rotation > 90:
        rotation = 90
    elif rotation < -90:
        rotation = -90
    rotation += 90

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm, # 0 to 200 MAX, but 100 really damn fast alr
                253, curr.value] # currently 0 to 5
                # 252, see_line] # 0 for false or 1 for true
    ser.write(to_pico)

    key = cv2.waitKey(1) #! remove for optimisation before robocup
    if key == ord('q'):
        break

lt_stream.stop()
cv2.destroyAllWindows()
