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

# cam_x = width/2 #-1 to bias robot to the right
# cam_y = height - 1
# print(f"Line track image centred on ({cam_x}, {cam_y})")

#* SERIAL
ser = serial.Serial("/dev/serial0", 9600)

#* COLOR CALIBRATION
# l_black = 0
u_black = 102

#~ Real values
# l_green = np.array([30, 50, 60], np.uint8)
# u_green = np.array([85, 255, 255], np.uint8)
#~ My house's values
l_green = np.array([70, 90, 80], np.uint8)
u_green = np.array([96, 255, 255], np.uint8)

#~ Real values
l_blue = np.array([96, 170, 80], np.uint8)
u_blue = np.array([106, 245, 191], np.uint8)
#~ My house's values
l_blue = np.array([96, 170, 80], np.uint8)
u_blue = np.array([109, 245, 191], np.uint8)

l_red1 = np.array([0, 100, 80], np.uint8) #! untuned red values
u_red1 = np.array([15, 255, 255], np.uint8)
l_red2 = np.array([170, 100, 80], np.uint8) 
u_red2 = np.array([180, 255, 255], np.uint8) #! 179 or 180?

#* CONSTANTS CALIBRATION
height = height_org
centre_x = int(width/2)

#~ Linegap
crop_h_bw = 93
crop_h_bw_gap = 110
gap_check_h = 180
horizon_crop_h = 40

#~ GS
gs_roi_h = 310 #the crop height for gs  
gs_bksampleoffset = 10 #offset sample above green squares
gs_bksampleh = 40
gs_minbkpct = 0.35 #! to be tuned
gs_minarea = 4000 #min pixels

#~ RK
b_minarea = 6000

#~ PID
rpm_setpt = 40
rpm = 40
kp = 1

#* IMAGE PROCESSING
x_com = np.tile(np.linspace(-1., 1., width), (height, 1)) #reps is (outside, inside)
y_com = np.array([[i] * width for i in np.linspace(1., 0, height)])
#^ Glenda's method:
#~ Powering x component with respect to y
# x_com_scale = ((1-y_com) ** 0.3)
# x_com *= x_com_scale
#^ Kenneth's method:
#~ Powering x component
x_com[:, :int(width/2)] *= -1
x_com = x_com ** 1
x_com[:, :int(width/2)] *= -1
#~ Powering y component
# y_com = y_com ** 2

#~ Triangle mask
peak_triangle_gap = 0
peak_triangle_width = 60
triangle_margin = 20
mask_gap = np.zeros([height, width], dtype="uint8")
points = np.array([[triangle_margin, 0], [width-triangle_margin, 0], [int(width/2+peak_triangle_width), int(height-peak_triangle_gap)], [int(width/2-peak_triangle_width), int(height-peak_triangle_gap)]])
cv2.fillConvexPoly(mask_gap, points, 255)

gs_erode_kernel = np.ones((3, 3), np.uint8)

#* LOGIC SETUP
class Task(enum.Enum):
    EMPTY = 0
    LEFT_GREEN = 1
    RIGHT_GREEN = 2
    DOUBLE_GREEN = 3
    RED = 4
    TURN_LEFT = 5
    TURN_RIGHT = 6
    TURN_BLUE = 7
    BLUE = 8
    LINEGAP = 11

curr = Task.EMPTY
rotation = 0
see_line = 0
end_line_gap = 0

red_now = False
gs_now = False
blue_now = False

#* BEGIN LINETRACK LOOP ----------------------------------------------------------------------------------------------

while True:
    if lt_stream.stopped:
        break

    #* IMAGE SETUP

    frame_org = lt_stream.read()
    frame_org = cv2.flip(frame_org, 0)
    frame_org = cv2.flip(frame_org, 1)
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_green, u_green)
    mask_gs = mask_green.copy()
    mask_gs[:gs_roi_h, :] = 0
    mask_gs = cv2.erode(mask_gs, gs_erode_kernel, iterations=1)
    mask_gs = cv2.dilate(mask_gs, gs_erode_kernel, iterations=1)
    gs_sum = np.sum(mask_gs)/255
    # cv2.imshow("green square mask", frame_org[gs_roi_h:, :])
    # cv2.imshow("green square mask", mask_gs) #& debug green square mask
    # print("Green sum:", gs_sum) #& debug green min pixels

    mask_black_org = cv2.inRange(frame_gray, 0, u_black) - mask_green
    # cv2.imshow('black frame', black_mask) #& debug black mask
    # print(np.sum(mask_black), curr.name)

    mask_red1 = cv2.inRange(frame_hsv, l_red1, u_red1)
    mask_red2 = cv2.inRange(frame_hsv, l_red2, u_red2)
    mask_red = mask_red1 + mask_red2
    red_sum = np.sum(mask_red) / 255
    # print("Red sum:", red_sum) #& debug red min area

    mask_blue = cv2.inRange(frame_hsv, l_blue, u_blue) 
    mask_blue[:horizon_crop_h, :] = 0
    blue_sum = np.sum(mask_blue) / 255
    # print("Blue sum:", blue_sum) #& debug blue min area

    #* RED LINE 

    if red_sum > 60000:
        red_now = True
        curr = Task.RED
        rpm = 0
        rotation = 0
    elif red_now == True:
        red_now = False
        rpm = rpm_setpt
        rotation = 0

    #* JUST FOUND GREEN SQUARE PREVIOUSLY

    #~ Turn while still seeing green
    elif gs_now and gs_sum < 5:
        gs_now = False

    #* GREEN SQUARES

    #~ Test for sufficient green in frame
    elif not gs_now and gs_sum > gs_minarea:

        #~ Find y position of green
        green_row = np.amax(mask_gs, axis=1)
        g_indices_v = np.where(green_row==255) #v for vertical
        # gs_top = g_indices_v[0][0]
        gs_bot = g_indices_v[0][-1]
        # cv2.imshow("Green bounding box", frame_org[gs_top:, gs_left:gs_right]) #& debug green bounds

        #~ Wait for green to reach last 5 pixels
        if gs_bot > 475:

            #~ Find x positions of green
            green_col = np.amax(mask_gs, axis=0)
            g_indices_h = np.where(green_col==255) #h for horizontal
            gs_left = g_indices_h[0][0]
            gs_right = g_indices_h[0][-1]
            # print("left:", gs_left, "right:", gs_right)

            #~ Test if below or above line
            gs_bkabove = mask_black_org[gs_roi_h - gs_bksampleoffset - gs_bksampleh : gs_roi_h - gs_bksampleoffset, gs_left : gs_right]
            gs_bkpct = np.sum(gs_bkabove) / 255 / gs_bksampleh / (gs_right - gs_left)
            # print("Percentage of black above green:", gs_bkpct) #& debug green's black
            # cv2.imshow("black area above green square", gs_bkabove) 
            # cv2.imshow("black area above green in orig frame", frame_org[gs_top - gs_bksampleoffset - gs_bksampleh : gs_top - gs_bksampleoffset, gs_left : gs_right])
            
            #~ GREEM SQUARE FOUND
            if gs_bkpct > gs_minbkpct:

                #~ Find x position of green
                gs_centre = (gs_left + gs_right) / 2

                #~ Find x position of black
                gs_bkbeside = mask_black_org[gs_roi_h:, :]
                blackM = cv2.moments(gs_bkbeside)
                cx_black = int(blackM["m10"]/blackM["m00"]) if np.sum(gs_bkbeside) else 0 #theoretically divide by zero error should never happen
                # cv2.imshow("Black beside green squares", gs_bkbeside) #& debug green type triggered
                # print("Black x-centre:", cx_black)
                # print("Green x-centre:", gs_centre)

                #~ Identify type of green square
                if cx_black > gs_left and cx_black < gs_right and gs_sum > 35000:
                    curr = Task.DOUBLE_GREEN
                    gs_now = True
                elif cx_black > gs_centre:
                    curr = Task.LEFT_GREEN
                    gs_now = True
                elif cx_black < gs_centre:
                    curr = Task.RIGHT_GREEN
                    gs_now = True
                else:
                    print("ERROR: Green found but type indetermined")
    
    #* RESCUE KIT

    #~ Rescue kit spotted:
    if blue_sum >= b_minarea:
        blue_now = True
        if (curr.name != "BLUE"):
            curr = Task.TURN_BLUE

        #~ Find x and y positions of rescue kit
        blueM = cv2.moments(mask_blue)
        cx_blue = int(blueM["m10"] / blueM["m00"]) if np.sum(mask_blue) else 0
        cy_blue = int(blueM["m01"] / blueM["m00"]) if np.sum(mask_blue) else 0
        finalx = cx_blue - centre_x

        #~ If the block isn't centred
        if abs(finalx) >= 60: #^ DOM: not refined yet; consider doing abs(...) >= 100
            if cx_blue < centre_x: #if rescue kit is to the left of the centre of the frame
                rotation = -45 # fixed rotation of the bot
                rpm = 20
            elif cx_blue > centre_x: #to the right
                rotation = 45
                rpm = 20

        #~ Else if the block is (relatively) centred:
        else:
            curr = Task.BLUE

    #~ No longer sees the rescue kit
    elif blue_now and blue_sum < 2000: 
        blue_now = False
        rpm = rpm_setpt

    #* LINETRACK

    if not gs_now and not red_now and not blue_now:

        #~ Obstacle see line
        obstacle_line_mask = mask_black_org.copy()
        obstacle_line_mask[:height-60, :] = 0
        obstacle_line_pixels = np.sum(obstacle_line_mask) / 255
        see_line = 1 if obstacle_line_pixels > 17000 else 0
        if see_line:
            print('|'* 5, "SEE LINE", '|' * 5)
        print("See line pixels:", obstacle_line_pixels)
        # cv2.imshow("Line after obstacle", obstacle_line_mask) #& debug obstacle line



        #~ Image processing erode-dilate
        curr = Task.EMPTY
        black_kernel = np.ones((5, 5), np.uint8)

        mask_black = mask_black_org.copy()
        mask_black = cv2.erode(mask_black, black_kernel)
        mask_black = cv2.dilate(mask_black, black_kernel)
        mask_uncropped_black = mask_black.copy()
        mask_supercrop_black = mask_black.copy()
        mask_linegap = mask_black.copy()
        # mask_linegap[]

        mask_black[:horizon_crop_h, :] = 0
        mask_supercrop_black[:-gap_check_h, :] = 0
        #& debug masks
        # cv2.imshow("black mask", mask_black)
        # cv2.imshow("black uncropped mask", mask_uncropped_black)
        # cv2.imshow("super cropped mask", mask_supercrop_black)

        end_line_gap = 0

        #~ Finding the lowest black and white pixels in UNCROPPED black
        black_row = np.amax(mask_uncropped_black, axis=1)
        black_indices_v = np.where(black_row == 255)
        black_start_y = black_indices_v[0][-1] if len(black_indices_v[0]) else 0
        black_row[black_start_y:] = 255
        white_indices = np.where(black_row == 0)
        white_start_y = white_indices[0][-1] if len(white_indices[0]) else 0

        black_line_height = black_start_y-white_start_y
        # print("black: ", black_start_y, "white:", white_start_y)

        #~ Finding left and right index of black
        black_col = np.amax(mask_supercrop_black, axis=0)
        black_indices_h = np.where(black_col == 255)
        black_start_x = black_indices_h[0][0] if len(black_indices_h[0]) else 0
        black_end_x = black_indices_h[0][-1] if len(black_indices_h[0]) else 0
        black_line_width = black_end_x-black_start_x

        # mask_black = mask_black[white_start_y:black_start_y, :] #& debug continous line
        print("x amount:", black_line_width) #& debug width of line


        #~ If line gap (line ending and line width small) 
        if (black_start_y < height-gap_check_h or white_start_y > height-gap_check_h) and black_line_width < 360:
                print(">" * 15 + 'LINE GAP' + '<' * 15)
                mask_black = cv2.bitwise_and(mask_gap, mask_black)
                #^ DOM: using obstacle mask for now
                curr = Task.LINEGAP
                powered_y = 1

            #~ 90 degrees (NOT WORKING)
            # elif black_line_width > 360:
            #     if centre_x-black_start_x > black_end_x-centre_x:
            #         curr = Task.TURN_LEFT
            #         print("|||||||||||LEFT LEFT LEFT")
            #     else:
            #         curr = Task.TURN_RIGHT
            #         print("RIGHT RIGHT RIGHT||||||||||||||||||||||")

        #~ Line continuation
        else:
            mask_black[:white_start_y, :] = 0
            curr = Task.EMPTY

            #~ Plain line track
            powered_y = (height-40)/black_line_height if black_line_height != 0 else 1
            powered_y = powered_y ** 0.5
            powered_y = min(3.5, powered_y)

        if black_start_y > 400 and white_start_y < 250 and black_line_width > 60: #TODO: offset mask
            print("-------------------------------------end line gap-------------------------------------")
            end_line_gap = 1

        #~ Vectorizing the black components
        #^ Ancillary: Powering xcom
        # x_com = x_com ** 1.5
        #^ Method 1: Powering ycom
        # powered_y = 2
        # y_com = y_com ** powered_y
        # y_black = cv2.bitwise_and(y_com, y_com, mask = mask_black)
        # x_black = cv2.bitwise_and(x_com, x_com, mask = mask_black)
        # y_resultant = np.mean(y_black)
        # x_resultant = np.mean(x_black)
        #^ Method 2: Powering the mean
        y_black = cv2.bitwise_and(y_com, y_com, mask = mask_black)
        x_black = cv2.bitwise_and(x_com, x_com, mask = mask_black)
        y_resultant = np.mean(y_black) ** powered_y
        x_resultant = np.mean(x_black)
        #& debug
        # print("power:", powered_y)
        # cv2.imshow("yframe", y_black)
        # cv2.imshow("xframe", x_black)
        # print(y_resultant, x_resultant)

        #~ Formatting data for transfer
        angle = math.atan2(x_resultant, y_resultant) * 180/math.pi if y_resultant != 0 else 0
        rotation = angle * kp

    # print("rotation:", rotation)

    if rotation > 90:
        rotation = 90
    elif rotation < -90:
        rotation = -90
    rotation = int(rotation) + 90
        # print(rotation)

        # print("rpm:", rpm)
        # # print("rotation:", rotation)

    #* SEND DATA

    print("value:", curr)

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm,
                253, curr.value,
                252, see_line,
                251, end_line_gap]

    # print(to_pico)
    
    ser.write(to_pico)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

lt_stream.stop()
cv2.destroyAllWindows()