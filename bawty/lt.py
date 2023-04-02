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
crop_h_bw_gap = 110
height = height_org
gap_check_h = 180
horizon_crop_h = 40
# cam_x = width/2 #-1 to bias robot to the right
# cam_y = height - 1
# print(f"Line track image centred on ({cam_x}, {cam_y})")
centre_x = int(width/2)

#* SERIAL
ser = serial.Serial("/dev/serial0", 9600)

#* COLOR CALIBRATION
# l_black = 0
u_black = 115

#~ Real values
# l_green = np.array([30, 50, 60], np.uint8)
# u_green = np.array([85, 255, 255], np.uint8)
#~ My house's values
l_green = np.array([70, 50, 50], np.uint8)
u_green = np.array([100, 255, 255], np.uint8)

l_red1 = np.array([0, 100, 80], np.uint8) #! untuned red values
u_red1 = np.array([15, 255, 255], np.uint8)
l_red2 = np.array([170, 100, 80], np.uint8) 
u_red2 = np.array([180, 255, 255], np.uint8) #! 179 or 180?

#* CONSTANTS CALIBRATION
gs_roi_h = 310 #the crop height for gs  
gs_bksampleoffset = 10 #offset sample above green squares
gs_bksampleh = 40
gs_minbkpct = 0.35 #! to be tuned
gs_minarea = 4000 #min pixels

set_rpm = 40
rpm = 40
kp = 1
rotation = 0

#* IMAGE PROCESSING
x_com = np.tile(np.linspace(-1., 1., width), (height, 1)) #reps is (outside, inside)
y_com = np.array([[i] * width for i in np.linspace(1., 0, height)]) #1/height is just to save pixels
# x_com_scale = ((1-y_com) ** 0.3)
# x_com *= x_com_scale

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

curr = Task.EMPTY
red_now = False
gs_now = False
lt_gap = True

#* BEGIN OF LINETRACK LOOP CODE

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
    print("Green sum:", gs_sum) #& debug green min pixels

    mask_black_org = cv2.inRange(frame_gray, 0, u_black) - mask_green
    # cv2.imshow('black frame', black_mask) #& debug black mask
    # print(np.sum(mask_black), curr.name)

    mask_red1 = cv2.inRange(frame_hsv, l_red1, u_red1)
    mask_red2 = cv2.inRange(frame_hsv, l_red2, u_red2)
    mask_red = mask_red1 + mask_red2
    red_sum = np.sum(mask_red) / 255
    print("Red sum:", red_sum) #& debug red min area

    #* RED LINE 

    if red_sum > 60000:
        red_now = True
        curr = Task.RED
        rpm = 0
        rotation = 0
    elif red_now == True:
        red_now = False
        rpm = set_rpm
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

    #* LINETRACK

    if not gs_now and not red_now:

        #~ Obstacle see line
        obstacle_line_mask = mask_black_org.copy()
        obstacle_line_mask[:height-60, :] = 0
        obstacle_line_pixels = np.sum(obstacle_line_mask) / 255
        see_line = 1 if obstacle_line_pixels > 25000 else 0
        # print("See line pixels:", obstacle_line_pixels)
        # cv2.imshow("Line after obstacle", obstacle_line_mask) #& debug obstacle line

        #~ Image processing erode-dilate
        curr = Task.EMPTY
        mask_uncropped_black = mask_black_org.copy()
        mask_black = mask_black_org.copy()
        mask_black[:horizon_crop_h, :] = 0

        black_kernel = np.ones((5, 5), np.uint8)
        mask_black = cv2.erode(mask_black, black_kernel)
        mask_black = cv2.dilate(mask_black, black_kernel)
        # cv2.imshow("black mask", mask_black)

        black_kernel = np.ones((5, 5), np.uint8)
        mask_uncropped_black = cv2.erode(mask_uncropped_black, black_kernel)
        mask_uncropped_black = cv2.dilate(mask_uncropped_black, black_kernel)
        # cv2.imshow("black uncropped mask", mask_uncropped_black)

        #~ Finding the lowest black and white pixels in UNCROPPEd black
        black_row = np.amax(mask_uncropped_black, axis=1)
        black_indices_v = np.where(black_row > 0)
        black_start_y = black_indices_v[0][-1] if len(black_indices_v[0]) else 0
        black_row[black_start_y:] = 255
        white_indices = np.where(black_row == 0)
        white_start_y = white_indices[0][-1] if len(white_indices[0]) else 0
        # mask_black = mask_black[white_start_y:black_start_y, :] #& debug continous line
        curr_height = black_start_y-white_start_y
        print("black:", black_start_y, "white:", white_start_y)

        #~ Finding left and right index of black
        mask_supercrop_black = mask_black_org.copy()
        mask_supercrop_black[:-gap_check_h, :] = 0
        # cv2.imshow("super crop", mask_supercrop_black)
        black_col = np.amax(mask_supercrop_black, axis=0)
        black_indices_h = np.where(black_col == 255)
        black_start_x = black_indices_h[0][0] if len(black_indices_h[0]) else 0
        black_end_x = black_indices_h[0][-1] if len(black_indices_h[0]) else 0
        black_diff_x = black_end_x-black_start_x
        # print("x amount:", black_diff_x) #& debug width of line

        if (black_start_y < height-gap_check_h or white_start_y > height-gap_check_h) and black_diff_x < 360:
                print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>LINE GAP<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
                mask_black = cv2.bitwise_and(mask_gap, mask_black_org.copy())
                # curr = Task.RED
                mask_black[:horizon_crop_h, :] = 0 #get out horizon
                powered_y = 1
            # elif black_diff_x > 360:
            #     if centre_x-black_start_x > black_end_x-centre_x:
            #         curr = Task.TURN_LEFT
            #         print("|||||||||||LEFT LEFT LEFT")
            #     else:
            #         curr = Task.TURN_RIGHT
            #         print("RIGHT RIGHT RIGHT||||||||||||||||||||||")
        else:
            mask_black[:white_start_y, :] = 0
            curr = Task.EMPTY

            #~ Plain line track
            powered_y =  (height-40)/curr_height if curr_height != 0 else 1
            powered_y = powered_y ** 0.5
            # print("power:", powered_y) #& debug y power
            powered_y = min(2.7, powered_y)

        # if curr_height > 5:
        #     x_black_com = x_com[white_start_y:black_start_y, :]
        #     y_black_com = y_com[white_start_y:black_start_y, :]
        #     mask_black = mask_black[white_start_y:black_start_y, :]

        # cv2.imshow("black", mask_black)
        # cv2.imshow("black org", mask_black_org)

        #~ Vectorizing the black components
        y_black = cv2.bitwise_and(y_com, y_com, mask = mask_black)
        x_black = cv2.bitwise_and(x_com, x_com, mask = mask_black)
        # cv2.imshow("yframe", y_black)
        # cv2.imshow("xframe", x_black)
            

        y_resultant = np.mean(y_black) ** powered_y
        x_resultant = np.mean(x_black)
        # print(y_resultant, x_resultant) #& debug resultant angle

        #~ Formatting data for transfer
        angle = math.atan2(x_resultant, y_resultant) * 180/math.pi if y_resultant != 0 else 0
        # print(angle) #& debug angle
        pidangle = angle * kp
        
        # else:
        #     angle = 0

        print("rotation:", pidangle)

        if pidangle > 90:
            pidangle = 90
        elif pidangle < -90:
            pidangle = -90
        rotation = int(pidangle) + 90
        # print(rotation)

        # print("rpm:", rpm)
        # # print("rotation:", rotation)

    #* SEND DATA

    print("value:", curr)

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm,
                253, curr.value,
                252, see_line]

    # print(to_pico)
    
    ser.write(to_pico)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

lt_stream.stop()
cv2.destroyAllWindows()