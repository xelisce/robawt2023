import cv2
from MultiThread import WebcamStream
import serial
import numpy as np
np.set_printoptions(threshold=1000) #^ to print entire array, use this np.inf and to truncate the array when printing, use the default 1000 or any length preferred
import enum
import math
import time

#* SERIAL
ser = serial.Serial("/dev/serial0", 115200)

#* CAMERAS START
bot_stream = WebcamStream(stream_id=0)
bot_stream.start()
bot_stream_frame = bot_stream.read()
bot_stream_width_org, bot_stream_height_org = bot_stream_frame.shape[1], bot_stream_frame.shape[0]
print("Bottom camera width:", bot_stream_width_org, "Camera height:", bot_stream_height_org)

top_stream = WebcamStream(stream_id=2)
top_stream.start()
top_stream_frame = top_stream.read()
top_stream_width_org, top_stream_height_org = top_stream_frame.shape[1], top_stream_frame.shape[0]
print("Top camera width:", top_stream_width_org, "Camera height:", top_stream_height_org)

#* GREEN SQUARES CONSTANTS
gs_roi_h = 40 #! arbitrary number
gs_erode_kernel = np.ones((3, 3), np.uint8)
gsVotes = [0, 0, 0]

#* LINE TRACK CONSTANTS
crop_lt_h = 40
supercrop_lt_h = 72//2 + 5
kp_lt = 1
width_lt = top_stream_width_org // 4
height_lt = top_stream_height_org // 4
print("width of downscaled image:", width_lt, "height:", height_lt)
centre_x_lt = width_lt//2
black_kernel = np.ones((2, 2), np.uint8)

#~ PID
rpm_setptlt = 100

#~ Trapezium-ish mask 
# to mask out robot and claw
# mask_trapeziums_claw_down = np.zeros([height_lt, width_lt], dtype="uint8")
# crop_bot_h_blue = 62//2
# higher_crop_triangle_h_blue = 72//2
# higher_crop_triangle_w_blue = 75//2
# higher_crop_triangle_gap_w_blue = 35//2
# left_triangle_pts_blue = np.array([[0, height_lt - crop_bot_h_blue], [0, height_lt - higher_crop_triangle_h_blue], [centre_x_lt - higher_crop_triangle_gap_w_blue - higher_crop_triangle_w_blue, height_lt - higher_crop_triangle_h_blue], [centre_x_lt - higher_crop_triangle_gap_w_blue , height_lt - crop_bot_h_blue]])
# right_triangle_pts_blue = np.array([[width_lt, height_lt - crop_bot_h_blue], [width_lt, height_lt - higher_crop_triangle_h_blue], [centre_x_lt + higher_crop_triangle_gap_w_blue + higher_crop_triangle_w_blue, height_lt - higher_crop_triangle_h_blue], [centre_x_lt + higher_crop_triangle_gap_w_blue , height_lt - crop_bot_h_blue]])
# bottom_rectangle_pts_blue = np.array([[0, height_lt], [0, height_lt - crop_bot_h_blue], [width_lt, height_lt - crop_bot_h_blue], [width_lt, height_lt]])
# cv2.fillPoly(mask_trapeziums_claw_down, [left_triangle_pts_blue, right_triangle_pts_blue, bottom_rectangle_pts_blue], 255)
# mask_trapeziums_claw_down = cv2.bitwise_not(mask_trapeziums_claw_down)
# to mask out robot
mask_trapeziums = np.zeros([height_lt, width_lt], dtype="uint8")
# crop_bot_h = 15
# higher_crop_triangle_h = 23
# higher_crop_triangle_w = 26
# higher_crop_triangle_gap_w = 24
crop_bot_h = 62//2
higher_crop_triangle_h = 72//2
higher_crop_triangle_w = 75//2
higher_crop_triangle_gap_w = 35//2
left_triangle_pts = np.array([[0, height_lt - crop_bot_h], [0, height_lt - higher_crop_triangle_h], [centre_x_lt - higher_crop_triangle_gap_w - higher_crop_triangle_w, height_lt - higher_crop_triangle_h], [centre_x_lt - higher_crop_triangle_gap_w , height_lt - crop_bot_h]])
right_triangle_pts = np.array([[width_lt, height_lt - crop_bot_h], [width_lt, height_lt - higher_crop_triangle_h], [centre_x_lt + higher_crop_triangle_gap_w + higher_crop_triangle_w, height_lt - higher_crop_triangle_h], [centre_x_lt + higher_crop_triangle_gap_w , height_lt - crop_bot_h]])
bottom_rectangle_pts = np.array([[0, height_lt], [0, height_lt - crop_bot_h], [width_lt, height_lt - crop_bot_h], [width_lt, height_lt]])
cv2.fillPoly(mask_trapeziums, [left_triangle_pts, right_triangle_pts, bottom_rectangle_pts], 255)
mask_trapeziums = cv2.bitwise_not(mask_trapeziums)

#~ Vectors (use visualiser.py to understand better)
x_com = np.tile(np.linspace(-1., 1., width_lt), (height_lt, 1)) #reps is (outside, inside)
y_com = np.array([[i] * width_lt for i in np.linspace(1., 0, height_lt)])
#^ Kenneth's method:
#~ Powering x component
# x_com[:, :int(width_lt/2)] *= -1
# x_com = x_com ** 0.8
# x_com[:, :int(width_lt/2)] *= -1
#~ Powering y component
# y_com = y_com ** 2
#^ Glenda's method: 
#~ Powering x component with respect to y
# x_com_scale = ((1-y_com) ** 0.3)
# x_com *= x_com_scale
#~ Same but bumped up to remove cropped out
x_com_scale = 1 - np.array([[i] * width_lt for i in np.linspace(1., 0, height_lt-higher_crop_triangle_h)])
x_com_scale = x_com_scale ** 3
x_com_scale = np.concatenate((x_com_scale, np.array([[1] * width_lt for i in range(higher_crop_triangle_h)])))
x_com *= x_com_scale

#* EVAC CONSTANTS
crop_h_evac = 50
evac_height = top_stream_height_org-crop_h_evac
height_evac_t = top_stream_height_org - crop_h_evac

centre_x_botcam = bot_stream_width_org//8 #! cuz we r pyr down bot camz
centre_x_topcam = top_stream_width_org//4 #! LOL no wonder the ball detection wasnt working

kp_ball = 1
rpm_evac = 30 #not actually used

# u_sat_thresh = np.array([0, 0, 0], np.uint8)
# l_sat_thresh = np.array([180, 255, 255], np.uint8)

crop_bh_evactolt = 120 #with top cam
crop_th_evactolt = 100 #with top cam

entered_evac = False


#* IMAGE THRESHOLDS

#~ Xel's house values
l_blue = np.array([95, 100, 100], np.uint8) #? idk what your prev values were sorryz
u_blue = np.array([115, 255, 255], np.uint8)

l_red1lt = np.array([0, 50, 50], np.uint8)
u_red1lt = np.array([10, 255, 255], np.uint8)
l_red2lt = np.array([170, 50, 50], np.uint8) 
u_red2lt = np.array([180, 255, 255], np.uint8)

l_greenlt = np.array([63, 85, 85], np.uint8) #alternate values: 50,50,90
# u_greenlt = np.array([80, 255, 255], np.uint8)
u_greenlt = np.array([90, 255, 255], np.uint8) # my house
l_greenlt_forblack = np.array([60, 70, 90], np.uint8)
u_greenlt_forblack = np.array([80, 220, 255], np.uint8)

l_red1evac = np.array([0, 60, 20], np.uint8)
u_red1evac = np.array([15, 255, 255], np.uint8)
l_red2evac = np.array([170, 60, 20], np.uint8) 
u_red2evac = np.array([180, 255, 255], np.uint8)

l_greenevac = np.array([70, 35, 20], np.uint8)
u_greenevac = np.array([95, 255, 255], np.uint8)

dp = 3
min_dist = 77 #67
param1 = 191 #128
param2 = 103 #62
min_radius = 65
max_radius = 88

u_blackforball = 49
u_black_lt = 72 #102
u_black_lineforltfromevac = 55

u_sat_thresh = np.array([0, 0, 0], np.uint8)
l_sat_thresh = np.array([180, 255, 255], np.uint8)

#* VIDEO STREAM

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (top_stream_width_org ,top_stream_height_org))

#* VARIABLE INITIALISATIONS
class Task(enum.Enum):
    #~ Linetrack
    EMPTY = 0
    LEFT_GREEN = 1
    RIGHT_GREEN = 2
    DOUBLE_GREEN = 3
    ALIGN_LINEGAP = 10
    RED = 4
    TURN_BLUE = 7
    BLUE = 8
    LINEGAP = 12
#     TURN_LEFT = 5
#     TURN_RIGHT = 6
       
#     # SILVER = 12
#     #~ Evac
    NOBALL = 20
    BALL = 21
    DEPOSITALIVE = 22
    DEPOSITDEAD = 23
    EMPTYEVAC = 24
    FINDINGLINE = 25


rotation = 0
curr = Task.EMPTY
rpm_lt = rpm_setptlt
see_thin_line = 0 #unused
# ball_type = 1
# silver_line = 0
end_line_gap = 0
# seesaw = 1
# new_endlinegap = 1

red_now = False
gs_now = False
blue_now = False
linegap_now = False
short_linegap_now = False #unused
# align_long_linegap_now = False
long_linegap_now = False #unused

pico_task = 0

# pi_start = True

#* FUNCTIONS

def receive_pico() -> str:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        print("pico task:", line)
        if line:
            line = line.replace('\x00', '')
            return line
        else:
            return -1
    else:
        return -1 #no info
    
#* ---------------------------------- LINETRACK FUNCTIONS ----------------------------------

def contoursCalc(frame_org, mask_black):
    contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
    # cv2.drawContours(frame_org, contours, -1, (0, 255, 0), 3)
    # cv2.imshow("with contours", frame_org)
    # print(contours)
    cnts = []
    for cnt in contours:
        x,y,w,h = cv2.boundingRect(cnt)
        cnts.append({
            "x": x,
            "y": y,
            "w": w,
            "h": h
        })
    if len(cnts):
        closest_contour = max(cnts, key=lambda x: x["y"]+x["h"])
        contour_mask = np.zeros([height_lt, width_lt], dtype="uint8")
        cv2.rectangle(contour_mask,(closest_contour["x"],closest_contour["y"]),(closest_contour["x"]+closest_contour["w"],closest_contour["y"]+closest_contour["h"]), 255 , -1)
        # cv2.imshow("contours", contour_mask)
        mask_black = cv2.bitwise_and(mask_black, contour_mask)
        return (mask_black, closest_contour["y"]) 
    else:
        return (mask_black, -1)

#* MAIN FUNCTION ------------------------ MAIN LINETRACK ----------------------------------

def task0_lt():
    global rotation, rpm_lt, see_thin_line, end_line_gap, curr
    global gs_now, red_now, blue_now, short_linegap_now, long_linegap_now, linegap_now
    global gsVotes

    #~ Basic conversion of color spaces
    frame_org = top_stream.read()
    out.write(frame_org)
    frame_org = cv2.pyrDown(frame_org, dstsize=(top_stream_width_org//2, top_stream_height_org//2))
    frame_org = cv2.pyrDown(frame_org, dstsize=(width_lt, height_lt))
    # frame_org = cv2.bitwise_and(frame_org, frame_org, mask=mask_trapeziums)
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)
    # cv2.imshow("top camera frame", frame_org)

    #~ Masking out red
    mask_red1 = cv2.inRange(frame_hsv, l_red1lt, u_red1lt)
    mask_red2 = cv2.inRange(frame_hsv, l_red2lt, u_red2lt)
    mask_red = mask_red1 + mask_red2
    # cv2.imshow("red mask", mask_red)
    red_sum = np.sum(mask_red)/ 255
    print("red sum:", red_sum)

    #~ Masking out green
    mask_green = cv2.inRange(frame_hsv, l_greenlt, u_greenlt)
    # cv2.imshow("green square mask", mask_green)
    mask_gs = mask_green.copy()
    mask_gs[: gs_roi_h] = 0
    mask_gs = cv2.erode(mask_gs, gs_erode_kernel, iterations=1)
    mask_gs = cv2.dilate(mask_gs, gs_erode_kernel, iterations=1)
    gs_sum = np.sum(mask_gs)/255
    # cv2.namedWindow('gs mask', 2)
    # cv2.resizeWindow('gs mask', 550, 50)
    cv2.imshow("gs mask", mask_gs) #& debug green square mask
    print("Green sum:", gs_sum) #& debug green min pixels

    #~ Masking out blue
    mask_blue = cv2.inRange(frame_hsv, l_blue, u_blue)
    blue_sum = np.sum(mask_blue) / 255
    # cv2.namedWindow('blue mask', 2)
    # cv2.resizeWindow('blue mask', 550, 100)
    # cv2.imshow("blue mask", mask_blue) #& debug blue mask
    print("Blue sum:", blue_sum) #& debug blue pixels

    #~ Masking out black
    mask_green_for_black = cv2.inRange(frame_hsv, l_greenlt_forblack, u_greenlt_forblack)
    mask_all_green = cv2.bitwise_or(mask_green, mask_green_for_black)
    mask_black_org = cv2.inRange(frame_gray, 0, u_black_lt) - mask_all_green
    # cv2.namedWindow('black mask', 2)
    # cv2.resizeWindow('black mask', 550, 50)
    # cv2.imshow("black mask", mask_black_org) #& debug green square mask
    # mask_black = cv2.bitwise_and(mask_black_org, mask_black_org, mask=mask_trapeziums)

    if red_sum > 2200: #^ measured values: ~2500
        red_now = True
        curr = Task.RED
    elif red_now == True:
        red_now = False

    #* JUST FOUND GREEN SQUARES PREVIOUSLY
    if gs_now and gs_sum < 5:
        gs_now = False


    #* GREEN SQUARES
    elif not gs_now and gs_sum > 100: #! arbitrary number

        gs_blacksample_offset = 0 #! arbitrary number
        gs_blacksample_h = 10 #! arbitrary number
        GS_MIN_BKPCT = 0.20 #! arbitrary number

        #~ Find y position of green
        green_row = np.amax(mask_gs, axis=1)
        gs_vertical_indices = np.where(green_row==255)
        gs_top = gs_vertical_indices[0][0]
        gs_bot = gs_vertical_indices[0][-1]
        print("GS_top: ", gs_top, " |  GS_bot: ", gs_bot)

        #~ Find left & right of green
        green_col = np.amax(mask_gs, axis=0)
        gs_horizontal_indices = np.where(green_col==255)
        gs_left = gs_horizontal_indices[0][0]
        gs_right = gs_horizontal_indices[0][-1]
        gs_width = gs_right - gs_left 
        gs_centre = (gs_left + gs_right) / 2
        print("GS_width: ", gs_width)

        #~ Test if below or above line (percentage of black above green)
        gs_bkabove = mask_black_org[gs_top - (gs_blacksample_offset + gs_blacksample_h): gs_top-gs_blacksample_offset, gs_left: gs_right]
        gs_bkpct = np.sum(gs_bkabove) / 255 / gs_blacksample_h / (gs_width) 
        # cv2.namedWindow('bk_above', 2)
        # cv2.resizeWindow('bk_above', 550, 50)
        # if gs_bkabove:
        # cv2.imshow("bk_above", gs_bkabove)
        print("Percentage of black above green: ", gs_bkpct)  

        #~ Test if green is left or right of black
        #TODO might want to consider moving confidence votes out of the black percent check so that the circle triggers more easily 
        if gs_bkpct > GS_MIN_BKPCT:
            gs_bkbeside = mask_black_org[gs_top: gs_bot]
            gs_bkbeside[:, :30] = 0
            gs_bkbeside[:, -30:] = 0
            gs_black_moments = cv2.moments(gs_bkbeside)
            cx_black = int(gs_black_moments["m10"]/gs_black_moments["m00"]) if np.sum(gs_bkbeside) else 0
            # cv2.namedWindow('bkbeside', 2)
            # cv2.resizeWindow('bkbeside', 550, 50)
            # cv2.imshow('bkbeside', gs_bkbeside)
            print("Cx_Black: ", cx_black)
            print("Left: ", gs_left, " | right: ", gs_right)
            
            #~ Confidence votes
            if cx_black > gs_left and cx_black < gs_right and gs_sum > 200 and gs_width > 35: #! arbitrary numbers
                gsVotes[2] += 1            
            elif cx_black > gs_centre: #left
                gsVotes[0] += 1
            elif cx_black < gs_centre:
                gsVotes[1] += 1 

            print("Left Votes: ", gsVotes[0], " ||  Right Votes: ", gsVotes[1], " ||  Double Votes: ", gsVotes[2])

            #~ Wait for green to reach near the bottom of frame
            if gs_bot > 65: 
                dir = gsVotes.index(max(gsVotes))

                if dir == 2:
                    curr = Task.DOUBLE_GREEN
                    gs_now = True
                    print(">" * 15, "Double green ", "<" * 15)
                elif dir == 0:
                    curr = Task.LEFT_GREEN
                    gs_now = True
                    print("'\033[41m'" + "<" * 40, "left green" + "'\033[0m'")
                elif dir == 1:
                    curr = Task.RIGHT_GREEN
                    gs_now = True
                    print("'\033[94m'" + "right green", ">" * 40 + "'\033[0m'")

        else:
            gsVotes[0] -= 1
            gsVotes[1] -= 1
            gsVotes[2] -= 1
    
    #~ Reset green votes
    elif gs_sum < 100:
        gsVotes = [0, 0, 0]

    #* RESCUE KIT

    b_minarea = 250 #! arbitrary number
    #~ Rescue kit spotted:
    if blue_sum >= b_minarea:

        # mask_blue = cv2.bitwise_and(mask_blue, mask_blue, mask_trapeziums_claw_down)
        blue_now = True
        if (curr.name != "BLUE"):
            curr = Task.TURN_BLUE

        #~ Find x and y positions of rescue kit
        blueM = cv2.moments(mask_blue)
        cx_blue = int(blueM["m10"] / blueM["m00"]) if np.sum(mask_blue) else 0
        # cy_blue = int(blueM["m01"] / blueM["m00"]) if np.sum(mask_blue) else 0
        finalx = cx_blue - centre_x_lt

        #~ If the block isn't centred
        if abs(finalx) >= 5: #! arbitrary number
            if cx_blue < centre_x_lt: #if rescue kit is to the left of the centre of the frame
                rotation = -45 # fixed rotation of the bot
                rpm_lt = 80
            elif cx_blue > centre_x_lt: #to the right
                rotation = 45
                rpm_lt = 80

        #~ Else if the block is (relatively) centred:
        else:
            curr = Task.BLUE

    #~ No longer sees the rescue kit
    elif blue_now and blue_sum < 20: #! arbitrary number
        blue_now = False
        rpm_lt = rpm_setptlt


    #* LINETRACK    

    if not red_now and not blue_now:

        #~ Image processing erode-dilate
        if not gs_now:
            curr = Task.EMPTY
        mask_black = mask_black_org.copy()
        mask_black = cv2.bitwise_and(mask_black_org, mask_black_org, mask=mask_trapeziums)
        mask_black = cv2.erode(mask_black, black_kernel)
        mask_black = cv2.dilate(mask_black, black_kernel)
        mask_uncropped_black = mask_black.copy()
        mask_supercrop_black = mask_black.copy()

        mask_black[:crop_lt_h, :] = 0
        mask_supercrop_black[:supercrop_lt_h, :] = 0

        # cv2.imshow("mask black", mask_black)

        if np.sum(mask_black) > 0:
            #~ Finding the closest line segment
            black_rows = np.amax(mask_uncropped_black, axis=1); 
            black_indices_v = np.where(black_rows == 255)
            first_line_bottom = black_indices_v[0][-1] if len(black_indices_v[0]) else 0
            black_rows[first_line_bottom:] = 255 #coloring black below the first line
            white_indices_v = np.where(black_rows == 0)
            first_line_top = white_indices_v[0][-1] if len(white_indices_v[0]) else 0
            first_line_height = first_line_bottom - first_line_top
            print("first line start: ", first_line_top, "end: ", first_line_bottom, "height: ", first_line_height)

            #~ Finding the next line segment
            black_rows[first_line_top:] = 0 #now removing the first line
            black_indices_v2 = np.where(black_rows == 255)
            second_line_bottom = black_indices_v2[0][-1] if len(black_indices_v2[0]) else 0
            black_rows[second_line_bottom:] = 255 #coloring in everything below the second line
            white_indices_v2 = np.where(black_rows == 0)
            second_line_top = white_indices_v2[0][-1] if len(white_indices_v2[0]) else 0
            second_line_height = second_line_bottom - second_line_top
            print("second line start: ", second_line_top, "end: ", second_line_bottom, "height: ", second_line_height)

            #~ Finding the width of super cropped black line
            black_cols = np.amax(mask_supercrop_black, axis=0)
            black_indices_h = np.where(black_cols == 255)
            black_left_x = black_indices_h[0][0] if len(black_indices_h[0]) else 0
            black_right_x = black_indices_h[0][-1] if len(black_indices_h[0]) else 0
            black_line_width = black_right_x - black_left_x
            print("x width:", black_line_width) #& debug width of line

            #~ Trigger line gap
            if (first_line_top > 80) and not gs_now:
                if first_line_top - second_line_bottom < 30: #~ small line gap
                    mask_black = mask_black_org
                    mask_black[:, :30] = 0
                    mask_black[:, -30:] = 0
                    mask_black[first_line_top:, :] = 0
                else: #~ long line gap
                    curr = Task.LINEGAP
                    linegap_now = True

            #~ Trigger end line gap 
            #^ This happens even in green squares
            if first_line_height > 32 and first_line_bottom > 65: #89 lowest possible, 89-32=57 < 65 so it is ok
                end_line_gap = 1 # to end the linegap move forward
                curr = Task.EMPTY
                linegap_now = False
            else:
                end_line_gap = 0

            if not gs_now:
                rpm_lt = rpm_setptlt
                #~ Line continuation:
                #^ Contour method
                mask_black = contoursCalc(frame_org, mask_black)[0]
                #^ Index method
                # mask_black[:first_line_top, :] = 0
                # cv2.imshow("black mask", mask_black)

                #~ Powers and components
                # powered_y = (height_lt-crop_lt_h-crop_bot_h)/first_line_height if first_line_height != 0 else 1
                # powered_y = powered_y ** 0.01 #? prev value: 0.5
                # powered_y = min(3.5, powered_y) #? prev value: 4
                powered_y = 0.5
                x_black_com = x_com

                #~ Vectorizing the black components
                #^ Ancillary: Powering xcom
                # x_com[:, :int(centre/2)] *= -1
                # x_com = x_com ** 1
                # x_com[:, :int(width/2)] *= -1
                #^ Method 1: Powering ycom
                # powered_y = 2
                # y_com = y_com ** powered_y
                # y_black = cv2.bitwise_and(y_com, y_com, mask = mask_black)
                # x_black = cv2.bitwise_and(x_com, x_com, mask = mask_black)
                # y_resultant = np.mean(y_black)
                # x_resultant = np.mean(x_black)
                #^ Method 2: Powering the mean
                y_black = cv2.bitwise_and(y_com, y_com, mask = mask_black)
                x_black = cv2.bitwise_and(x_black_com, x_black_com, mask=mask_black)
                # print(mask_black)
                # print(x_black[x_black!=0])
                y_resultant = np.mean(y_black) ** powered_y
                x_black_nonzero = x_black[x_black!=0]
                x_resultant = np.mean(x_black_nonzero) if len(x_black_nonzero) else 0
                #& debug
                print("power:", powered_y)
                # cv2.imshow("yframe", y_black)
                # cv2.imshow("xframe", x_black)
                print("y_resultant:", y_resultant, "x_resultant:", x_resultant)

                #~ Formatting data for transfer
                angle = math.atan2(x_resultant, y_resultant) * 180/math.pi if y_resultant != 0 else 0
                print("angle:", angle)
                rotation = angle * kp_lt
                print("rotation:", rotation)
        
        elif not gs_now:
            print("NO BLACK DETECTED")
            curr = Task.LINEGAP
            end_line_gap = 0
            linegap_now = True

    if rotation > 90:
        rotation = 90
    elif rotation < -90:
        rotation = -90
    rotation = int(rotation) + 90

    #* SEND DATA
    # print("value:", curr)
    # print("rpm_lt:", rpm_lt)
    # # print("rotation:", rotation)

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm_lt,
                253, curr.value,
                251, end_line_gap]
    # print(to_pico)
    ser.write(to_pico)
    print(curr.name)

#* MAIN FUNCTION ------------------------ EVAC FIND BALL ----------------------------------

def task_1_ball(): #^ downres-ed once
    global rotation, curr, ball_type

    #* IMAGE SETUP
    evac_org = top_stream.read()
    # evac_org = cv2.bitwise_and(evac_org, evac_org, mask=mask_trapeziums_claw_down)
    out.write(evac_org)
    evac_org = cv2.pyrDown(evac_org, dstsize=(top_stream_width_org//2, top_stream_height_org//2))
    evac_gray = cv2.cvtColor(evac_org, cv2.COLOR_BGR2GRAY) #! not used
    evac_hsv = cv2.cvtColor(evac_org, cv2.COLOR_BGR2HSV)
    evac_sat_mask = cv2.inRange(evac_hsv, u_sat_thresh, l_sat_thresh) #! why the heck is it upper then lower
    evac_max = np.amax(evac_org, axis=2)
    evac_max = cv2.bitwise_and(evac_max, evac_max, mask=evac_sat_mask)
    evac_max = evac_max[:evac_height, :]
    
    #* CIRCLE DETECTION (using sat max)
    #^ DOM: To finetune the detection of circles(reduce false positives), can consider changing the following params:
    #^ (I still have no idea what dp does btw)
    #^ 1. min and max radius
    #^ 2. param1, which controls the sensitivity of the edge detection (gradient value); a higher value will reduce no. of circles detected
    #^ 3. param2, which controls the threshold for circle detection; a larger value will reduce no. of circles detected
    #^ Prev values were param1 = 200, param2 = 27

    circles = cv2.HoughCircles(evac_max, cv2.HOUGH_GRADIENT, dp, min_dist, param1 = param1 , param2=param2, minRadius= min_radius, maxRadius=max_radius)   
    balls = []
    # cv2.imshow("frame of ball", evac_max)
    if circles is not None: 
        for x, y, r in circles[0]:
            mask = np.zeros(evac_max.shape[:2], dtype=np.uint8)
            mask = cv2.circle(mask, (int(x),int(y)), int(r), 255, -1)
            ball_mask = cv2.inRange(evac_max, 0, u_blackforball) #! DOM: im not actually sure if the frame should be evac_max or evac_gray in this case
            ball_mask = cv2.bitwise_and(ball_mask, ball_mask, mask = mask)
            # cv2.imshow("ball_circle", mask)
            # cv2.imshow("ball", ball_mask)
            black_percent_ball = (np.sum(ball_mask) / 255) / (math.pi * (r ** 2))
            balls.append({
                    "x": x,
                    "y": y,
                    "r": r,
                    "black": black_percent_ball
                })
        curr = Task.BALL

        #& Debug balls
        # circles = np.uint16(np.around(circles))
        # for i in circles[0,:]:
        #     cv2.circle(evac_max,(i[0],i[1]),i[2],(0,255,0),2)
        #     cv2.circle(evac_max,(i[0],i[1]),2,(0,0,255),3)
        # cv2.imshow("ballz", cv2.pyrDown(evac_max))
        # print(balls)

        closest_ball = max(balls, key=lambda b: b['y'])
        print("Closest ball:", closest_ball) #& debug ball

        y_ball = closest_ball["y"]
        x_ball = closest_ball["x"] - centre_x_topcam
        rotation = math.atan2(x_ball, y_ball) * 180/math.pi if y != 0 else 0

        #~ Type of ball
        if closest_ball["black"] > 0.3: #! was previously 0.1
            ball_type = 1                                                                                                                                                           
        else:
            ball_type = 0
        
        #~ Power rotation
        if rotation < 0:
            rotation = -((-rotation/90) ** 0.5) * 90
        else:
            rotation = ((rotation/90) ** 0.5) * 90
        # print("rotation:", rotation)
        # print("task:", curr.value)
    
    #* NO BALL
    else:
        curr = Task.NOBALL
        rotation = 0
        ball_type = 1

    #* SEND PICO
    rotation = int(rotation)
    if rotation > 90:
        rotation = 90
    elif rotation < -90:
        rotation = -90
    rotation += 90

    curr = Task.NOBALL

    print("rotation", rotation)

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm_evac, # 0 to 200 MAX, but 100 really damn fast alr
                253, curr.value,
                249, ball_type] # 1: black, 0: silver
    
    print(to_pico)

    ser.write(to_pico)

#* MAIN FUNCTION ------------------------ EVAC FIND ALIVE DEPOSIT POINT ----------------------------------

def task_2_depositalive():
    
    global rotation, curr

    frame_org = bot_stream.read()
    out.write(frame_org)
    frame_org = cv2.pyrDown(frame_org, dstsize=(top_stream_width_org//2, top_stream_height_org//2))
    frame_org = cv2.pyrDown(frame_org, dstsize=(width_lt, height_lt))
    frame_org = cv2.flip(frame_org, 0)
    frame_org = cv2.flip(frame_org, 1)
    # frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_greenevac, u_greenevac)
    # mask_gs = cv2.erode(mask_gs, green_erode_kernel, iterations=1)
    # mask_gs = cv2.dilate(mask_gs, green_erode_kernel, iterations=1)
    # cv2.imshow("green", mask_green)
    green_sum = np.sum(mask_green)/255
    print("Green sum", green_sum)

    #~ Moving to green
    if 500 < green_sum:
        print("GREEN")
        
        #~ Minimum green width so the robot centres on green
        green_evac_col = np.amax(mask_green, axis=0)
        green_evac_indices = np.where(green_evac_col == 255)
        green_evac_start_x = green_evac_indices[0][0] if len(green_evac_indices[0]) else 0
        green_evac_end_x = green_evac_indices[0][-1] if len(green_evac_indices[0]) else 0
        evac_green_width = green_evac_end_x - green_evac_start_x
        print("Evac green width", evac_green_width)

        #~ Green confirmed
        if evac_green_width > 400: #! consider tuning this value (esp when bot is far)
            print("-"*30, "found", "-"*30)
            greenM = cv2.moments(mask_green)
            cx_green = int(greenM["m10"]/greenM["m00"])
            rotation = (cx_green-centre_x_botcam) / 20 #tune constant for rotating to deposit point
            curr = Task.DEPOSITALIVE

    #~ Still finding green
    else:
        rotation = 0
        curr = Task.EMPTYEVAC

    #* DATA
    if rotation > 90:
        rotation = 90
    elif rotation < -90:
        rotation = -90
    rotation = int(rotation) + 90

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm_evac,
                253, curr.value]
    ser.write(to_pico)

#* MAIN FUNCTION ------------------------ EVAC FIND DEAD DEPOSIT POINT ----------------------------------

def task_3_depositdead():
    
    global rotation, curr

    frame_org = bot_stream.read()
    out.write(frame_org)
    frame_org = cv2.flip(frame_org, 0)
    frame_org = cv2.flip(frame_org, 1)
    # frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_red1 = cv2.inRange(frame_hsv, l_red1evac, u_red1evac)
    mask_red2 = cv2.inRange(frame_hsv, l_red2evac, u_red2evac)
    mask_red = mask_red1 + mask_red2
    # cv2.imshow("red", mask_red)
    red_sum = np.sum(mask_red)/255
    print("Red sum", red_sum)

    #~ Moving to red
    if 400 < red_sum:
        print("RED")
        
        #~ Minimum red width so the robot centres on red
        red_evac_col = np.amax(mask_red, axis=0)
        red_evac_indices = np.where(red_evac_col == 255)
        red_evac_start_x = red_evac_indices[0][0] if len(red_evac_indices[0]) else 0
        red_evac_end_x = red_evac_indices[0][-1] if len(red_evac_indices[0]) else 0
        evac_red_width = red_evac_end_x - red_evac_start_x
        print("Evac red width", evac_red_width)

        #~ Red confirmed
        if evac_red_width > 400:
            redM = cv2.moments(mask_red)
            cx_red = int(redM["m10"]/redM["m00"])
            rotation = (cx_red-centre_x_botcam) / 20 #tune constant for rotating to deposit point
            curr = Task.DEPOSITDEAD

    #~ Still finding red
    else:
        rotation = 0
        curr = Task.EMPTYEVAC

    #* DATA
    if rotation > 90:
        rotation = 90
    elif rotation < -90:
        rotation = -90
    rotation = int(rotation) + 90

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm_evac,
                253, curr.value]
    ser.write(to_pico)

#* MAIN FUNCTION ------------------------ AFTER EVAC BACK TO LINETRACK ----------------------------------
        
def task4_backtolt():
    global rotation, curr

    frame_org = top_stream.read()
    out.write(frame_org)
    frame_org = cv2.pyrDown(frame_org, dstsize=(top_stream_width_org//2, top_stream_height_org//2))
    frame_org = cv2.pyrDown(frame_org, dstsize=(width_lt, height_lt))
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_black_org = cv2.inRange(frame_gray, 0, u_black_lineforltfromevac)

    mask_green = cv2.inRange(frame_hsv, l_greenevac, u_greenevac)
    mask_black = mask_black_org.copy() - mask_green

    mask_black[-crop_bh_evactolt:, :] = 0
    mask_black[:crop_th_evactolt, :] = 0
    mask_black = cv2.erode(mask_black, black_kernel)
    mask_black = cv2.dilate(mask_black, black_kernel)
    # cv2.imshow("black mask", mask_black)
    black_sum = np.sum(mask_black) / 255
    print("black sum", black_sum)

    if black_sum > 23000: #! tune this value
        black_col = np.amax(mask_black, axis=0)
        black_indices_x = np.where(black_col == 255)
        black_start_x = black_indices_x[0][0] if len(black_indices_x[0]) else 0
        black_end_x = black_indices_x[0][-1] if len(black_indices_x[0]) else 0
        black_width = black_end_x - black_start_x
        print("black width", black_width)

        if (black_width) > 520: 
            curr = Task.EMPTY

            blackM = cv2.moments(mask_black)
            cx_black = int(blackM["m10"]/blackM["m00"])
            rotation = (cx_black-centre_x_botcam) / 20 #! tune this value

        else:
            curr = Task.FINDINGLINE
    else:
        curr = Task.FINDINGLINE

    #* DATA
    if rotation > 90:
        rotation = 90
    elif rotation < -90:
        rotation = -90
    rotation = int(rotation) + 90

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm_evac,
                253, curr.value]
    ser.write(to_pico)
    
#* MAIN FUNCTION ---------------------------- LINETRACK SEE LINE ON RIGHT WHEN TURN LEFT ------------------------------------------------
def task5_leftlookright():
    global curr, rotation, rpm_lt, see_line

    frame_org = top_stream.read()
    out.write(frame_org)
    frame_org = cv2.pyrDown(frame_org, dstsize=(top_stream_width_org//2, top_stream_height_org//2))
    frame_org = cv2.pyrDown(frame_org, dstsize=(width_lt, height_lt))
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_greenlt, u_greenlt)

    mask_green_for_black = cv2.inRange(frame_hsv, l_greenlt_forblack, u_greenlt_forblack)
    mask_all_green = cv2.bitwise_or(mask_green, mask_green_for_black)
    mask_black_org = cv2.inRange(frame_gray, 0, u_black_lt) - mask_all_green

    #~ Obstacle see line
    obstacle_line_mask = mask_black_org.copy()
    obstacle_line_mask = cv2.bitwise_and(obstacle_line_mask, mask_trapeziums)
    obstacle_line_mask = cv2.erode(obstacle_line_mask, black_kernel)
    obstacle_line_mask = cv2.dilate(obstacle_line_mask, black_kernel)
    # obstacle_line_mask[:40, :] = 0 #^ xel: no need crop cuz we r checking the height 
    
    if np.sum(obstacle_line_mask):
        obstacle_line_mask, obstacle_line_y = contoursCalc(frame_org, obstacle_line_mask)
        obstacle_line_pixels = np.sum(obstacle_line_mask) / 255
        print("See line pixels:", obstacle_line_pixels)
        print(obstacle_line_y)

        if obstacle_line_pixels > 600 and obstacle_line_y > 55:
            obs_line_cols = np.amax(obstacle_line_mask, axis=0)
            obs_line_indices_x = np.where(obs_line_cols==255)
            obs_line_left = obs_line_indices_x[0][0] if len(obs_line_indices_x[0]) else 0
            obs_line_right = obs_line_indices_x[0][-1] if len(obs_line_indices_x[0]) else 0 
            obs_line_width = obs_line_right - obs_line_left
            print("right x", obs_line_right, " ||  obstacle left", obs_line_left)

            if obs_line_right > 120: 
                see_line = 1
                print('|'* 5, "SEE LINE", '|' * 5)
            else:
                see_line = 0
        else:
            see_line = 0
    else:
        see_line = 0
    # cv2.namedWindow("Line after obstacle", 2)
    # cv2.resizeWindow("Line after obstacle", 550, 100)
    # cv2.imshow("Line after obstacle", obstacle_line_mask) #& debug obstacle line


    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm_lt,
                253, curr.value,
                252, see_line]
    ser.write(to_pico)

#* MAIN FUNCTION ------------------------ LINETRACK SEE LINE ON LEFT WHEN TURN RIGHT ----------------------------------

def task6_rightlookleft():
    global curr, rotation, rpm_lt, see_line

    frame_org = top_stream.read()
    out.write(frame_org)
    frame_org = cv2.pyrDown(frame_org, dstsize=(top_stream_width_org//2, top_stream_height_org//2))
    frame_org = cv2.pyrDown(frame_org, dstsize=(width_lt, height_lt))
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_greenlt, u_greenlt)

    mask_green_for_black = cv2.inRange(frame_hsv, l_greenlt_forblack, u_greenlt_forblack)
    mask_all_green = cv2.bitwise_or(mask_green, mask_green_for_black)
    mask_black_org = cv2.inRange(frame_gray, 0, u_black_lt) - mask_all_green

    #~ Obstacle see line
    obstacle_line_mask = mask_black_org.copy()
    obstacle_line_mask = cv2.bitwise_and(obstacle_line_mask, mask_trapeziums)
    obstacle_line_mask = cv2.erode(obstacle_line_mask, black_kernel)
    obstacle_line_mask = cv2.dilate(obstacle_line_mask, black_kernel)
    # obstacle_line_mask[:40, :] = 0

    if np.sum(obstacle_line_mask):
        obstacle_line_mask, obstacle_line_y = contoursCalc(frame_org, obstacle_line_mask)
        obstacle_line_pixels = np.sum(obstacle_line_mask) / 255
        print("See line pixels:", obstacle_line_pixels)
        print(obstacle_line_y)

        if obstacle_line_pixels > 600 and obstacle_line_y > 55: 
            obs_line_cols = np.amax(obstacle_line_mask, axis=0)
            obs_line_indices_x = np.where(obs_line_cols==255)
            obs_line_left = obs_line_indices_x[0][0] if len(obs_line_indices_x[0]) else 0
            obs_line_right = obs_line_indices_x[0][-1] if len(obs_line_indices_x[0]) else 0 
            obs_line_width = obs_line_right - obs_line_left
            print("right x", obs_line_right, " ||  obstacle left", obs_line_left)
        
            if obs_line_left < 40:
                see_line = 1
                print('|'* 5, "SEE LINE", '|' * 5)
            else:
                see_line = 0
        else:
            see_line = 0

    else:
        see_line = 0

    # cv2.namedWindow("Line after obstacle", 2)
    # cv2.resizeWindow("Line after obstacle", 550, 100)
    # cv2.imshow("Line after obstacle", obstacle_line_mask) #& debug obstacle line

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm_lt,
                253, curr.value,
                252, see_line]
    ser.write(to_pico)

#* MAIN FUNCTION ------------------------ DIAGONAL LIDARS DETECT EXTRUSION; DETECT AMOUNT OF BLACK ----------------------------------

def task7_lt_to_evac(): 
    global rotation, rpm_lt, see_thin_line, end_line_gap, curr
    # global entered_evac

    frame_org = top_stream.read()[:360] #! TUNE
    # frame_org = cv2.pyrDown(frame_org, dstsize=(top_stream_width_org//2, top_stream_height_org//2))
    # frame_org = cv2.pyrDown(frame_org, dstsize=(width_lt, height_lt))
    out.write(frame_org)
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)
    frame_sat_mask = cv2.inRange(frame_hsv, u_sat_thresh, l_sat_thresh)

    #~ Mask out black
    mask_green = cv2.inRange(frame_hsv, l_greenlt, u_greenlt)
    mask_green_for_black = cv2.inRange(frame_hsv, l_greenlt_forblack, u_greenlt_forblack)
    mask_all_green = cv2.bitwise_or(mask_green, mask_green_for_black)
    mask_black_org = cv2.inRange(frame_gray, 0, u_black_lt) - mask_all_green
    print("original black sum", np.sum(mask_black_org)/255)
    # cv2.imshow("org black mask", mask_black_org) #& debug original black mask

    frame_max = np.amax(frame_org, axis=2)
    frame_max = cv2.bitwise_and(frame_max, frame_max, mask=frame_sat_mask)
    # cv2.imshow("frame max", frame_max)

    circles = cv2.HoughCircles(frame_max, cv2.HOUGH_GRADIENT, dp, min_dist, param1 = param1 , param2=param2, minRadius= min_radius, maxRadius=max_radius)   
    # cv2.imshow("frame of ball", evac_max)
    if circles is not None: 
        overall_mask = np.zeros(frame_org.shape[:2], dtype=np.uint8)
        for x, y, r in circles[0]:
            mask = np.zeros((frame_org.shape[:2]), dtype=np.uint8)
            mask = cv2.circle(mask, (int(x), int(y)), int(r), 255, -1)
            # ball_mask = cv2.inRange(frame_max, 0, u_blackforball) #? replaced frame_gray with frame_max
            # ball_mask = cv2.bitwise_and(ball_mask, ball_mask, mask = mask)
            # cv2.imshow("ball_circle", mask)
            # cv2.imshow("ball", ball_mask)
            overall_mask = cv2.bitwise_or(overall_mask, mask)

        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            cv2.circle(frame_max,(i[0],i[1]),i[2],(0,255,0),2)
            cv2.circle(frame_max,(i [0],i[1]),2,(0,0,255),3)

        # cv2.imshow("new frame max", frame_max)
    
        mask_black_org = mask_black_org - overall_mask
        # cv2.imshow("new mask black", mask_black_org)
        black_sum = np.sum(mask_black_org)/255
        print("new black sum", black_sum)

        if black_sum < 10000: #! TUNE
            entered_evac = True
            print("ENTERED EVAC!!")
        else:
            entered_evac = False
            task0_lt()
    else:
        black_sum = np.sum(mask_black_org)/255
        if black_sum < 10000:
            entered_evac = True
            print("ENTERED EVAC!!")
        else:
            entered_evac = False
            task0_lt()
    # else:
    #     task0_lt() #! consider creating special conditions for this

    print(entered_evac)
    curr = Task.EMPTY
    to_pico = [253, curr.value, 250, entered_evac]
    ser.write(to_pico)


#* ------------------------ RESPOND TO PICO ----------------------------------

while True:

    if top_stream.stopped: #or bot_stream.stopped:
        ser.write(253, curr.RED)
        break

    received_task = receive_pico()
    if received_task != -1:
        pico_task = int(received_task)

    start = time.time()
    if pico_task == 0:
        print("Linetrack")
        task0_lt()
    elif pico_task == 1:
        print("Evac looking for ball")
        task_1_ball()
    # elif pico_task == 2:
    #     print("Evac looking for alive deposit")
    #     task_2_depositalive()
    # elif pico_task == 3:
    #     print("Evac looking for dead deposit")
    #     task_3_depositdead()
    # elif pico_task == 4:
    #     print("Looking for linetrack")
    #     task4_backtolt()
    elif pico_task == 5:
        print("Obstacle turning left, looking at right of camera for line")
        task5_leftlookright()
    elif pico_task == 6:
        print("Obstacle turning right, looking at left of camera for line")
        task6_rightlookleft()
    elif pico_task == 7:
        task7_lt_to_evac()
        print("'\033[43m'", "Preparing to enter evac", "'\033[0m'")
    elif pico_task == 9:
        print("Switch off")
        gsVotes = [0, 0, 0]
        # task7_lt_to_evac()
    else:
        print("Pico task unknown:", pico_task)

    end = time.time()
    print("Loop time: ", end-start)

    #! remove b4 comp for optimisation and the video stream too
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

top_stream.stop()
# bot_stream.stop()
out.release()
cv2.destroyAllWindows()