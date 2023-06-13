import cv2
from MultiThread import WebcamStream
import numpy as np
import serial
import enum
import math

#* SERIAL
ser = serial.Serial("/dev/serial0", 9600)

#* CAMERAS START
bot_stream = WebcamStream(stream_id=0)
bot_stream.start()
bot_stream_frame = bot_stream.read()
bot_stream_width, bot_stream_height_org = bot_stream_frame.shape[1], bot_stream_frame.shape[0]
print("Bottom camera width:", bot_stream_width, "Camera height:", bot_stream_height_org)

top_stream = WebcamStream(stream_id=2)
top_stream.start()
top_stream_frame = top_stream.read()
top_stream_width, top_stream_height_org = top_stream_frame.shape[1], top_stream_frame.shape[0]
print("Top camera width:", top_stream_width, "Camera height:", top_stream_height_org)

#* LINE TRACK CONSTANTS
kp_lt = 1
height_lt = bot_stream_height_org
centre_x_lt = bot_stream_width/2

black_kernel = np.ones((7, 7), np.uint8)
black_linegap_kernel = np.ones((10, 10), np.uint8)

#~ Vectors
x_com = np.tile(np.linspace(-1., 1., bot_stream_width), (height_lt, 1)) #reps is (outside, inside)
y_com = np.array([[i] * bot_stream_width for i in np.linspace(1., 0, height_lt)])
x_linegap_com = x_com.copy()

#^ Glenda's method\]=#```~ Powering x component with respect to y
# x_com_scale = ((1-y_com) ** 0.3)
# x_com *= x_com_scale
#^ Kenneth's method:
#~ Powering x component
x_com[:, :int(bot_stream_width/2)] *= -1
x_com = x_com ** 1
x_com[:, :int(bot_stream_width/2)] *= -1
#~ Powering y component
# y_com = y_com ** 2

#~ Linegap Com
x_linegap_com[:, :int(bot_stream_width/2)] *= -1
x_linegap_com = x_linegap_com ** 0.1
x_linegap_com[:, :int(bot_stream_width/2)] *= -1

#~ Triangle mask
peak_triangle_gap = 0
peak_triangle_width = 60
triangle_margin = 20
mask_gap = np.zeros([height_lt, bot_stream_width], dtype="uint8")
points = np.array([[triangle_margin, height_lt-100], [bot_stream_width-triangle_margin, height_lt-100], [int(bot_stream_width/2+peak_triangle_width), int(height_lt/2)], [int(bot_stream_width/2-peak_triangle_width),int(height_lt/2)]])
cv2.fillConvexPoly(mask_gap, points, 255)

#~ GS
gs_roi_h = 310 #the crop height for gs  
gs_bksampleoffset = 10 #offset sample above green squares
gs_bksampleh = 40
gs_minbkpct = 0.35 #! to be tuned
gs_minarea = 4000 #min pixels
gs_erode_kernel = np.ones((3, 3), np.uint8)
redsilver_kernel = np.ones((7,7), np.uint8)

#~ Linegap
crop_h_bw = 93
crop_h_bw_gap = 110
gap_check_h = 155 # was 150, 170, 180
horizon_crop_h = 40
crop_bh_silver = 100 #top camera
crop_th_silver = 180

#~ RK
b_minarea = 6000

#~ PID
rpm_setptlt = 41

#* EVAC CONSTANTS
crop_h_evac = 100
evac_height = top_stream_height_org-crop_h_evac
height_evac_t = top_stream_height_org - crop_h_evac

centre_x_botcam = bot_stream_width/2
centre_x_topcam = top_stream_width/2

kp_ball = 1
rpm_evac = 30 #not actually used

# u_sat_thresh = np.array([0, 0, 0], np.uint8)
# l_sat_thresh = np.array([180, 255, 255], np.uint8)

crop_bh_evactolt = 120 #with top cam
crop_th_evactolt = 100 #with top cam



#* IMAGE THRESHOLDS

# ~ Club values

# l_red1evac = np.array([0, 150, 100], np.uint8)
# u_red1evac = np.array([10, 255, 255], np.uint8)
# l_red2evac = np.array([175, 150, 100], np.uint8) 
# u_red2evac = np.array([180, 255, 255], np.uint8)

# l_greenlt = np.array([65, 55, 65], np.uint8)
# u_greenlt = np.array([80, 255, 255], np.uint8)
# l_greenlt_alt = np.array([65, 55, 65], np.uint8)
# u_greenlt_alt = np.array([80, 220, 255], np.uint8)

# l_red1lt = np.array([0, 100, 80], np.uint8)
# u_red1lt = np.array([20, 255, 255], np.uint8)
# l_red2lt = np.array([170, 100, 80], np.uint8) 
# u_red2lt = np.array([180, 255, 255], np.uint8)

# l_red1silver = np.array([0, 20, 165], np.uint8) 
# u_red1silver = np.array([15, 255, 255], np.uint8)
# l_red2silver = np.array([170, 20, 165], np.uint8) 
# u_red2silver = np.array([180, 255, 255], np.uint8) 

# l_blue = np.array([96, 170, 80], np.uint8)
# u_blue = np.array([106, 245, 191], np.uint8)

# l_greenevac = np.array([80, 100, 5], np.uint8)
# u_greenevac = np.array([100, 255, 255], np.uint8)

# l_red1evac_top = np.array([0, 110, 110], np.uint8)
# u_red1evac_top = np.array([10, 255, 255], np.uint8)
# l_red2evac_top = np.array([175, 110, 110], np.uint8) 
# u_red2evac_top = np.array([180, 255, 255], np.uint8)

# l_greenevac_top = np.array([75, 80, 95], np.uint8)
# u_greenevac_top = np.array([91, 255, 255], np.uint8)

# u_black_lineforltfromevac = 70 #! didnt tune
# u_black = 110
# u_blackforball = 40

# #* HOUGH CIRCLE PARAMETERS
# dp = 3
# min_dist = 77 #67
# param1 = 198 #128
# param2 = 73 #62
# min_radius = 65
# max_radius = 88



#~ COMPETITION VALUES (WITH HAT ON)

#! below are not actually used !#
# l_red1silver = np.array([0, 1, 200], np.uint8) 
# u_red1silver = np.array([10, 255, 255], np.uint8)
# l_red2silver = np.array([175, 1, 200], np.uint8) 
# u_red2silver = np.array([180, 255, 255], np.uint8) 

# l_red1evac = np.array([0, 150, 100], np.uint8)
# u_red1evac = np.array([10, 255, 255], np.uint8)
# l_red2evac = np.array([175, 150, 100], np.uint8) 
# u_red2evac = np.array([180, 255, 255], np.uint8)

# l_greenevac = np.array([70, 80, 10], np.uint8)
# u_greenevac = np.array([90, 255, 255], np.uint8)

#? absolute useless
# l_greenevac_top = np.array([60, 70, 40], np.uint8)
# u_greenevac_top = np.array([80, 255, 255], np.uint8)

# l_red1evac_top = np.array([0, 100, 80], np.uint8)
# u_red1evac_top = np.array([10, 255, 255], np.uint8)
# l_red2evac_top = np.array([170, 100, 80], np.uint8) 
# u_red2evac_top = np.array([180, 255, 255], np.uint8)
# crop_h_evactolt = 180
#! end unused values !#


l_blue = np.array([100, 135, 40], np.uint8)
u_blue = np.array([115, 255, 255], np.uint8)

l_red1lt = np.array([0, 130, 105], np.uint8)
u_red1lt = np.array([10, 255, 255], np.uint8)
l_red2lt = np.array([170, 96, 84], np.uint8) 
u_red2lt = np.array([180, 255, 255], np.uint8)

l_greenlt = np.array([60, 80, 100], np.uint8) #alternate values: 50,50,90
u_greenlt = np.array([80, 255, 255], np.uint8)
l_greenlt_alt = np.array([60, 70, 90], np.uint8) #! not sure if this works
u_greenlt_alt = np.array([80, 220, 255], np.uint8)


dp = 3
min_dist = 77 #67
param1 = 191 #128
param2 = 103 #62
min_radius = 65
max_radius = 88


#! top camera green and red
u_blackforball = 49

u_black = 60

u_black_lineforltfromevac = 55 #! didnt tune (from 50)


#~ COMPETITION VALUES (WITHOUT HAT ON)

# l_red1evac = np.array([0, 150, 70], np.uint8) #calibbed on bright field
# u_red1evac = np.array([15, 255, 255], np.uint8)
# l_red2evac = np.array([170, 150, 70], np.uint8) 
# u_red2evac = np.array([180, 255, 255], np.uint8)

l_red1evac = np.array([0, 60, 20], np.uint8)
u_red1evac = np.array([15, 255, 255], np.uint8)
l_red2evac = np.array([170, 60, 20], np.uint8) 
u_red2evac = np.array([180, 255, 255], np.uint8)

l_greenevac = np.array([70, 35, 20], np.uint8)
u_greenevac = np.array([95, 255, 255], np.uint8)

#! red silver
l_red1silver = np.array([0, 15, 171], np.uint8) #!lowered sat from 15 to 12
u_red1silver = np.array([10, 40, 255], np.uint8)
l_red2silver = np.array([170, 15, 171], np.uint8) 
u_red2silver = np.array([180, 40, 255], np.uint8) 



#* VARIABLE INITIALISATIONS
class Task(enum.Enum):
    #~ Linetrack
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
    # SILVER = 12
    #~ Evac
    NOBALL = 20
    BALL = 21
    DEPOSITALIVE = 22
    DEPOSITDEAD = 23
    EMPTYEVAC = 24
    FINDINGLINE = 25


rotation = 0
curr = Task.EMPTY
rpm_lt = rpm_setptlt
see_line = 0
ball_type = 1
silver_line = 0
end_line_gap = 0
seesaw = 1
new_endlinegap = 1

red_now = False
gs_now = False
blue_now = False

pico_task = 0

pi_start = True

#* FUNCTIONS

def receive_pico() -> str:
    if(ser.in_waiting > 0):
        received_data = ser.read()
        data_left = ser.inWaiting()
        received_data += ser.read(data_left)
        # print(received_data)
        # print("pico data:", ord(received_data))
        cleanstring = received_data.split(b'\x00',1)
        if len(cleanstring):
            pico_data = cleanstring[0].decode()
            if len(pico_data) and len(pico_data[0]):
                print("decode:", pico_data[0])
                return pico_data[0]
            else:
                return -1 #empty string
        else:
            return -1 #\x00 is the only byte
    else:
        return -1 #no info

def debug_silvertape():
    frame_org = top_stream.read()
    frame_org = cv2.flip(frame_org, 0)
    frame_org = cv2.flip(frame_org, 1)
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    #~ Old method standard deviation
    # mask_green = cv2.inRange(frame_hsv, l_greenevac, u_greenevac)
    # t, thresh = cv2.threshold(frame_gray, u_black_lineforltfromevac, 255 ,cv2.THRESH_BINARY_INV)
    # st = see_entry(frame_gray, thresh, mask_green, 43, 15000000, 150, 120)
    # print(st)

    mask_red1 = cv2.inRange(frame_hsv, l_red1silver, u_red1silver)
    mask_red2 = cv2.inRange(frame_hsv, l_red2silver, u_red2silver)
    mask_red = mask_red1 + mask_red2
    mask_red[-crop_bh_silver:, :] = 0
    mask_red[:crop_th_silver, :] = 0 #? redundant?
    red_sum = np.sum(mask_red) / 255
    # cv2.imshow("red mask", mask_red)
    # cv2.imshow("original", frame_org)

    # print(red_sum)

    if 500 < red_sum < 1500:
        print("silver")

def sort_contours(cnts):
    boundingBoxes = [cv2.boundingRect(c) for c in cnts]
    (cnts, boundingBoxes) = zip(*sorted(zip(cnts, boundingBoxes), key=lambda b:b[1][1], reverse=True))
    return (cnts, boundingBoxes)

def draw_contour(image, c, i):
    # compute the center of the contour area and draw a circle
    # representing the center
    M = cv2.moments(c)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    cv2.putText(image, "#{}".format(i + 1), (cX - 20, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    # return the image with the contour number drawn on it
    return image

#* MAIN FUNCTION ------------------------ MAIN LINETRACK ----------------------------------

def task0_lt():
    global rotation, rpm_lt, curr, red_now, gs_now, blue_now, silver_line, end_line_gap, pi_start, seesaw, new_endlinegap

    frame_org = bot_stream.read()
    frame_org = cv2.flip(frame_org, 0)
    frame_org = cv2.flip(frame_org, 1)
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_greenlt, u_greenlt)
    mask_gs = mask_green.copy()
    mask_gs[:gs_roi_h, :] = 0
    mask_gs = cv2.erode(mask_gs, gs_erode_kernel, iterations=1)
    mask_gs = cv2.dilate(mask_gs, gs_erode_kernel, iterations=1)
    gs_sum = np.sum(mask_gs)/255
    # cv2.imshow("green square mask", frame_org[gs_roi_h:, :])
    # cv2.imshow("green square mask", mask_gs) #& debug green square mask
    # print("Green sum:", gs_sum) #& debug green min pixels

    mask_green_alt = cv2.inRange(frame_hsv, l_greenlt_alt, u_greenlt_alt)
    mask_both_green = cv2.bitwise_or(mask_green, mask_green_alt)
    mask_black_org = cv2.inRange(frame_gray, 0, u_black) - mask_both_green
    # cv2.imshow('black frame', black_mask) #& debug black mask
    # print(np.sum(mask_black), curr.name)
 
    mask_red1 = cv2.inRange(frame_hsv, l_red1lt, u_red1lt)
    mask_red2 = cv2.inRange(frame_hsv, l_red2lt, u_red2lt)
    mask_red = mask_red1 + mask_red2
    red_sum = np.sum(mask_red) / 255
    # print("Red sum:", red_sum) #& debug red min area

    mask_blue = cv2.inRange(frame_hsv, l_blue, u_blue) 
    mask_blue[:horizon_crop_h, :] = 0
    blue_sum = np.sum(mask_blue) / 255
    # print("Blue sum:", blue_sum) #& debug blue min area
    # print("green sum:", gs_sum)
    #* RED LINE 

    if red_sum > 60000:
        red_now = True
        curr = Task.RED
        rpm_lt = 0
        rotation = 0
    elif red_now == True:
        red_now = False
        rpm_lt = rpm_setptlt
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
        gs_top = g_indices_v[0][0]
        gs_bot = g_indices_v[0][-1]
        # cv2.imshow("Green bounding box", frame_org[gs_top:, gs_left:gs_right]) #& debug green bounds

        #~ Wait for green to reach last 5 pixels
        if gs_bot > 475:

            #~ Find x positions of green
            green_col = np.amax(mask_gs, axis=0)
            g_indices_h = np.where(green_col==255) #h for horizontal
            gs_left = g_indices_h[0][0]
            gs_right = g_indices_h[0][-1]
            gs_width = gs_right - gs_left
            # print("left:", gs_left, "right:", gs_right)
            print("width of greensquare: ", gs_width)

            #~ Test if below or above line
            gs_bkabove = mask_black_org[gs_roi_h - gs_bksampleoffset - gs_bksampleh : gs_roi_h - gs_bksampleoffset, gs_left : gs_right]
            gs_bkpct = np.sum(gs_bkabove) / 255 / gs_bksampleh / (gs_right - gs_left)
            # print("Percentage of black above green:", gs_bkpct) #& debug green's black
            # cv2.imshow("black area above green square", gs_bkabove) 
            # cv2.imshow("black area above green in orig frame", frame_org[gs_top - gs_bksampleoffset - gs_bksampleh : gs_top - gs_bksampleoffset, gs_left : gs_right])
            
            print("green sum:", gs_sum)

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

                print("green sum:", gs_sum)
                #~ Identify type of green square
                if cx_black > gs_left and cx_black < gs_right and gs_sum > 35000 and gs_width > 600: #largest width of single green is ~400 i think
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

        else:
            gs_prev_sum = np.sum(mask_gs[gs_top:gs_bot, :]) / 255
            print("far away green:", gs_prev_sum)
    
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
        finalx = cx_blue - centre_x_lt

        #~ If the block isn't centred
        if abs(finalx) >= 60: #^ DOM: not refined yet; consider doing abs(...) >= 100
            if cx_blue < centre_x_lt: #if rescue kit is to the left of the centre of the frame
                rotation = -45 # fixed rotation of the bot
                rpm_lt = 20
            elif cx_blue > centre_x_lt: #to the right
                rotation = 45
                rpm_lt = 20

        #~ Else if the block is (relatively) centred:
        else:
            curr = Task.BLUE

    #~ No longer sees the rescue kit
    elif blue_now and blue_sum < 2000: 
        blue_now = False
        rpm_lt = rpm_setptlt

    #* LINETRACK

    #^ DOM: FRIENDLY REMINDER that the coordinates of the image start from 0 at the top of the frame, to 480 at the bottom.
    #TODO: 1. Perhaps add a minimum width for the bot to actually linetrack on? if not just ignore/linegap

    if not gs_now and not red_now and not blue_now:

        #~ Silver line
        frame_top_org = top_stream.read()
        frame_top_gray = cv2.cvtColor(frame_top_org, cv2.COLOR_BGR2GRAY)
        frame_top_gray = frame_top_gray[:-100, :]
        frame_top_hsv = cv2.cvtColor(frame_top_org, cv2.COLOR_BGR2HSV)

        mask_top_black = cv2.inRange(frame_top_gray, 0, u_black_lineforltfromevac)
        black_top_sum = np.sum(mask_top_black) / 255
        print("black_top_sum", black_top_sum)

        mask_top_red1 = cv2.inRange(frame_top_hsv, l_red1silver, u_red1silver)
        mask_top_red2 = cv2.inRange(frame_top_hsv, l_red2silver, u_red2silver)
        mask_top_red = mask_top_red1 + mask_top_red2
        # cv2.imshow("before erode dilate", mask_top_red)
        mask_top_red = cv2.dilate(mask_top_red, redsilver_kernel)
        mask_top_red = cv2.erode(mask_top_red, redsilver_kernel)
        mask_top_red[-crop_bh_silver:, :] = 0
        mask_top_red[:crop_th_silver, :] = 0
        red_top_sum = np.sum(mask_top_red) / 255
        # cv2.imshow("red mask", mask_top_red) #& debug silver line
        print("red top sum", red_top_sum)
        # cv2.imshow("original", frame_top_org[crop_th_silver:top_stream_height_org-crop_bh_silver, :])

        if 1000 < red_top_sum < 7000 and red_sum < 16000 and black_top_sum < 10000: #!black_top_sum was previously 550000 so that was possibly why it was randomly entering evac during Robocup lol
            print('|'* 30, "silver", '|'* 30)
            silver_line = 1
        else:
            silver_line = 0


        #~ Image processing erode-dilate
        curr = Task.EMPTY
        # mask_black_org = mask_black_org - mask_top_red

        mask_black = mask_black_org.copy()
        mask_black = cv2.erode(mask_black, black_kernel)
        mask_black = cv2.dilate(mask_black, black_kernel)
        mask_uncropped_black = mask_black.copy()
        mask_supercrop_black = mask_black.copy()
        black_newlinegap_mask = mask_black.copy()

        mask_black[:horizon_crop_h, :] = 0
        mask_supercrop_black[:-gap_check_h-40, :] = 0 
        mask_supercrop_black[-40:, :] = 0 
        # black_newlinegap_mask[:]
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
        print("black start: ", black_start_y, "white: start", white_start_y)
        print("black line height: ", black_line_height)

        #~ Finding next line segment
        black_row[white_start_y:] = 0
        black_indices_v2 = np.where(black_row == 255)
        black_second_start_y = black_indices_v2[0][-1] if len(black_indices_v2[0]) else 0
        black_row[black_second_start_y:] = 255
        white_indices_v2 = np.where(black_row == 0)
        white_second_start_y = white_indices_v2[0][-1] if len(white_indices_v2[0]) else 0
        print("2nd black: ", black_second_start_y, "2nd white:", white_second_start_y) #& debug height of second line

        #~ Finding left and right index of black (based off supercropped black)
        black_col = np.amax(mask_supercrop_black, axis=0)
        black_indices_h = np.where(black_col == 255)
        black_start_x = black_indices_h[0][0] if len(black_indices_h[0]) else 0
        black_end_x = black_indices_h[0][-1] if len(black_indices_h[0]) else 0
        black_line_width = black_end_x-black_start_x

        print("x width:", black_line_width) #& debug width of line

        #~ Next line segment 
        black_second_mask = mask_black.copy()
        black_second_mask[:white_second_start_y, :] = 0
        black_second_mask[black_second_start_y:, :] = 0
        black_second_sum = np.sum(black_second_mask) /255

        if black_second_sum > 0: #^ getting (offsetted) width of second line segment 
            black_second_row = np.amax(black_second_mask[black_second_start_y-35:black_second_start_y, :], axis=0)
            black_second_indices = np.where(black_second_row == 255)
            black_second_mask_start_x = black_second_indices[0][0] if len(black_second_indices[0]) else 0
            black_second_mask_end_x = black_second_indices[0][-1] if len(black_second_indices[0]) else 0
            black_second_mask_width = black_second_mask_end_x - black_second_mask_start_x

            print("second x width:", black_second_mask_width) #& debug width of line
            print("black second pixels sum:", black_second_sum)
            # cv2.imshow("second line", black_second_mask)

        else:
            black_second_mask_width = 0

        #^ supposedly accounts for the stupid gaps between tiles (hopefully works now)
        print("minigap", white_start_y - black_second_start_y)
        if white_start_y - black_second_start_y < 22 and white_start_y > 140 and black_second_start_y > 300: #? technically white_start_y > 140 is redundant but wtv
            white_start_y = white_second_start_y
            black_line_height = black_start_y - white_start_y #^ adjusts the new height so bot stops wiggling
            print("/" * 20 + "minigap detected" + "\\" * 20)
            print("new white_start_y", white_start_y)
            print("new black line height: ", black_line_height)

        else:
             #~ Contours for weird close national tile
            if white_start_y < 140:
                # cv2.imshow("before contours", mask_black)
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
                    # white_start_y = int(closest_contour["y"] - closest_contour["h"])
                    contour_mask = np.zeros([height_lt, bot_stream_width], dtype="uint8")
                    cv2.rectangle(contour_mask,(closest_contour["x"],closest_contour["y"]),(closest_contour["x"]+closest_contour["w"],closest_contour["y"]+closest_contour["h"]), 255 , -1)
                    # print("after contours, white start y:", white_start_y)
                    # cv2.imshow("contours", contour_mask)

                    mask_black = cv2.bitwise_and(mask_black, contour_mask)

        #~ If line gap (line ending and line width small) 
        if ((black_start_y < 460 or white_start_y > 300) and black_line_width < 300): #or (black_second_start_y > 150 and white_second_start_y < 150):
                if black_second_start_y <= 120 and not 80 <= black_second_mask_width <= 600: #or black_start_y < 200:
                    curr = Task.LINEGAP
                    print(">" * 15 + 'LINE GAP' + '<' * 15)
                mask_linegap_black = mask_black_org.copy()
                mask_linegap_black = cv2.erode(mask_linegap_black, kernel=black_linegap_kernel)
                mask_linegap_black = cv2.dilate(mask_linegap_black, kernel=black_linegap_kernel)
                mask_black = cv2.bitwise_and(mask_gap, mask_linegap_black)
                x_black_com = x_linegap_com
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
            # cv2.imshow("continuous", mask_black)

            curr = Task.EMPTY
            x_black_com = x_com

            #~ Plain line track
            powered_y = (height_lt-40)/black_line_height if black_line_height != 0 else 1
            powered_y = powered_y ** 0.5
            powered_y = min(3.5, powered_y) #? prev value: 4

        #~ Conditions for ending line sweep:
        #^ First condition:
        #? Force bot to move forward when encoder vals remain relatively unchanged for ~5 sec?
        if (black_start_y > 200 and white_start_y < 200 and black_line_width > 100) or (black_second_start_y > 120 and white_second_start_y < 120 and black_second_sum > 3500 and 80 < black_second_mask_width < 600): #!!! white2ndstart = 120, 80<black_second_mask_width<600
            #!! whitestarty < 165, white_2nd_start < 110, 70<black_second_width<600
            curr = Task.EMPTY
            end_line_gap = 1
            x_black_com = x_com
            # cv2.imshow("offset black mask", mask_black[200:250, :])
            print("-" * 40 + "end line gap" + "-" * 40)

        #^ Second condition: (do you want curr to go to task.EMPTY ?)
        #? i don't rmb what this does (i assume its for the oscillation motion)
        #? oh we could probs make the marching ants an actual standalone case
        #? + i foresee it potentially moving outside of a tile and catching onto another line seg in another tile lol
        #TODO: Finetune the sum for new_endlinegap (max possible pixels based on current mask is 51000)

        black_newlinegap_mask = cv2.bitwise_and(black_newlinegap_mask, mask_gap)
        #~ Width of new linegap
        new_mask_cols = np.amax(mask_gap, axis=0)
        new_mask_line_indices_x = np.where(new_mask_cols==255)
        new_mask_line_start_x = new_mask_line_indices_x[0][0] if len(new_mask_line_indices_x[0]) else 0
        new_mask_line_end_x = new_mask_line_indices_x[0][-1] if len(new_mask_line_indices_x[0]) else 0
        new_mask_width = new_mask_line_end_x - new_mask_line_start_x
        black_newlinegap_sum = np.sum(black_newlinegap_mask) / 255 

        
        #^ used with alternate linegap
        if black_newlinegap_sum > 10000 and 80<new_mask_width<600 : #sum was previously 500
            new_endlinegap = 1
            print(";" * 43, "new linegap", ";"*43)
        else:
            new_endlinegap = 0

        # attempt at seesaw which apparently wasnt even being used
        # if black_line_height > 250 and 100 < black_line_width < 460 and black_start_x > 450 and white_start_y < 450:
        #     seesaw = 1
        # else:
        #     seesaw = 0

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
        y_resultant = np.mean(y_black) ** powered_y
        x_resultant = np.mean(x_black)
        #& debug
        # print("power:", powered_y)
        # cv2.imshow("yframe", y_black)
        # cv2.imshow("xframe", x_black)
        # print(y_resultant, x_resultant)

        #~ Formatting data for transfer
        angle = math.atan2(x_resultant, y_resultant) * 180/math.pi if y_resultant != 0 else 0
        rotation = angle * kp_lt

    # print("b4 processing rotation:", rotation)

    if rotation > 90:
        rotation = 90
    elif rotation < -90:
        rotation = -90
    rotation = int(rotation) + 90

    #* SEND DATA
    print("value:", curr)
    # print("rpm_lt:", rpm_lt)
    # # print("rotation:", rotation)

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm_lt,
                253, curr.value,
                251, end_line_gap,
                249, silver_line,
                248, 1,
                # 247, seesaw,
                246, new_endlinegap]

    # print(to_pico)
    
    ser.write(to_pico)

#* MAIN FUNCTION ------------------------ EVAC FIND BALL ----------------------------------

def task_1_ball():
    global rotation, curr, ball_type

    #* IMAGE SETUP
    evac_org = top_stream.read()
    # evac_hsv = cv2.cvtColor(evac_org, cv2.COLOR_BGR2HSV) #? not currently used
    evac_gray = cv2.cvtColor(evac_org, cv2.COLOR_BGR2GRAY)
    evac_max = np.amax(evac_org, axis=2)
    
    # evac_sat_mask = cv2.inRange(evac_hsv, u_sat_thresh, l_sat_thresh)
    # evac_max = cv2.bitwise_and(evac_max, evac_max, mask=evac_sat_mask)
    evac_max = evac_max[:evac_height, :]
    
    #* CIRCLE DETECTION
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
            mask = np.zeros(evac_org.shape[:2], dtype=np.uint8)
            mask = cv2.circle(mask, (int(x),int(y)), int(r), 255, -1)
            ball_mask = cv2.inRange(evac_gray, 0, u_blackforball) #? try replacing with evac_max instead?
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

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm_evac, # 0 to 200 MAX, but 100 really damn fast alr
                253, curr.value,
                250, ball_type] # 1: black, 0: silver
    
    print(to_pico)

    ser.write(to_pico)

#* MAIN FUNCTION ------------------------ EVAC FIND ALIVE DEPOSIT POINT ----------------------------------

def task_2_depositalive():
    
    global rotation, curr

    frame_org = bot_stream.read()
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

#* MAIN FUNCTION ------------------------ LINETRACK SEE LINE ON RIGHT WHEN TURN LEFT ----------------------------------

def task5_leftlookright():
    global curr, rotation, rpm_lt, see_line

    frame_org = bot_stream.read()
    frame_org = cv2.flip(frame_org, 0)
    frame_org = cv2.flip(frame_org, 1)
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_greenlt, u_greenlt)
    mask_black_org = cv2.inRange(frame_gray, 0, u_black) - mask_green

    #~ Obstacle see line
    obstacle_line_mask = mask_black_org.copy()
    obstacle_line_mask = cv2.erode(obstacle_line_mask, black_kernel)
    obstacle_line_mask = cv2.dilate(obstacle_line_mask, black_kernel)
    obstacle_line_mask[:height_lt-60, :] = 0
    obstacle_line_pixels = np.sum(obstacle_line_mask) / 255

    if obstacle_line_pixels > 10000:
        obs_line_cols = np.amax(obstacle_line_mask, axis=0)
        obs_line_indices_x = np.where(obs_line_cols==255)
        # obs_line_start_x = obs_line_indices_x[0][0] if len(obs_line_indices_x[0]) else 0
        obs_line_end_x = obs_line_indices_x[0][-1] if len(obs_line_indices_x[0]) else 0

        print("end x", obs_line_end_x)
        if obs_line_end_x > 610:
            see_line = 1
            print('|'* 5, "SEE LINE", '|' * 5)
        else:
            see_line = 0
    else:
        see_line = 0

    print("See line pixels:", obstacle_line_pixels)
    # cv2.imshow("Line after obstacle", obstacle_line_mask) #& debug obstacle line


    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm_lt,
                253, curr.value,
                252, see_line]
    ser.write(to_pico)

#* MAIN FUNCTION ------------------------ LINETRACK SEE LINE ON RIGHT WHEN TURN LEFT ----------------------------------

def task6_rightlookleft():
    global curr, rotation, rpm_lt, see_line

    frame_org = bot_stream.read()
    frame_org = cv2.flip(frame_org, 0)
    frame_org = cv2.flip(frame_org, 1)
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_greenlt, u_greenlt)
    mask_black_org = cv2.inRange(frame_gray, 0, u_black) - mask_green

    #~ Obstacle see line
    obstacle_line_mask = mask_black_org.copy()
    obstacle_line_mask[:height_lt-60, :] = 0
    obstacle_line_pixels = np.sum(obstacle_line_mask) / 255

    if obstacle_line_pixels > 10000:
        obs_line_cols = np.amax(obstacle_line_mask, axis=0)
        obs_line_indices_x = np.where(obs_line_cols==255)
        obs_line_start_x = obs_line_indices_x[0][0] if len(obs_line_indices_x[0]) else 0
        # obs_line_end_x = obs_line_indices_x[0][-1] if len(obs_line_indices_x[0]) else 0

        print("start x", obs_line_start_x)
        if obs_line_start_x < 70:
            see_line = 1
            print('|'* 5, "SEE LINE", '|' * 5)
        else:
            see_line = 0
    else:
        see_line = 0

    print("See line pixels:", obstacle_line_pixels)
    # cv2.imshow("Line after obstacle", obstacle_line_mask) #& debug obstacle line

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm_lt,
                253, curr.value,
                252, see_line]
    ser.write(to_pico)


#* ------------------------ RESPOND TO PICO ----------------------------------

while True:
    if top_stream.stopped or bot_stream.stopped:
        break

    received_task = receive_pico()
    if received_task != -1:
        pico_task = int(received_task)

    # task4_backtolt()
    # debug_silvertape()
    # task5_leftlookright()
    

    if pico_task == 0:
        print("Linetrack")
        task0_lt()
    elif pico_task == 1:
        print("Evac looking for ball")
        task_1_ball()
    elif pico_task == 2:
        print("Evac looking for alive deposit")
        task_2_depositalive()
    elif pico_task == 3:
        print("Evac looking for dead deposit")
        task_3_depositdead()
    elif pico_task == 4:
        print("Looking for linetrack")
        task4_backtolt()
    elif pico_task == 5:
        print("Obstacle turning left, looking at right of camera for line")
        task5_leftlookright()
    elif pico_task == 6:
        print("Obstacle turning right, looking at left of camera for line")
        task6_rightlookleft()
    elif pico_task == 9:
        print("Switch off")
        task0_lt()
    else:
        print("Pico task unknown:", pico_task)

    #! remove b4 comp for optimisation
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

top_stream.stop()
bot_stream.stop()
cv2.destroyAllWindows()

#* ------------------------ UNUSED FUNCTIONS ----------------------------------

# def task4OLDOLDOLD_backtolt():
#     global rotation, curr

#     frame_org = bot_stream.read()
#     frame_org = cv2.flip(frame_org, 0)
#     frame_org = cv2.flip(frame_org, 1)
#     frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
#     frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

#     mask_black_org = cv2.inRange(frame_gray, 0, u_black_lineforltfromevac)

#     mask_green = cv2.inRange(frame_hsv, l_greenevac, u_greenevac)

#     mask_black = mask_black_org.copy() - mask_green
#     mask_black[:-crop_h_evactolt, :] = 0
#     # cv2.imshow("black mask", mask_black)
#     black_sum = np.sum(mask_black) / 255

#     if black_sum > 20: #! tune this value
#         black_col = np.amax(mask_black, axis=0)
#         black_indices_x = np.where(black_col == 255)
#         black_start_x = black_indices_x[0][0] if len(black_indices_x[0]) else 0
#         black_end_x = black_indices_x[0][-1] if len(black_indices_x[0]) else 0
#         black_width = black_end_x - black_start_x
#         print(black_width)

#         if (black_width) > 400: #! tune this value too
#             curr = Task.EMPTY

#             blackM = cv2.moments(mask_black)
#             cx_black = int(blackM["m10"]/blackM["m00"])
#             rotation = (cx_black-centre_x_botcam) / 20 #! also tune this value

#         else:
#             curr = Task.FINDINGLINE
#     else:
#         curr = Task.FINDINGLINE

#     #* DATA
#     if rotation > 90:
#         rotation = 90
#     elif rotation < -90:
#         rotation = -90
#     rotation = int(rotation) + 90

#     to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
#                 254, rpm_evac,
#                 253, curr.value]
#     ser.write(to_pico)



# def see_entry(gray, thresh, green, min_std_dev, min_mask_sum, t_crop, b_crop) -> bool:
#     """Ensure that max(thresh) = 255!"""
#     not_green = cv2.bitwise_not(green)
#     not_black = cv2.bitwise_not(thresh)
#     st_mask = cv2.bitwise_and(not_black, not_green)

#     st_mask_c = st_mask[t_crop:(st_mask.shape[0] - b_crop), :]
#     gray_c = gray[t_crop:(gray.shape[0] - b_crop), :]

#     cv2.imshow("st_mask_c", st_mask_c)
#     cv2.imshow("gray_c", gray_c)

#     _, std = cv2.meanStdDev(gray_c ,mask = st_mask_c)
#     print(std, np.sum(st_mask))
#     return (std[0][0] > min_std_dev and np.sum(st_mask_c) > min_mask_sum)