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
ser = serial.Serial("/dev/ttyS0", 9600)

#* COLOR CALIBRATION
# l_black = 0
u_black = 70

l_green = np.array([30, 50, 60], np.uint8)
u_green = np.array([85, 255, 255], np.uint8)

l_blue = np.array([114, 4, 159], np.uint8)  #! to tune the blue values
u_blue = np.array([133, 110, 255], np.uint8)

l_red1 = np.array([0, 100, 80], np.uint8) #! untuned red values
u_red1 = np.array([15, 255, 255], np.uint8)
l_red2 = np.array([170, 100, 80], np.uint8) 
u_red2 = np.array([180, 255, 255], np.uint8) #! 179 or 180?

#* CONSTANTS CALIBRATION
gs_roi_h = 300 #the crop height for gs #! increase after tuning  
#? DOM: what the hell is roi 
#? XEL: region of interest pfff
gs_bksampleoffset = 10 #to offset sample above green squares
gs_bksampleh = 40
gs_minbkpct = 0.35 #! to be tuned properly #DOM: this is gs_minimum_black_percentage
gs_minarea = 4000000 #^ consider making this scaled by pixels (/255)

b_minarea = 10000 #! ~DOM: to tune the values 

rpm_setpt = 40
rpm = 40
kp = 1.5 #constant used for PID?

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
    RED = 4
    BLUE = 5  #rescue kit LOL

curr = Task.EMPTY
red_now = False
gs_now = False
blue_now = False #^ not sure if this is necessary; might be quite superfluous

#~ Rescue kit
reversing_now = False #for rescue kit
moving_towards_blue = False #^ IM SORRY I KNOW THIS IS DISGUSTING :sob: lmao :eyes:
journey = [] #to record the rotations of the robot


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
    red_sum = np.sum(mask_red)
    # print(np.sum(mask_red)) #& debug red min area

    mask_blue = cv2.inRange(frame_hsv, l_blue, u_blue) 
    #? DOM: Are there any specific restrictions for the bluemask eg. cropped height/width 
    #? XEL: yeah depending on like how near u want the cube to be sensed, but this should only be done inside your np.sum if else to optimise performance
    blue_sum = np.sum(mask_blue)
    #print(blue_sum) #& debug red min & max area

    #* RED LINE 

    if red_sum > 17000000:
        red_now = True
        curr = Task.RED
        rpm = 0
    elif red_now == True:
        red_now = False
        rpm = rpm_setpt

    #* JUST FOUND GREEN SQUARE PREVIOUSLY
    #TODO XEL make single green squares for longer

    #~ Turn while still seeing green
    elif gs_now and gs_sum < 1000:
        if curr.name == "DOUBLE_GREEN":
            if np.sum(mask_black) > 18000000:
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
        #cv2.imshow("Green bounding box", frame_org[gs_top:, gs_left:gs_right]) #& debug green bounds

        if gs_bot > 470:
            #~ Find x positions of green
            green_col = np.amax(mask_gs, axis=0)
            g_indices_h = np.where(green_col==255) #h for horiontal
            gs_left = g_indices_h[0][0]
            gs_right = g_indices_h[0][-1]
            # print("left:", gs_left, "right:", gs_right)

            #~ Test if below or above line
            gs_bkabove = mask_black[gs_top - gs_bksampleoffset - gs_bksampleh : gs_top - gs_bksampleoffset, gs_left : gs_right]
            gs_bkpct = np.sum(gs_bkabove) / 255 / gs_bksampleh / (gs_right - gs_left)
            # print("Percentage of black above green:", gs_bkpct) #& debug green's black
            # cv2.imshow("black area above green square", gs_bkabove) 
            # cv2.imshow("black area above green in orig frame", frame_org[gs_top - gs_bksampleoffset - gs_bksampleh : gs_top - gs_bksampleoffset, gs_left : gs_right])
            
            #~ GREEN SQUARE FOUND
            if gs_bkpct > gs_minbkpct:

                #~ Find x position of green
                gs_centre = (gs_left + gs_right) / 2

                #~ Find x position of black
                gs_bkbeside = mask_black[gs_roi_h:, :]
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

    #TODO: LOGIC HANDLING FOR WHEN + AFTER THE BOT PICKS UP THE RESCUE KIT?

    #* REVERSING MOVEMENTS (RESCUE KIT)
    #^ DOM: thought process: reverse translation, then reverse rotation; handle movement on pico side
    #^ Instead of time.time(), use TOF sensors to record time taken for bot to move
    #^ XEL: the tof sensors dont read time btw you'll have to use millis() or elapsedMillis() on the pico
    if reversing_now: 
    #? XEL: ^ shouldnt this be elif because red takes precedence (line 122)
        #~ If not enough time has elapsed since bot started moving backwards
        if time.time() - stop_blue < (stop_blue - start_blue): 
            curr = Task.BLUE #^ rpm will be negative on pico side
            rpm = 40 
        #~ If bot still has to rotate to its original rotation
        elif len(journey) > 0:
            curr = Task.EMPTY #^ A stopgap for now; more sophisticated way of doing this?
            #print(len(journey)) #& debug rotations of bot
            rotation = -(journey.pop())  #negative of the angle? adding 90 since original rotation ranges from 0 to 180 #XEL: removed the 90
            print(rotation)
        else:
            blue_now = False
            reversing_now = False; 

    #* DETECTION OF RESCUE KIT

    #~ If close enough to rescue kit:
    #? XEL: Shouldnt this one also be elif
    if blue_sum >= 2000000: #^ DOM: I foresee some problems when bot tries to pick up block & still sees it? add additional condition of not reversing_now?
        #time.sleep(2) #^ DOM: I assume the bot will pick up the cube here somehow?

        stop_blue = time.time()
        reversing_now = True
        moving_towards_blue = False #^ sorry 
        # time_elapsed = stop_blue - start_blue
        # dist_travelled = rpm * time_elapsed

    #~ Rescue kit spotted:
    elif not reversing_now and blue_sum >= b_minarea:
        blue_now = True
        #mask_gs = cv2.erode(mask_gs, gs_erode_kernel, iterations=1)
        #mask_gs = cv2.dilate(mask_gs, gs_erode_kernel, iterations=1)
        #? XEL: can we remove these two lines ^^

        #~ Find x and y positions of rescue kit
        blueM = cv2.moments(mask_blue)
        cx_blue = int(blueM["m10"] / blueM["m00"])
        cy_blue = int(blueM["m01"] / blueM["m00"])
        finalx = cx_blue - frame_org.shape[1]/2
        finaly = frame_org.shape[0]/2 - cy_blue

        #~ if the block isn't centred
        if finalx != 0: 
            deviation = math.atan(finalx/finaly) * 180/math.pi #conversion of angle from bot to rescue kit into degrees 
            pid_deviation = deviation * kp

            if pid_deviation > 90:
                pid_deviation = 90
            elif pid_deviation < -90:
                pid_deviation = -90
            rotation = int(pid_deviation) + 90
            journey.append(rotation)
        elif not moving_towards_blue: #^ if block is centred: start timer and start moving towards block (maybe track the distance travelled too?)
            start_blue = time.time() #time when bot starts moving towards blue
            moving_towards_blue = True #^ this is dumbbb but its to ensure that this code runs only once


    #* LINETRACK

    if not gs_now and not red_now and not blue_now:

        curr = Task.EMPTY
        mask_gap = mask_black[crop_h_bw_gap:, :] 
        mask_black = mask_black[crop_h_bw:, :]
        # cv2.imshow("black lt mask", mask_gap)
        #^ maybe try, not needed (og kernel=5,5)
        # black_mask = cv2.erode(black_mask, kernel) 
        # black_mask = cv2.dilate(black_mask, kernel)

        #~ Vectorizing the black components
        y_black = cv2.bitwise_and(y_com, y_com, mask = mask_black)
        x_black = cv2.bitwise_and(x_com, x_com, mask = mask_black)
        # cv2.imshow("yframe", y_black)
        # cv2.imshow("xframe", x_black)

        #~ Line gap

        # print("max black:", np.max(y_black))

        #~ Plain line track
        y_resultant = np.mean(y_black)
        x_resultant = np.mean(x_black)

        #~ Formatting data for transfer
        angle = math.atan2(x_resultant, y_resultant) * 180/math.pi if y_resultant != 0 else 0
        # print(y_resultant, x_resultant) #& debug resultant angle
        # print(angle)
        rotation = angle * kp
    
    #* SEND DATA TO PICO

    # print("rpm:", rpm) #& debug sent variables
    # print("rotation:", rotation)
    # print("task:", curr.value)

    rotation = int(rotation)
    if rotation > 90:
        rotation = 90
    elif rotation < -90:
        rotation = -90
    rotation += 90

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm, # 0 to 200 MAX, but 100 really damn fast alr
                253, curr.value] #currently 0 to 5

    ser.write(to_pico)

    key = cv2.waitKey(1) #! remove for optimisation before robocup
    if key == ord('q'):
        break

lt_stream.stop()
cv2.destroyAllWindows()
