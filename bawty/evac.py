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

#* EVAC CONSTANTS
crop_h_evac = 100
height_evac_t = top_stream_height_org - crop_h_evac

centre_x_botcam = bot_stream_width/2
centre_x_topcam = top_stream_width/2

kp_ball = 1
rpm_evac = 30 #not actually used
# u_black = 55

#* HOUGH CIRCLE PARAMETERS
dp = 3
min_dist = 67
param1 = 128
param2 = 62
min_radius = 65
max_radius = 88

#* IMAGE PROCESSING THRESHOLDS
u_sat_thresh = np.array([0, 0, 0], np.uint8)
l_sat_thresh = np.array([180, 100, 255], np.uint8)

#~ Real values (probably for red line)
# l_red1 = np.array([0, 100, 80], np.uint8)
# u_red1 = np.array([15, 255, 255], np.uint8)
# l_red2 = np.array([170, 100, 80], np.uint8) 
# u_red2 = np.array([180, 255, 255], np.uint8)
#~ My house's values (day for evac red)
l_red1 = np.array([0, 90, 20], np.uint8)
u_red1 = np.array([15, 255, 255], np.uint8)
l_red2 = np.array([170, 90, 20], np.uint8) 
u_red2 = np.array([180, 255, 255], np.uint8)

#~ Real values
# l_green = np.array([30, 50, 60], np.uint8)
# u_green = np.array([85, 255, 255], np.uint8)
#~ My house's values (day) 
# l_green = np.array([70, 90, 30], np.uint8)
# u_green = np.array([96, 255, 255], np.uint8)
#~ My house's values (night)
l_green = np.array([55, 171, 5], np.uint8)
u_green = np.array([96, 255, 255], np.uint8)

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
    #~ Evac
    NOBALL = 20
    BALL = 21
    DEPOSITALIVE = 22
    DEPOSITDEAD = 23
    EMPTYEVAC = 24

rotation = 0
curr = Task.EMPTY
ball_type = 1

pico_task = 0

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
            if len(pico_data[0]):
                print("decode:", pico_data[0])
                return pico_data[0]
            else:
                return -1 #empty string
        else:
            return -1 #\x00 is the only byte
    else:
        return -1 #no info
    
#* MAIN FUNCTION ------------------------ EVAC FIND BALL ----------------------------------

def task_1_ball():
    global rotation, curr, ball_type

    #* IMAGE SETUP
    evac_org = bot_stream.read()
    evac_hsv = cv2.cvtColor(evac_org, cv2.COLOR_BGR2HSV)
    evac_gray = cv2.cvtColor(evac_org, cv2.COLOR_BGR2GRAY)
    evac_max = np.amax(evac_org, axis=2)
    
    evac_sat_mask = cv2.inRange(evac_hsv, u_sat_thresh, l_sat_thresh)
    evac_max = cv2.bitwise_and(evac_max, evac_max, mask=evac_sat_mask)

    #* CIRCLE DETECTION
    #^ DOM: To finetune the detection of circles(reduce false positives), can consider changing the following params:
    #^ (I still have no idea what dp does btw)
    #^ 1. min and max radius
    #^ 2. param1, which controls the sensitivity of the edge detection (gradient value); a higher value will reduce no. of circles detected
    #^ 3. param2, which controls the threshold for circle detection; a larger value will reduce no. of circles detected
    #^ Prev values were param1 = 200, param2 = 27

    circles = cv2.HoughCircles(evac_max, cv2.HOUGH_GRADIENT, dp, min_dist, param1=param1, param2=param2, minRadius=min_dist, maxRadius=max_radius)
    balls = []
    if circles is not None:
        for x, y, r in circles[0]:
            mask = np.zeros(evac_org.shape[:2], dtype=np.uint8)
            mask = cv2.circle(mask, (int(x),int(y)), int(r), 255, -1)
            # cv2.imshow("ball_circle", mask)
            ball_mask = cv2.inRange(evac_gray, 0, 40)
            ball_mask = cv2.bitwise_and(ball_mask, ball_mask, mask = mask)
            # cv2.imshow("ball", ball_mask)
            black_percent_ball = (np.sum(ball_mask) / 255) / (math.pi * r * r)
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
        if closest_ball["black"] > 0.1 :
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
                253, curr.value]
    #TODO: add back ball type to send to pico (removed cuz encountered serial comm problem - task was reading as the ball type)
    
    print(to_pico)

    ser.write(to_pico)

#* MAIN FUNCTION ------------------------ EVAC FIND ALIVE DEPOSIT POINT ----------------------------------

def task_2_depositalive():
    
    global rotation, curr

    frame_org = bot_stream.read()
    frame_org = cv2.flip(frame_org, 0)
    frame_org = cv2.flip(frame_org, 1)
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_green, u_green)
    # mask_gs = cv2.erode(mask_gs, green_erode_kernel, iterations=1)
    # mask_gs = cv2.dilate(mask_gs, green_erode_kernel, iterations=1)
    cv2.imshow("green", mask_green)
    green_sum = np.sum(mask_green)/255
    print("Green sum", green_sum)

    #~ Moving to green
    if 200 < green_sum:
        print("GREEN")
        
        #~ Minimum green width so the robot centres on green
        green_evac_col = np.amax(mask_green, axis=0)
        green_evac_indices = np.where(green_evac_col == 255)
        green_evac_start_x = green_evac_indices[0][0] if len(green_evac_indices[0]) else 0
        green_evac_end_x = green_evac_indices[0][-1] if len(green_evac_indices) else 0
        evac_green_width = green_evac_end_x - green_evac_start_x
        print("Evac green width", evac_green_width)

        #~ Green confirmed
        if evac_green_width > 400:
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
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_red1 = cv2.inRange(frame_hsv, l_red1, u_red1)
    mask_red2 = cv2.inRange(frame_hsv, l_red2, u_red2)
    mask_red = mask_red1 + mask_red2
    cv2.imshow("red", mask_red)
    red_sum = np.sum(mask_red)/255
    print("Red sum", red_sum)

    #~ Moving to red
    if 200 < red_sum:
        print("RED")
        
        #~ Minimum red width so the robot centres on red
        red_evac_col = np.amax(mask_red, axis=0)
        red_evac_indices = np.where(red_evac_col == 255)
        red_evac_start_x = red_evac_indices[0][0] if len(red_evac_indices[0]) else 0
        red_evac_end_x = red_evac_indices[0][-1] if len(red_evac_indices) else 0
        evac_red_width = red_evac_end_x - red_evac_start_x
        print("Evac green width", evac_red_width)

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

#* RESPOND TO PICO

while True:
    if top_stream.stopped or bot_stream.stopped:
        break

    received_task = receive_pico()
    if received_task != -1:
        pico_task = received_task

    if int(pico_task) == 0:
        print("Linetrack")
    elif int(pico_task) == 1:
        print("Evac looking for ball")
        task_1_ball()
    elif int(pico_task) == 2:
        print("Evac looking for alive deposit")
        task_2_depositalive()
    elif int(pico_task) == 3:
        print("Evac looking for dead deposit")
        task_3_depositdead()
    else:
        print("Pico task:", pico_task)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

top_stream.stop()
bot_stream.stop()
cv2.destroyAllWindows()