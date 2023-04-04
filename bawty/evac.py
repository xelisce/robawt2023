import cv2
from MultiThread import WebcamStream
import numpy as np
import serial
import enum
import math

#* SERIAL
ser = serial.Serial("/dev/serial0", 9600)

rpm = 25

#* IMAGE START
evac_stream = WebcamStream(stream_id=2)
evac_stream.start()
evac_org = evac_stream.read()
width, height_org = evac_org.shape[1], evac_org.shape[0]
print("Evac camera width:", width, "Camera height:", height_org)

crop_h_evac = 100
kp_ball = 1

height = height_org - crop_h_evac
# u_black = 55

u_sat_thresh = np.array([0, 0, 0], np.uint8)
l_sat_thresh = np.array([180, 100, 255], np.uint8)

dp = 1.2
min_dist = 70
param1 = 184
param2 = 28
min_radius = 40
max_radius = 170

l_red1 = np.array([0, 100, 80], np.uint8) #! untuned red values
u_red1 = np.array([15, 255, 255], np.uint8)
l_red2 = np.array([170, 100, 80], np.uint8) 
u_red2 = np.array([180, 255, 255], np.uint8) #! 179 or 180?

#~ Real values
# l_green = np.array([30, 50, 60], np.uint8)
# u_green = np.array([85, 255, 255], np.uint8)
#~ My house's values
l_green = np.array([70, 90, 30], np.uint8)
u_green = np.array([96, 255, 255], np.uint8)

class Task(enum.Enum):
    NOBALL = 20
    BALL = 21
    DEPOSITALIVE = 22
    DEPOSITDEAD = 23
    EMPTYEVAC = 24

def receive_pico() -> int:
    global ser
    if(ser.in_waiting > 0):
        received_data = ser.read()
        data_left = ser.inWaiting()
        received_data += ser.read(data_left)
        return ord(received_data)
    else:
        return 0

while True:
    if evac_stream.stopped:
        break

    pico_task = receive_pico()
    if pico_task == 2:
        break #start looking for evac point
    
    #* IMAGE SETUP

    evac_org = evac_stream.read()
    evac_hsv = cv2.cvtColor(evac_org, cv2.COLOR_BGR2HSV)
    evac_max = np.amax(evac_org, axis=2)
    
    evac_sat_mask = cv2.inRange(evac_hsv, u_sat_thresh, l_sat_thresh)
    evac_max = cv2.bitwise_and(evac_max, evac_max, mask=evac_sat_mask)
    evac_max = evac_max[:height, :]

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
        # mask_max = np.zeros(evac_max.shape[:2], dtype=np.uint8)
        # mask_max = cv2.circle(mask_max, (int(x),int(y)), int(r), 255, -1)
        for x, y, r in circles[0]:
            balls.append({
                    "x": x,
                    "y": y,
                    "r": r,
                }) #! add standard deviation for sorting (later)
            
        curr = Task.BALL

        #& debug balls
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            cv2.circle(evac_max,(i[0],i[1]),i[2],(0,255,0),2)
            cv2.circle(evac_max,(i[0],i[1]),2,(0,0,255),3)
        cv2.imshow("ballz", evac_max)
        
        # print(balls)
        biggest_ball = max(balls, key=lambda b: b['r'])
        print("Biggest ball:", biggest_ball)

        y_ball = biggest_ball["y"]
        x_ball = biggest_ball["x"]
        rotation = math.atan2(x, y) * 180/math.pi if y != 0 else 0

        print("rotation:", rotation)
        rotation *= kp_ball
        # print("task:", curr.value)
    
    #* NO BALL
    else:
        curr = Task.NOBALL
        rotation = 0

    rotation = int(rotation)
    if rotation > 90:
        rotation = 90
    elif rotation < -90:
        rotation = -90
    rotation += 90

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm, # 0 to 200 MAX, but 100 really damn fast alr
                253, curr.value]
    ser.write(to_pico)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

evac_stream.stop()
cv2.destroyAllWindows()

#* LOOKING FOR EVAC POINT *******************************************

#* IMAGE START
lt_stream = WebcamStream(stream_id=0)
lt_stream.start()
frame_org = lt_stream.read()
width, height_org = frame_org.shape[1], frame_org.shape[0]
print("Evac camera width:", width, "Camera height:", height_org)

deposit_now = 1 #deposit alive first
rpm = 30
centre_x = width/2

# green_erode_kernel = np.ones((3, 3), np.uint8)

print("Done")

while True:
    frame_org = lt_stream.read()
    frame_org = cv2.flip(frame_org, 0)
    frame_org = cv2.flip(frame_org, 1)
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_green, u_green)
    # mask_gs = cv2.erode(mask_gs, green_erode_kernel, iterations=1)
    # mask_gs = cv2.dilate(mask_gs, green_erode_kernel, iterations=1)
    cv2.imshow("green", mask_green)
    green_sum = np.sum(mask_green)/255
    print(green_sum)

    mask_red1 = cv2.inRange(frame_hsv, l_red1, u_red1)
    mask_red2 = cv2.inRange(frame_hsv, l_red2, u_red2)
    mask_red = mask_red1 + mask_red2
    cv2.imshow("red", mask_red)
    red_sum = np.sum(mask_red)/255

    #* Deposit alive
    if deposit_now == 1:
        if green_sum > 200:
            print("GREEN")
            greenM = cv2.moments(mask_green)
            cx_green = int(greenM["m10"]/greenM["m00"])
            rotation = (cx_green-centre_x) / 20 #tune constant for rotating to deposit point
            curr = Task.DEPOSITALIVE
        else:
            rotation = 0
            curr = Task.EMPTYEVAC

    #* Deposit dead
    else:
        if red_sum > 200:
            print("RED")
            curr = Task.DEPOSITDEAD
        else:
            curr = Task.EMPTYEVAC

    #* DATA
    if rotation > 90:
        rotation = 90
    elif rotation < -90:
        rotation = -90
    rotation = int(rotation) + 90

    to_pico = [255, rotation, # 0 to 180, with 0 actually being -90 and 180 being 90
                254, rpm,
                253, curr.value]
    
    ser.write(to_pico)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

print("Back to linetrack")

lt_stream.stop()
cv2.destroyAllWindows()
