import cv2
from MultiThread import WebcamStream
import numpy as np
import serial
import enum
import math

evac_stream = WebcamStream(stream_id=2)
evac_stream.start()
evac_org = evac_stream.read()
width, height_org = evac_org.shape[1], evac_org.shape[0]
print("Line track camera width:", width, "Camera height:", height_org)

#* SERIAL
ser = serial.Serial("/dev/serial0", 9600)

rpm = 25

crop_h = 183
kp_ball = 1.5

height = height_org - crop_h
# u_black = 55

u_sat_thresh = np.array([0, 0, 0], np.uint8)
l_sat_thresh = np.array([180, 100, 255], np.uint8)

class Task(enum.Enum):
    EMPTY = 0
    LEFT_GREEN = 1
    RIGHT_GREEN = 2
    DOUBLE_GREEN = 3
    RED = 4
    BEFORE_BLUE = 5  #Turning towards blue #^ TEMPORARY
    BLUE = 6
    BLUEFINAL = 7
    NOBALL = 9
    BALL = 10

while True:
    if evac_stream.stopped:
        break
    
    #* IMAGE SETUP

    evac_org = evac_stream.read()
    evac_hsv = cv2.cvtColor(evac_org, cv2.COLOR_BGR2HSV)
    evac_max = np.amax(evac_org, axis=2)
    
    evac_sat_mask = cv2.inRange(evac_hsv, u_sat_thresh, l_sat_thresh)
    evac_max = cv2.bitwise_and(evac_max, evac_max, mask=evac_sat_mask)
    evac_max = evac_max[crop_h:, :]

    #* CIRCLE DETECTION
    #^ DOM: To finetune the detection of circles(reduce false positives), can consider changing the following params:
    #^ (I still have no idea what dp does btw)
    #^ 1. min and max radius
    #^ 2. param1, which controls the sensitivity of the edge detection (gradient value); a higher value will reduce no. of circles detected
    #^ 3. param2, which controls the threshold for circle detection; a larger value will reduce no. of circles detected
    #^ Prev values were param1 = 200, param2 = 27

    circles = cv2.HoughCircles(evac_max, cv2.HOUGH_GRADIENT, 1.2, 70, param1=273, param2=30, minRadius=60, maxRadius=100)
    balls = []
    if circles is not None:
        for x, y, r in circles[0]:
            mask_max = np.zeros(evac_max.shape[:2], dtype=np.uint8)
            mask_max = cv2.circle(mask_max, (int(x),int(y)), int(r), 255, -1)
            balls.append({
                    "x": x,
                    "y": y,
                    "r": r,
                })
            
        curr = Task.BALL
        
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
                253, curr.value] #currently 0 to 5
    ser.write(to_pico)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

evac_stream.stop()
cv2.destroyAllWindows()