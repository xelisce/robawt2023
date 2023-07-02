import cv2
from MultiThread import WebcamStream
import serial
import numpy as np
# from time import sleep

#* CAMERA
lt_stream = WebcamStream(stream_id=0)
lt_stream.start()
lt_frame = lt_stream.read()
width, height = lt_frame.shape[1], lt_frame.shape[0]
print("Line track camera width:", width, "Camera height:", height)

cam_x = width/2 #-1 to bias to the right 
cam_y = height - 1
print("Line track image centred on (" + cam_x, cam_y + ")")

#* SERIAL
ser = serial.Serial("/dev/ttyS0", 9600)

#* IMAGE PROCESSING
crop_h = 0 #! horizon, 35

l_black = 0
u_black = 75

x_com = y_com = np.zeros(shape=(height, width), dytpe=float)

for i in range(height):
    for j in range(width):
        x_com[i][j] = (j-cam_x) / (width/2)
        y_com[i][j] = (cam_y-i) / height

while True:
    if lt_stream.stopped:
        break
    
    frame_org = lt_stream.read()
    frame = frame_org[crop_h:height, :]
    frame_org_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)

    black_mask = cv2.inRange(frame_org_gray, l_black, u_black)
    # black_mask = cv2.erode(black_mask, kernel)
    # black_mask = cv2.dilate(black_mask, kernel)
    t, black_thresh = cv2.threshold(black_mask, )


    key = cv2.waitKey(1)
    if key == ord('q'):
        break

lt_stream.stop()
cv2.destroyAllWindows()
