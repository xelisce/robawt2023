import cv2 
import numpy as np
np.set_printoptions(threshold=1000) #^ to print entire array, use this np.inf and to truncate the array when printing, use the default 1000 or any length preferred
import enum
import math
import os

#* ----------------------------------------------=Initialising HSV Analyser=---------------------------------------------------------

# wd = os.getcwd()
# file = os.chdir("bawty\output.avi")
#^init the image stream and shit
def on_trackbar_change(x):
    global current_frame, frame_hsv
    current_frame = x
    top_stream.set(cv2.CAP_PROP_POS_FRAMES, current_frame)
    ret, frame_org = top_stream.read()
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)
    if ret:
        cv2.imshow("frame", frame_org)
        _mask_red1 = cv2.inRange(frame_hsv, l_red1, u_red1)
        _mask_red2 = cv2.inRange(frame_hsv, l_red2, u_red2)
        mask_red = _mask_red1 + _mask_red2
        res = cv2.bitwise_and(frame_hsv, frame_hsv, mask=mask_red)
        cv2.imshow('masked frame', res)
        # cv2.imshow("masked frame", frame_hsv)


top_stream = cv2.VideoCapture('rednorange.avi')
_, top_stream_frame = top_stream.read()
# top_stream_width, top_stream_height = top_stream_frame.shape[1], top_stream_frame.shape[0]
# print("Top camera width:", top_stream_width_org, "Camera height:", top_stream_height_org)

#^ hsv sliders
l_red1 = np.array([0, 0, 0], np.uint8)
u_red1 = np.array([10, 0, 0], np.uint8)
l_red2 = np.array([170, 0, 0], np.uint8)
u_red2 = np.array([180, 0, 0], np.uint8) 

def callback(x):
    # global l_hsv, u_hsv
    global frame_hsv
    # cv2.imshow('masked frame', frame_hsv)
    #Mask 1
    l_red1[0] = cv2.getTrackbarPos('L1 HUE', 'controls')
    l_red1[1] = cv2.getTrackbarPos('L1 SAT', 'controls')
    l_red1[2] = cv2.getTrackbarPos('L1 VAL', 'controls')
    u_red1[0] = cv2.getTrackbarPos('U1 HUE', 'controls')
    u_red1[1] = cv2.getTrackbarPos('U1 SAT', 'controls')
    u_red1[2] = cv2.getTrackbarPos('U1 VAL', 'controls')
    #Mask 2
    l_red2[0] = cv2.getTrackbarPos('L2 HUE', 'controls')
    l_red2[1] = cv2.getTrackbarPos('L2 SAT', 'controls')
    l_red2[2] = cv2.getTrackbarPos('L2 VAL', 'controls')
    u_red2[0] = cv2.getTrackbarPos('U2 HUE', 'controls')
    u_red2[1] = cv2.getTrackbarPos('U2 SAT', 'controls')
    u_red2[2] = cv2.getTrackbarPos('U2 VAL', 'controls')

    _mask_red1 = cv2.inRange(frame_hsv, l_red1, u_red1)
    _mask_red2 = cv2.inRange(frame_hsv, l_red2, u_red2)
    mask_red = _mask_red1 + _mask_red2
    res = cv2.bitwise_and(frame_hsv, frame_hsv, mask=mask_red)
    cv2.imshow('masked frame', res)


current_frame = 0
total_frames = int(top_stream.get(cv2.CAP_PROP_FRAME_COUNT)) #! idk why but for some reason this is rlly inaccurate
fps = top_stream.get(cv2.CAP_PROP_FPS)
print("Total frames: ", total_frames)
# print("seconds:", round(total_frames/fps))

frame_hsv = cv2.cvtColor(top_stream_frame, cv2.COLOR_BGR2HSV)

#^ init windows
#~ Original Frame w frame slider
cv2.namedWindow("frame", 2)
cv2.resizeWindow("frame", 550, 500)
cv2.createTrackbar("Frame", "frame", 0, 5000, on_trackbar_change)

#~ Masked Frame
cv2.namedWindow("masked frame", 2)
cv2.resizeWindow("masked frame", 550, 500)

#~ Hsv slider windows
cv2.namedWindow('controls', 2)
cv2.resizeWindow('controls', 550, 300) 

cv2.createTrackbar('L1 HUE', 'controls', 0, 180, callback)
cv2.createTrackbar('L1 SAT', 'controls', 0, 255, callback)
cv2.createTrackbar('L1 VAL', 'controls', 0, 255, callback)
cv2.createTrackbar('U1 HUE', 'controls', 180, 180, callback)
cv2.createTrackbar('U1 SAT', 'controls', 255, 255, callback)
cv2.createTrackbar('U1 VAL', 'controls', 255, 255, callback)

cv2.createTrackbar('L2 HUE', 'controls', 0, 180, callback)
cv2.createTrackbar('L2 SAT', 'controls', 0, 255, callback)
cv2.createTrackbar('L2 VAL', 'controls', 0, 255, callback)
cv2.createTrackbar('U2 HUE', 'controls', 0, 180, callback)
cv2.createTrackbar('U2 SAT', 'controls', 0, 255, callback)
cv2.createTrackbar('U2 VAL', 'controls', 0, 255, callback)

#* -----------------------------------==========END INITIALISATION OF HSV ANALYSER=========--------------------------------------

while True:

    top_stream.set(cv2.CAP_PROP_POS_FRAMES, current_frame) # shows stepped through frame
    ret, frame_org = top_stream.read()
    frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)
    if not ret:
        print("end of video")
        print(current_frame)
        break

    cv2.imshow("frame", frame_org)
    cv2.imshow("masked frame", frame_hsv)
    print(current_frame)

    #^ various keybinds
    key = cv2.waitKey(0)
    if key == ord('q'): # quit 
        break
    elif key == ord('s'): # skip to frame count
        current_frame = int(input("Enter frame to jump to: "))
        cv2.setTrackbarPos("Frame", "frame", current_frame)
    elif key == ord('n'): # next frame
        current_frame += 1 
        if current_frame >= total_frames:
            current_frame = total_frames
        cv2.setTrackbarPos("Frame", "frame", current_frame)
    elif key == ord('b'): # previous frame
        current_frame -= 1
        if current_frame < 0:
            current_frame = 0
        cv2.setTrackbarPos("Frame", "frame", current_frame)

top_stream.release()
cv2.destroyAllWindows()