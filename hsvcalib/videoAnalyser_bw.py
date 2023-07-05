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
    global current_frame, frame_gray
    current_frame = x
    top_stream.set(cv2.CAP_PROP_POS_FRAMES, current_frame)
    ret, frame_org = top_stream.read()
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    if ret:
        cv2.imshow("frame", frame_org)
        black_thresh = cv2.getTrackbarPos('black thresh', 'controls')
        mask_black = cv2.inRange(frame_gray, 0, black_thresh)
        # t, thresh = cv2.threshold(frame_gray, bw_thresh, 255, cv2.THRESH_BINARY_INV)

        # res = cv2.bitwise_and(frame_gray, frame_gray, mask=mask_black)
        cv2.imshow('masked frame', mask_black)

        # cv2.imshow("masked frame", frame_hsv)


top_stream = cv2.VideoCapture('output.avi')
_, top_stream_frame = top_stream.read()
# top_stream_width, top_stream_height = top_stream_frame.shape[1], top_stream_frame.shape[0]
# print("Top camera width:", top_stream_width_org, "Camera height:", top_stream_height_org)

#^ hsv sliders
black_thresh = 100

def callback(x):
    # global l_hsv, u_hsv
    global frame_gray
    # cv2.imshow('masked frame', frame_hsv)
    #Mask 1
    black_thresh = cv2.getTrackbarPos('black thresh', 'controls')

    mask_black = cv2.inRange(frame_gray, 0, black_thresh)
    # res = cv2.bitwise_and(frame_gray, frame_gray, mask=mask_black)
    cv2.imshow('masked frame', mask_black)

current_frame = 0
total_frames = int(top_stream.get(cv2.CAP_PROP_FRAME_COUNT)) #! idk why but for some reason this is rlly inaccurate
fps = top_stream.get(cv2.CAP_PROP_FPS)
print("Total frames: ", total_frames)
# print("seconds:", round(total_frames/fps))

frame_gray = cv2.cvtColor(top_stream_frame, cv2.COLOR_BGR2GRAY)

#^ init windows
#~ Original Frame w frame slider
cv2.namedWindow("frame", 2)
cv2.resizeWindow("frame", 550, 500)
cv2.createTrackbar("Frame", "frame", 0, total_frames, on_trackbar_change)

#~ Masked Frame
cv2.namedWindow("masked frame", 2)
cv2.resizeWindow("masked frame", 550, 500)

#~ Hsv slider windows
cv2.namedWindow('controls', 2)
cv2.resizeWindow('controls', 550, 300) 

cv2.createTrackbar('black thresh', 'controls', 0, 255, callback)

#* -----------------------------------==========END INITIALISATION OF HSV ANALYSER=========--------------------------------------

while True:

    top_stream.set(cv2.CAP_PROP_POS_FRAMES, current_frame) # shows stepped through frame
    ret, frame_org = top_stream.read()
    # frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)
    frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    if not ret:
        print("end of video")
        print(current_frame)
        break

    cv2.imshow("frame", frame_org)
    cv2.imshow("masked frame", frame_gray)
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