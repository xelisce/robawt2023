
#^ HOW TO USE cropcalib.py
# Drag slider to adjust crop height.
# To switch between cameras, select the cv2 window and press 0 for linetrack (bottom camera)
# and 1 for evac (top camera) for a short while (~ 1 sec)
# The cropping will be done to the top of the linetrack camera, and bottom of evac camera
# Press 'q' to quit

import cv2
from MultiThread import WebcamStream

cam_stream = WebcamStream(stream_id=0)
cam_stream.start()
frame_org = cam_stream.read()
height = frame_org.shape[0]
cam_type = 0

crop_h = 0
bw_thresh = 90

def update(val):
    global crop_h
    crop_h = val

frame_org = frame_org[crop_h:height, :]
gray_org = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
t, thresh = cv2.threshold(gray_org, bw_thresh, 255, cv2.THRESH_BINARY_INV)
cv2.imshow('cropped_frame', thresh)
cv2.createTrackbar('crop_h', 'cropped_frame', 0, height, update)

while True:

    if cv2.waitKey(1) == ord('0') and cam_type != 0:
        cam_stream.stop()
        cam_stream = WebcamStream(stream_id=0)
        cam_stream.start()
        cam_type = 0

    if cv2.waitKey(1) == ord('1') and cam_type != 1:
        cam_stream.stop()
        cam_stream = WebcamStream(stream_id=2)
        cam_stream.start()
        cam_type = 1

    if not cam_stream.stopped:
        frame_org = cam_stream.read()
        if cam_type == 0:
            frame_org = cv2.flip(frame_org, 0)
            frame_org = cv2.flip(frame_org, 1)
            frame_org = frame_org[crop_h:, :]
        else:
            frame_org = frame_org[:height-crop_h, :]
        gray_org = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
        t, thresh = cv2.threshold(gray_org, bw_thresh, 255, cv2.THRESH_BINARY_INV)
        cv2.imshow('original cropped frame', frame_org)
        cv2.imshow('cropped_frame', thresh)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()