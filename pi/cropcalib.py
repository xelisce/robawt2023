import cv2
from MultiThread import WebcamStream

cam_stream = WebcamStream(stream_id=0)
cam_stream.start()
frame_org = cam_stream.read()
height = frame_org.shape[0]

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

    if cam_stream.stopped:
        break

    frame_org = cam_stream.read()
    frame_org = frame_org[crop_h:height, :]
    gray_org = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    t, thresh = cv2.threshold(gray_org, bw_thresh, 255, cv2.THRESH_BINARY_INV)
    cv2.imshow('cropped_frame', thresh)

    if cv2.waitKey(1) == ord('q'):
        break