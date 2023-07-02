import cv2
from MultiThread import WebcamStream

cam_stream = WebcamStream(stream_id=0)
cam_stream.start()
bw_thresh = 0

def update(val):
    global bw_thresh
    bw_thresh = val

frame_org = cam_stream.read()
frame_org = cv2.flip(frame_org, 0)
frame_org = cv2.flip(frame_org, 1)
gray_org = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
t, thresh = cv2.threshold(gray_org, bw_thresh, 255, cv2.THRESH_BINARY_INV)
cv2.imshow('frame', thresh)
cv2.createTrackbar('threshold', 'frame', 0, 255, update)

while True:
    if cam_stream.stopped:
        break

    frame_org = cam_stream.read()
    gray_org = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
    t, thresh = cv2.threshold(gray_org, bw_thresh, 255, cv2.THRESH_BINARY_INV)
    cv2.imshow('frame', thresh)

    if cv2.waitKey(1) == ord('q'):
        break

