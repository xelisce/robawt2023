import cv2
import numpy as np


# image = cv2.imread('hsvcalib/bot/greennred')

from MultiThread import WebcamStream

cam_stream = WebcamStream(stream_id=4)
cam_stream.start()

l_greenevac = np.array([70, 35, 20], np.uint8)
u_greenevac = np.array([95, 255, 255], np.uint8)

l_red1evac = np.array([0, 60, 20], np.uint8)
u_red1evac = np.array([15, 255, 255], np.uint8)
l_red2evac = np.array([170, 60, 20], np.uint8) 
u_red2evac = np.array([180, 255, 255], np.uint8)




while True:

    frame = cam_stream.read()
    frame = cv2.pyrDown(frame, dstsize=(640//2, 480//2))
    frame = cv2.pyrDown(frame, dstsize=(640//4, 480//4))
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    cv2.imshow('frame', frame)

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(frame_hsv, l_greenevac, u_greenevac)
    mask_red1 = cv2.inRange(frame_hsv, l_red1evac, u_red1evac)
    mask_red2 = cv2.inRange(frame_hsv, l_red2evac, u_red2evac)
    mask_red = mask_red1 + mask_red2

    green_sum = np.sum(mask_green) / 255
    red_sum = np.sum(mask_red) / 255


    key = cv2.waitKey(1)
    if key == ord('q'): #press q to quit
        break

cam_stream.stop()
cv2.destroyAllWindows()
