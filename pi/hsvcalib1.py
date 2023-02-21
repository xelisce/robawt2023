# THIS FILE IS TO BE KEPT ON PI

from MultiThread import WebcamStream
import cv2

cam_stream = WebcamStream(stream_id=0)
cam_stream.start()
frame = cam_stream.read()
cv2.imwrite("hsvcalibimage.jpg", frame)
cam_stream.stop()