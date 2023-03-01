import cv2
from MultiThread import WebcamStream

evac_stream = WebcamStream(stream_id=0)
evac_stream.start()

crop_h = 183

u_black = 55

