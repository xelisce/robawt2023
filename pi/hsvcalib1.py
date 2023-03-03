# THIS FILE IS TO BE KEPT ON PI

from MultiThread import WebcamStream
import cv2

cam_stream = WebcamStream(stream_id=0)
cam_stream.start()


while True:
    if cam_stream.stopped == True:
        break
    else:
        frame = cam_stream.read()

    cv2.imshow('frame', frame)
    key = cv2.waitKey(1)
    if key == ord('k'): #press 'k' to take a photo 
        cv2.imwrite("hsvcalibimage.jpg", frame)
    elif key == ord('q'): #press q to quit
        break

cam_stream.stop()
cv2.destroyAllWindows()
