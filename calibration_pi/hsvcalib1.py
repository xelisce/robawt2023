
#^ THIS FILE IS TO BE KEPT ON PI
#^ HOW TO USE hsvcalib1.py
# Press 'k' to capture the photo shown in frame
# Capture will be saved images directory as hsvcalibimage.jpg
# Any subsequent capture will be saved as hsvcalibimage1.jpg, hsvcalibimage2.jpg, and so on
# Press 'q' to quit
# The image is meant to be used in conjunction with hsvcalib2.py

from MultiThread import WebcamStream
import cv2

cam_stream = WebcamStream(stream_id=0)
cam_stream.start()
i = 0


while True:
    if cam_stream.stopped == True:
        break
    else:
        frame = cam_stream.read()
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)

    cv2.imshow('frame', frame)
    key = cv2.waitKey(1)
    if key == ord('k'): #press 'k' to take a photo 
        cv2.imwrite(f"images/hsvcalibimage{i}.jpg", frame)
        print(f"saved to images/hsvcalibimage{i}.jpg")
        i += 1
    elif key == ord('q'): #press q to quit
        break

cam_stream.stop()
cv2.destroyAllWindows()
