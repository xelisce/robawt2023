
#^ THIS FILE IS TO BE KEPT ON PI 
#^ HOW TO USE hsvcalib1.py
# Press 'k' to capture the photo shown in frame
# Capture will be saved images directory as hsvcalibimage.jpg
# Any subsequent capture will be saved as hsvcalibimage1.jpg, hsvcalibimage2.jpg, and so on
# Press 'q' to quit
# The image is meant to be used in conjunction with hsvcalib2.py

#^ DOM: can consider reducing the res but not rlly recommended?

from MultiThread import WebcamStream
import cv2
import os

cam_stream = WebcamStream(stream_id=0)
cam_stream.start()
i = 0
cam_type = 0

wd = os.getcwd()
img_path = os.path.join(wd, "images")
os.chdir(img_path)

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

    if cam_stream.stopped == True:
        break
    
    elif cam_type == 0:
        frame = cam_stream.read()
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        # frame_org = cv2.pyrDown(frame_org, dstsize=(640//2, 480//2))
        # frame_org = cv2.pyrDown(frame_org, dstsize=(640//4, 480//4))
    else:
        frame = cam_stream.read()
        # frame_org = cv2.pyrDown(frame_org, dstsize=(640//2, 480//2))
        # frame_org = cv2.pyrDown(frame_org, dstsize=(640//4, 480//4))

    cv2.imshow('frame', frame)
    key = cv2.waitKey(1)
    if key == ord('k'): #press 'k' to take a photo 
        while True: #^ check if the the file already exists
            fileName = input("Enter a name to save the file as: ")
            actualName = f"hsvcalibimage{fileName}.jpg"
            if os.path.exists(actualName):
                print(f"hsvcalibimage{fileName} already exists, please choose another name.")
            else:
                cv2.imwrite(f"../images/hsvcalibimage{fileName}.jpg", frame)
                print(f"Saved to images/hsvcalibimage{fileName}.jpg")
                break
        i += 1
    elif key == ord('q'): #press q to quit
        break

cam_stream.stop()
cv2.destroyAllWindows()
