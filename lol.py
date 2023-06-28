import cv2
import numpy as np
infile = cv2.imread(r"C:\Users\ASUS\OneDrive - Raffles Institution\Desktop\bot\pff.png")
frame_gray = cv2.cvtColor(infile, cv2.COLOR_BGR2GRAY)
frame_gray = cv2.inRange(frame_gray, 0, 128)
frame_gray = cv2.dilate(frame_gray, kernel=np.ones((4, 4), np.uint8))
frame_gray = cv2.bitwise_not(frame_gray)
cv2.imwrite("pff2.png", frame_gray)