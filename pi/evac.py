import cv2
from MultiThread import WebcamStream
import numpy as np

# evac_stream = WebcamStream(stream_id=0)
# evac_stream.start()
# evac_org = evac_stream.read()
evac_org = cv2.imread("WIN_20230301_16_17_24_Pro.jpg")
width, height_org = evac_org.shape[1], evac_org.shape[0]

crop_h = 183

height = height_org - crop_h
u_black = 55

#* IN LOOP

evac_org = cv2.imread("WIN_20230301_16_17_24_Pro.jpg")
print("og shape:", evac_org.shape[1], evac_org.shape[0])
evac_hsv = cv2.cvtColor(evac_org, cv2.COLOR_BGR2HSV)

evac_gray = cv2.cvtColor(evac_org, cv2.COLOR_BGR2GRAY)
evac_max = np.amax(evac_org, axis=2)

print(evac_hsv)
evac_hsv = evac_hsv[:, :, 1:2]

cv2.imshow("sat", evac_hsv)





# print("shape:", evac_gray.shape[1], evac_gray.shape[0])
# # print(evac_gray)

# evac_gray = evac_gray[crop_h:, :]
# circles = cv2.HoughCircles(evac_gray, cv2.HOUGH_GRADIENT, 1.2, 70, param1 = 200 , param2 =27, minRadius= 40, maxRadius=100)
# balls = []
# if circles is not None:
#     for x, y, r in circles[0]:
#         mask = np.zeros(evac_org.shape[:2], dtype=np.uint8)
#         mask = cv2.circle(mask, (int(x),int(y)), int(r), 255, -1)
#         balls.append({
#                 "x": x,
#                 "y": y,
#                 "r": r,
#             })
        
#     # print(balls)

#     circles = np.uint16(np.around(circles))
#     for i in circles[0,:]:
#         # draw the outer circle
#         cv2.circle(evac_gray,(i[0],i[1]),i[2],(0,255,0),2)
#         # draw the center of the circle
#         cv2.circle(evac_gray,(i [0],i[1]),2,(0,0,255),3)

# evac_max = evac_max[crop_h:, :]
# circles = cv2.HoughCircles(evac_max, cv2.HOUGH_GRADIENT, 1.2, 70, param1 = 200 , param2 =27, minRadius= 40, maxRadius=100)
# balls = []
# if circles is not None:
#     for x, y, r in circles[0]:
#         mask_max = np.zeros(evac_max.shape[:2], dtype=np.uint8)
#         mask_max = cv2.circle(mask_max, (int(x),int(y)), int(r), 255, -1)
#         balls.append({
#                 "x": x,
#                 "y": y,
#                 "r": r,
#             })
        
#     # print(balls)

#     circles = np.uint16(np.around(circles))
#     for i in circles[0,:]:
#         # draw the outer circle
#         cv2.circle(evac_max,(i[0],i[1]),i[2],(0,255,0),2)
#         # draw the center of the circle
#         cv2.circle(evac_max,(i [0],i[1]),2,(0,0,255),3)
        
# # cv2.imshow('detected circles',evac_gray)

# combined = np.hstack((evac_gray, evac_max))
# combinedDown = cv2.pyrDown(combined)
# cv2.imshow("both", combinedDown)

cv2.waitKey()
cv2.destroyAllWindows()