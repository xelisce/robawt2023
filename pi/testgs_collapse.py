import cv2
from MultiThread import WebcamStream
import numpy as np
import enum
# import time

height = 480

# g_roi_lh = 400 #! increase lower height boundary in main code
# # g_roi_uh = height
# g_roi_b_lh = 340 # black above green upper boundary
# # g_roi_b_uh = g_roi_lh #black above green lower boundary
# gs_b_sampleoffset = 10 #offset ample to above by this amount of pixels
# gs_b_sampleh = 50 #this is the height of the black sample taken above green square
# gs_minblackpct = 0.35
# green_minarea = 1000000

u_black = 70

l_green = np.array([30, 90, 60], np.uint8)
u_green = np.array([85, 255, 255], np.uint8)


class Task(enum.Enum):
    EMPTY = 0
    LEFT_GREEN = 1
    RIGHT_GREEN = 2
    DOUBLE_GREEN = 3

curr = Task.EMPTY

# #* The following should be inside the loop

# frame_org = cv2.imread("hsvcalibimage.jpg")
# # frame_org = frame_org[crop_h:height, :]
# frame_bw = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
# frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)
# # cv2.imshow("frame_hsv", frame_hsv)

# #~ Presence of green square?
# green_mask = cv2.inRange(frame_hsv, l_green, u_green)
# frame_black = cv2.inRange(frame_bw, 0, u_black) - green_mask
# green_mask = green_mask[g_roi_lh:, :]
# green_sum = np.sum(green_mask)
# cv2.imshow("green mask", green_mask)


# if green_sum > green_minarea:

#     #~ Find x position of green
#     green_col = np.amax(green_mask, axis=0)
#     g_indices_h = np.where(green_col == 255)
#     gs_left = g_indices_h[0][0]
#     gs_right = g_indices_h[0][-1]
#     gs_centre = (gs_left + gs_right)/2

#     #~ Test if below or above line (SS)
#     green_row = np.amax(green_mask, axis=1)
#     g_indices_v = np.where(green_row == 255)
#     gs_top = g_indices_v[0][0]+g_roi_lh
#     # g_bot = g_indices_v[0][-1]
#     #^ another solution is skip these steps and input a fixed area
#     blackarea_abovegs = frame_black[gs_top-gs_b_sampleh-gs_b_sampleoffset:gs_top-gs_b_sampleoffset, gs_left:gs_right]
#     gs_blackpct = np.sum(blackarea_abovegs) / 255 / gs_b_sampleh / (gs_right-gs_left)
#     print("percatnage of black:", gs_blackpct)
#     cv2.imshow("black area above green square", blackarea_abovegs)

#     #~ GREEN SQUARE FOUND
#     if gs_blackpct > gs_minblackpct:

#         #~ Find x position of black
#         # black_leftofgs = frame_black[g_roi_lh:, :gs_left]
#         # black_rightofgs = frame_black[g_roi_lh:, gs_right:]
#         # black_leftsum = np.sum(black_leftofgs)
#         # black_rightsum = np.sum(black_rightofgs)

#         #~ Find x position of black (moments method)
#         black_besidegs = frame_black[g_roi_b_lh:, :]
#         cv2.imshow("black beside gs", black_besidegs)
#         blackM = cv2.moments(black_besidegs)
#         cx_black = int(blackM["m10"]/blackM["m00"]) if np.sum(black_besidegs) else 0 #theoretically divide by zero error should never happen
#         print(cx_black, gs_centre)

#         # print(black_leftsum, black_rightsum)

#         # #~ Identify type of green square
#         # if black_leftsum > 3000000 and black_rightsum > 3000000:
#         #     curr = Task.DOUBLE_GREEN
#         # elif black_leftsum > 3000000:
#         #     curr = Task.RIGHT_GREEN
#         # elif black_rightsum > 3000000:
#         #     curr = Task.LEFT_GREEN

#         #     #! NOT USED, MOMENTS LOGIC IS BEING USED



#         # #~ Identify type of green square (moments method)
#         if cx_black > gs_left and cx_black < gs_right and green_sum > 2*green_minarea:
#             curr = Task.DOUBLE_GREEN
#         elif cx_black > gs_centre:
#             curr = Task.LEFT_GREEN
#         elif cx_black < gs_centre:
#             curr = Task.RIGHT_GREEN
#         else:
#             print("hms")
        
#         cv2.imshow("green", frame_org[:, gs_left:gs_right])
#         # cv2.imshow("green", frame_hsv[g_top:g_bot, g_left:g_right])

# cv2.waitKey()

# print("task", curr)







#* TAKEN FROM PI LINETRACK

gs_roi_h = 400 #the crop height for gs #! increase after tuning 
gs_bksampleoffset = 10 #offset sample above green squares
gs_bksampleh = 50
gs_minbkpct = 0.35 #! to be tuned
gs_minarea = 1000000 #? consider making this scaled by pixels (/255)

frame_org = cv2.imread("hsvcalibimage.jpg")
frame_gray = cv2.cvtColor(frame_org, cv2.COLOR_BGR2GRAY)
frame_hsv = cv2.cvtColor(frame_org, cv2.COLOR_BGR2HSV)

mask_green = cv2.inRange(frame_hsv, l_green, u_green)
mask_gs = mask_green[gs_roi_h:, :]
gs_sum = np.sum(mask_gs)
# print(gs_sum) #& debug green min area

mask_black = cv2.inRange(frame_gray, 0, u_black) - mask_green
# cv2.imshow('black frame', black_mask) #& debug black mask

if gs_sum > gs_minarea:

    #~ Find x positions of green
    green_col = np.amax(mask_gs, axis=0)
    g_indices_h = np.where(green_col==255) #h for horiontal
    gs_left = g_indices_h[0][0]
    gs_right = g_indices_h[0][-1]
    # cv2.imshow("Green bounding box", frame_org[:, gs_left:gs_right]) #& debug green bounds

    #~ Find y position of green (Can cut processing time here by putting fixed constant instead)
    green_row = np.amax(mask_gs, axis=1)
    g_indices_v = np.where(green_row==255) #v for vertical
    gs_top = g_indices_v[0][0] + gs_roi_h

    #~ Test if below or above line
    gs_bkabove = mask_black[gs_top - gs_bksampleoffset - gs_bksampleh : gs_top - gs_bksampleoffset, gs_left : gs_right]
    gs_bkpct = np.sum(gs_bkabove) / 255 / gs_bksampleh / (gs_right - gs_left)
    print("Percentage of black above green:", gs_bkpct) #& debug green's black
    cv2.imshow("black area above green square", gs_bkabove) 
    cv2.imshow("black area above green in orig frame", frame_org[gs_top - gs_bksampleoffset - gs_bksampleh : gs_top - gs_bksampleoffset, gs_left : gs_right])
    
    #~ GREEM SQUARE FOUND
    if gs_bkpct > gs_minbkpct:

        #~ Find x position of green
        gs_centre = (gs_left + gs_right) / 2

        #~ Find x position of black
        gs_bkbeside = mask_black[gs_roi_h:, :]
        blackM = cv2.moments(gs_bkbeside)
        cx_black = int(blackM["m10"]/blackM["m00"]) if np.sum(gs_bkbeside) else 0 #theoretically divide by zero error should never happen
        # cv2.imshow("Black beside green squares", gs_bkbeside) #& debug green type triggered
        print("Black x-centre:", cx_black)
        print("Green x-centre:", gs_centre)

        #~ Identify type of green square
        if cx_black > gs_left and cx_black < gs_right and gs_sum > 2 * gs_minarea:
            curr = Task.DOUBLE_GREEN
        elif cx_black > gs_centre:
            curr = Task.LEFT_GREEN
        elif cx_black < gs_centre:
            curr = Task.RIGHT_GREEN

print(curr)

cv2.waitKey()