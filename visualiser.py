import cv2
import numpy as np

a = 1

width_lt = 640//4
height_lt = 480//4

#* CROPPING

centre_x_lt = width_lt//2
crop_bot_h = 60//2
higher_crop_triangle_h = 72//2
higher_crop_triangle_w = 75//2
higher_crop_triangle_gap_w = 35//2

mask_trapeziums = np.zeros([height_lt, width_lt], dtype="uint8")
left_triangle_pts = np.array([[0, height_lt - crop_bot_h], [0, height_lt - higher_crop_triangle_h], [centre_x_lt - higher_crop_triangle_gap_w - higher_crop_triangle_w, height_lt - higher_crop_triangle_h], [centre_x_lt - higher_crop_triangle_gap_w , height_lt - crop_bot_h]])
right_triangle_pts = np.array([[width_lt, height_lt - crop_bot_h], [width_lt, height_lt - higher_crop_triangle_h], [centre_x_lt + higher_crop_triangle_gap_w + higher_crop_triangle_w, height_lt - higher_crop_triangle_h], [centre_x_lt + higher_crop_triangle_gap_w , height_lt - crop_bot_h]])
bottom_rectangle_pts = np.array([[0, height_lt], [0, height_lt - crop_bot_h], [width_lt, height_lt - crop_bot_h], [width_lt, height_lt]])
cv2.fillPoly(mask_trapeziums, [left_triangle_pts, right_triangle_pts, bottom_rectangle_pts], 255)
mask_trapeziums = cv2.bitwise_not(mask_trapeziums)

#* VECTORS

def callback(x):
    global a, mask_trapeziums
    a = cv2.getTrackbarPos('x power (divide by 100)', 'controls')

    x_com_scale = np.array([[i] * width_lt for i in np.linspace(0, 1., height_lt-higher_crop_triangle_h)]) #? removed the nasty 1 - np.array
    x_com_scale = x_com_scale ** (a/100)
    x_com_scale = np.concatenate((x_com_scale, np.array([[1] * width_lt for i in range(higher_crop_triangle_h)])))

    final_x = x_com * x_com_scale
    with_mask = cv2.bitwise_and(final_x, final_x, mask=mask_trapeziums)

    cv2.imshow("x_scale", x_com_scale)
    cv2.imshow("final_x", final_x)
    cv2.imshow("with mask", with_mask)

x_com = np.tile(np.concatenate((np.linspace(1., 0, width_lt//2),np.linspace(0, 1., width_lt//2)),axis=0), (height_lt, 1)) #reps is (outside, inside)
y_com = np.array([[i] * width_lt for i in np.linspace(1., 0, height_lt)])

x_com_scale = np.array([[i] * width_lt for i in np.linspace(0, 1., height_lt-higher_crop_triangle_h)]) #? same here
x_com_scale = x_com_scale ** (a/100)
x_com_scale = np.concatenate((x_com_scale, np.array([[1] * width_lt for i in range(higher_crop_triangle_h)])))

final_x = x_com * x_com_scale

cv2.namedWindow('controls', 2)
cv2.resizeWindow('controls', 550, 50)
cv2.createTrackbar('x power (divide by 100)', 'controls', 0, 1000, callback)

cv2.imshow("y_com", y_com)
cv2.imshow("x_com", x_com)
cv2.imshow("x_scale", x_com_scale)
cv2.imshow("final_x", final_x)
cv2.waitKey()
cv2.destroyAllWindows()

#* CROPPING

top_frame = cv2.imread("hsvcalib\hsvcalibimagetop_cam_clawup.jpg")
top_frame = cv2.pyrDown(top_frame, dstsize=(640//2, 480//2))
top_frame = cv2.pyrDown(top_frame, dstsize=(width_lt, height_lt))

def callback_crop(x):
    global crop_bot_h, higher_crop_triangle_h, higher_crop_triangle_w, higher_crop_triangle_gap_w
    crop_bot_h = cv2.getTrackbarPos('crop_bot_h', 'controls_crop')
    higher_crop_triangle_h = cv2.getTrackbarPos('higher_crop_triangle_h', 'controls_crop')
    higher_crop_triangle_w = cv2.getTrackbarPos('higher_crop_triangle_w', 'controls_crop')
    higher_crop_triangle_gap_w = cv2.getTrackbarPos('higher_crop_triangle_gap_w', 'controls_crop')

    mask_trapeziums = np.zeros([height_lt, width_lt], dtype="uint8")
    left_triangle_pts = np.array([[0, height_lt - crop_bot_h], [0, height_lt - higher_crop_triangle_h], [centre_x_lt - higher_crop_triangle_gap_w - higher_crop_triangle_w, height_lt - higher_crop_triangle_h], [centre_x_lt - higher_crop_triangle_gap_w , height_lt - crop_bot_h]])
    right_triangle_pts = np.array([[width_lt, height_lt - crop_bot_h], [width_lt, height_lt - higher_crop_triangle_h], [centre_x_lt + higher_crop_triangle_gap_w + higher_crop_triangle_w, height_lt - higher_crop_triangle_h], [centre_x_lt + higher_crop_triangle_gap_w , height_lt - crop_bot_h]])
    bottom_rectangle_pts = np.array([[0, height_lt], [0, height_lt - crop_bot_h], [width_lt, height_lt - crop_bot_h], [width_lt, height_lt]])
    cv2.fillPoly(mask_trapeziums, [left_triangle_pts, right_triangle_pts, bottom_rectangle_pts], 255)
    mask_trapeziums = cv2.bitwise_not(mask_trapeziums)
    cv2.imshow("crop mask", mask_trapeziums)

    cropped_image = cv2.bitwise_and(top_frame, top_frame, mask=mask_trapeziums)
    cv2.imshow("image", cropped_image)

cv2.namedWindow('controls_crop', 2)
cv2.resizeWindow('controls_crop', 550, 50)
cv2.createTrackbar('crop_bot_h', 'controls_crop', crop_bot_h, height_lt, callback_crop)
cv2.createTrackbar('higher_crop_triangle_h', 'controls_crop', higher_crop_triangle_h, height_lt, callback_crop)
cv2.createTrackbar('higher_crop_triangle_w', 'controls_crop', higher_crop_triangle_w, width_lt, callback_crop)
cv2.createTrackbar('higher_crop_triangle_gap_w', 'controls_crop', higher_crop_triangle_gap_w, width_lt, callback_crop)

cv2.imshow("crop mask", mask_trapeziums)
cv2.imshow("image", top_frame)
cv2.waitKey()
cv2.destroyAllWindows()

print("RESULTS")
print("-"*30)
print("power", a/100)
print("-"*30)
print("crop_bot_h =", crop_bot_h)
print("higher_crop_triangle_h =", higher_crop_triangle_h)
print("higher_crop_triangle_w =", higher_crop_triangle_w)
print("higher_crop_triangle_gap_w =", higher_crop_triangle_gap_w)
print("-"*30)

#* LOG
# RESULTS
# ------------------------------
# power 0.01
# ------------------------------
# crop_bot_h 15
# higher_crop_triangle_h 23
# higher_crop_triangle_w 26
# higher_crop_triangle_gap_w 24
# ------------------------------