import cv2
import numpy as np

#* VECTORS

a = 1

def callback(x):
    global a
    a = cv2.getTrackbarPos('x power (divide by 100)', 'controls')

    x_com_scale = 1 - np.array([[i] * width_lt for i in np.linspace(1., 0, height_lt-higher_crop_triangle_h)])
    x_com_scale = x_com_scale ** (a/100)
    x_com_scale = np.concatenate((x_com_scale, np.array([[1] * width_lt for i in range(higher_crop_triangle_h)])))

    final_x = x_com * x_com_scale

    cv2.imshow("x_scale", x_com_scale)
    cv2.imshow("final_x", final_x)

width_lt = 310
height_lt = 240
higher_crop_triangle_h = 72

x_com = np.tile(np.concatenate((np.linspace(1., 0, width_lt//2),np.linspace(0, 1., width_lt//2)),axis=0), (height_lt, 1)) #reps is (outside, inside)
y_com = np.array([[i] * width_lt for i in np.linspace(1., 0, height_lt)])

x_com_scale = 1 - np.array([[i] * width_lt for i in np.linspace(1., 0, height_lt-higher_crop_triangle_h)])
x_com_scale = x_com_scale ** (a/100)
x_com_scale = np.concatenate((x_com_scale, np.array([[1] * width_lt for i in range(higher_crop_triangle_h)])))

final_x = x_com * x_com_scale

cv2.namedWindow('controls', 2)
cv2.resizeWindow('controls', 550, 10)
cv2.createTrackbar('x power (divide by 100)', 'controls', 0, 1000, callback)

cv2.imshow("y_com", y_com)
cv2.imshow("x_com", x_com)
cv2.imshow("x_scale", x_com_scale)
cv2.imshow("final_x", final_x)
cv2.waitKey()
cv2.destroyAllWindows()

#* CROPPING

centre_x_lt = width_lt//2

mask_trapeziums = np.zeros([height_lt, width_lt], dtype="uint8")

crop_bot_h = 60
higher_crop_triangle_h = 72
higher_crop_triangle_w = 75
higher_crop_triangle_gap_w = 35

left_triangle_pts = np.array([[0, height_lt - crop_bot_h], [0, height_lt - higher_crop_triangle_h], [centre_x_lt - higher_crop_triangle_gap_w - higher_crop_triangle_w, height_lt - higher_crop_triangle_h], [centre_x_lt - higher_crop_triangle_gap_w , height_lt - crop_bot_h]])
right_triangle_pts = np.array([[width_lt, height_lt - crop_bot_h], [width_lt, height_lt - higher_crop_triangle_h], [centre_x_lt + higher_crop_triangle_gap_w + higher_crop_triangle_w, height_lt - higher_crop_triangle_h], [centre_x_lt + higher_crop_triangle_gap_w , height_lt - crop_bot_h]])
bottom_rectangle_pts = np.array([[0, height_lt], [0, height_lt - crop_bot_h], [width_lt, height_lt - crop_bot_h], [width_lt, height_lt]])
cv2.fillPoly(mask_trapeziums, [left_triangle_pts, right_triangle_pts, bottom_rectangle_pts], 255)
mask_trapeziums = cv2.bitwise_not(mask_trapeziums)

cv2.imshow("crop mask", mask_trapeziums)
cv2.waitKey()
cv2.destroyAllWindows()