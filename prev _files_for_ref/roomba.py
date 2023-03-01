import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream
from servo import Claw, Servo

class GreenSquare(Enum):
	RIGHT = 1
	LEFT = 2
	DOUBLE = 3

ser = serial.Serial('/dev/serial/by-id/usb-DFRobot_www.dfrobot.com__0042_75932313838351201101-if00', 9600, timeout=1, write_timeout=5)
dim = {"w": 320, "h": 240}
boundaries = {
	"red": ((0,0,0), (0,0,0)),
	"green": ((50,50,50), (80,255,255)),
	"blue": ((94, 50, 50), (125, 255, 255))
}
crop_h = 60
b_crop_h = 10
gs_crop_h = 80
b_gs_crop_h = 0

b_w_thresh = 40 #
k_p = 1

#* the region of image to consider whether robot is "done" with the particular green square
gs_lt_roi = 0

rk_crop_h = 30
b_rk_crop_h = 0

obs_crop_h = 60 # height of image also affected by "crop_h" and "b_crop_h"
line_min = 2000

epsilon = 30 * math.pi/180
delta = 10 * math.pi/180

d_green_square_min = 1000000 #TODO: Determine Actual Min, this is just a guess (I was very incorrect)
s_green_square_min = 5000

right_turn_rotation = 0.72
min_area = 80

st_crop_h = 80
st_b_crop_h = 10

min_red = 50000

# s_tape_minmax = {"w": (120, 160), "h": (0, 30)}
# time.sleep(3)
claw = Claw(ser)
cam = VideoStream(resolution=(dim["h"], dim["w"])).start()



def see_entry(gray, thresh, green, min_std_dev, min_mask_sum, t_crop, b_crop):
	"""Ensure that max(thresh) = 255!"""
	not_green = cv.bitwise_not(green)
	not_black = cv.bitwise_not(thresh)
	st_mask = cv.bitwise_and(not_black, not_green)

	st_mask_c = st_mask[t_crop:(st_mask.shape[0] - b_crop), :]
	gray_c = gray[t_crop:(gray.shape[0] - b_crop), :]
	_, std = cv.meanStdDev(gray_org ,mask = st_mask)
	print(std, np.sum(st_mask))
	return (std[0][0] > min_std_dev and np.sum(st_mask) > min_mask_sum)

def see_exit(green, min_green, t_crop, b_crop):
	green_c = green[t_crop:(green.shape[0] - b_crop), :]
	# print(np.sum(green_c))
	return (np.sum(green_c)) >= min_green*255

def claw_deposit():
	claw.write_claw(Servo.ARM, 90).close(95).delay(0.5)
	claw.tilt_compartment()
	claw.open().delay(0.5)
	claw.raisin().delay(0.5)
	claw.reset_compartment()
	time.sleep(1)
	claw.lower().open()
	ser.write(b's' + struct.pack('f', 0) + b'\n')
	ser.write(b'r' + struct.pack('f', 0) + b'\n')

def leave_evac():
	while True:
		frame_org = cam.read()
		# frame_org = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
		frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
		green = cv.inRange(frame_org_hsv, boundaries["green"][0], boundaries["green"][1])
		gt = see_exit(green, 120, 90, 10)

		if gt:
			break
	ser.write(b'e' + (1).to_bytes(1, "big") + b'\n')

def deposit(seconds):
	start_time = time.time()
	while True:
		frame_org = cam.read()
		# cv.imwrite("org.png", frame_org)
		# frame_org = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
		frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
		gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
		t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)
		
		frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
		green = cv.inRange(frame_org_hsv, boundaries["green"][0], boundaries["green"][1])

		st = see_entry(gray_org, thresh, green, 40, 13000000, 180, 20)
		gt = see_exit(green, 120, 90, 10)
		if st:
			ser.write(b't' + (1).to_bytes(1, "big") + b'\n')
			print("hi1")
		if gt:
			ser.write(b'g' + (1).to_bytes(1, "big") + b'\n')
			print("hi2")

		if (s := np.sum(thresh)) > 300000:
			print(s)
			ser.write(b'd' + (1).to_bytes(1, "big") + b'\n')
			contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
			big_c = max(contours, key = cv.contourArea)
			M = cv.moments(big_c)
			cX = int(M["m10"] / M["m00"])
			ser.write(b'r' + struct.pack('f', (0.3 if cX > dim["w"]/2 else -0.3)) + b'\n')
		else:
			ser.write(b'd' + (0).to_bytes(1, "big") + b'\n')

		if (time.time() - start_time) >= seconds:
			ser.write(b'd' + (0).to_bytes(1, "big") + b'\n')
			break

def green_square(lines, g_frame):
	# global l
	if lines is None:
		return
	f_lines = list(filter(h_line_filter, lines))
	width = g_frame.shape[1]
	# print(True if len(f_lines) > 0 else False)
	if len(f_lines) > 0:
		y_intercepts = [int(y_value(line, 0)) for line in f_lines]
		width_intercepts = [int(y_value(line, width)) for line in f_lines]

		mask = np.ones(g_frame.shape[:2], dtype=np.uint8)
		useless_pixels = np.array([
			[0, 0], 
			[width, 0], 
			[width, max(width_intercepts)],
			[0, max(y_intercepts)]
		])
		
		cv.fillPoly(mask, pts =[useless_pixels], color=0)
		# print(mask.shape)
		# print(frame.shape)
		roi = cv.bitwise_and(g_frame, g_frame, mask=mask)	
		# cv.imwrite("out.png", roi)
		# roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
		# green = cv.inRange(roi, boundaries["green"][0], boundaries["green"][1])
		contours,hierarchy = cv.findContours(roi, 1, 2)
		
		green_vector = d_vector(roi, h_scaled_v=gs_scaled_v)
		# print(green_vector)
		# cv.imwrite("out.png", green)
		g_sum = np.sum(roi)
		# print(g_sum)
		# print("hello")
		# if g_sum > s_green_square_min:
		# print(len(contours))
		f_contours = list(filter(lambda cnt: cv.contourArea(cnt) > min_area, contours))
		# print(len(contours)
		if len(f_contours) >= 1:
			# Cropping isn't perfect and some green still remains on the screen
			# this is to prevent the "specks" of green to count as a green square

			# cv.imwrite(f"out{l}.png", green)
			# print(f"dv{l} =", green_vector)
			# l += 1
			if green_vector[0] < 0:
				return GreenSquare.LEFT
			elif green_vector[0] > 0:
				return GreenSquare.RIGHT

def see_stop(red, t_crop, b_crop):
	red_c = red[t_crop:(red.shape[0] - b_crop), :]
	# lower mask (0-10)
	lower_red = np.array([0,50,50])
	upper_red = np.array([10,255,255])
	mask0 = cv.inRange(red_c, lower_red, upper_red)

	# upper mask (170-180)
	lower_red = np.array([170,50,50])
	upper_red = np.array([180,255,255])
	mask1 = cv.inRange(red_c, lower_red, upper_red)

	# join my masks
	mask = mask0+mask1
	# print(np.sum(mask))
	return (np.sum(mask)) >= min_red

def scaled_2d_matrix(height, width, degree1, degree2, degree3):
	zs = np.zeros((height, width), dtype = float)
	max_v_len = math.sqrt((height ** 2 + (width/2) ** 2))
	with np.nditer(zs, op_flags=['readwrite'], flags=['multi_index']) as it:
		for a in it:
			j, i = it.multi_index
			i -= width/2
			j = height - j
			v_len = math.sqrt((j ** 2 + i ** 2))
			len_scale = (1 - (v_len/max_v_len)) ** degree1
			y_scale = (1 - (j/height)) ** degree2
			x_scale = (2 * abs(i)/width) ** degree3

			a[...] = len_scale * y_scale * x_scale
			# a[...] = (1 - (j/height)) ** degree
	return zs/np.max(zs)

def scaled_vector(image_h, degree):
	xs = np.array([])
	# shape => (height, width)
	# print(dim["h"] - crop_h)
	for i in range(0, image_h):
		xs = np.append(xs, ((i/image_h) ** degree))
	return xs[:, np.newaxis]

def w_scaled_vector(image_w, degree):
	xs = np.array([])
	for i in range(-int(image_w/2), int(image_w/2)):
		xs = np.append(xs, (2 * i / image_w) ** degree)
	pass

scaled_v = scaled_vector(dim["h"] - b_crop_h - crop_h, 2)
w_scaled_v = w_scaled_vector(dim["w"], 10)
gs_scaled_v = scaled_vector(dim["h"] - b_gs_crop_h - gs_crop_h, 0)
scaled_m = scaled_2d_matrix(dim["h"] - crop_h - b_crop_h, dim["w"], 2.5, 0.4, 0.2)
cv.imwrite("out.png", scaled_m * 255)

def d_vector(masked_img, h_scaled_v = None, w_scaled_v = None, scaled_m = None):
		masked_img = masked_img.astype(float)
		if scaled_m is None:
			if h_scaled_v is None:
				raise ValueError()
			masked_img *= h_scaled_v
			if w_scaled_v:
				masked_img *= w_scaled_v
		else: 
			masked_img *= scaled_m

		xs = np.sum(masked_img, axis = 0)
		ys = np.sum(masked_img, axis = 1)
		ys = ys[::-1]
		x, y = 0, 0
		for index, i in enumerate(xs):
				index -= len(xs)/2
				x += index*i
				# print(index)
		for index, j in enumerate(ys):
				y += index*j
		return (x, y)

def h_line_filter(line):
	rho, theta = line[0]
	if theta <= math.pi/2 + epsilon and theta >= math.pi/2 - epsilon:
		return True
	else:
		return False

def y_value(line, x_value):
	rho, theta = line[0]
	return (rho - x_value*math.cos(theta))/(math.sin(theta))

def x_value(line, y_value):
	rho, theta = line[0]
	return (rho - y_value*math.sin(theta))/(math.cos(theta))

ser.write(b's' + struct.pack('f', 1.0) + b'\n')
curr = None
start_time = 0
start1_time = time.time()
journey = []
retreat = False
prev_see_line = False
while True:
	# continue
	frame_org = cam.read()
	gs_frame_org = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	green_org = cv.inRange(gs_frame_org, boundaries["green"][0], boundaries["green"][1])
	# print(ret)
	# frame1 = frame[:, crop_h:(frame.shape[0])]
	# frame_org = cv.rotate(frame_org, cv.ROTATE_90_CLOCKWISE)
	frame = frame_org[crop_h:(frame_org.shape[0] - b_crop_h), :]
	gs_frame = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	green = green_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	blue_frame = frame_org[rk_crop_h:(frame_org.shape[0] - b_rk_crop_h), :]
	# print("gjiorjgi0owjri0o")

	frame_org_gray = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	edges = cv.Canny(frame_org_gray,100, 200)

	gs_frame_edges = edges[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	rotation = 0

	# print(curr)
	if curr:
		green = green[gs_lt_roi:(green.shape[0]) ,:]
		g_sum = np.sum(green)
		if curr == GreenSquare.DOUBLE:
			if (time.time() - start_time) < 2.5:
				rotation = 1
			else:
				curr = None
		else:
			if g_sum > 0:
				ser.write(b's' + struct.pack('f', 0.7) + b'\n')
				if curr == GreenSquare.RIGHT:
					rotation = right_turn_rotation
				elif curr == GreenSquare.LEFT:
					rotation = -right_turn_rotation
			else: 
				ser.write(b's' + struct.pack('f', 1.0) + b'\n')
				# ser.write(b's' + struct.pack('f', 1.0) + b'\n')
				curr = None
	else:
		lines = cv.HoughLines(gs_frame_edges,1,np.pi/180,60)
		curr = green_square(lines, green)
		start_time = time.time()

		gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
		t, thresh = cv.threshold(gray, b_w_thresh, 1 ,cv.THRESH_BINARY_INV)
		
		if see_stop(gs_frame_org, 90, 10):
			ser.write(b's' + struct.pack('f', 0) + b'\n')
		else: 
			ser.write(b's' + struct.pack('f', 1) + b'\n')

		dv = list(d_vector(thresh, h_scaled_v = scaled_v))#, w_scaled_v = w_scaled_v)
		
		deviation = 0
		if dv[1] != 0:
			deviation = math.atan(dv[0]/dv[1])

		deviation *= k_p 
		# deviation *= white_scale #** 10
		# print(deviation)
		deviation = 1 if deviation >= 1 else deviation
		deviation = -1 if deviation <= -1 else deviation
		# print(deviation)
		rotation = deviation
		# ser.write(b's' + struct.pack('f', 1) + b'\n')

		
		# print(b'l' + see_line + b'\n')

	# # blue_frame = cv.GaussianBlur(blue_frame, (5, 5), 0)

	# hsv = cv.cvtColor(blue_frame, cv.COLOR_BGR2HSV)
	# maskBlue = cv.inRange(hsv, boundaries["blue"][0], boundaries["blue"][1])
	# # maskBlue = cv.erode(maskBlue, None, iterations=2)
	# # maskBlue = cv.dilate(maskBlue, None, iterations=2)

	# if np.sum(maskBlue) >= 2000000:
	# 	ser.write(b's' + struct.pack('f', 0) + b'\n')
	# 	time.sleep(2)
	# 	print(journey)
	# 	retreat = True
	# elif np.sum(maskBlue) > 10000:
	# 	if not retreat:
	# 		print("blue")
	# 		Moments = cv.moments(maskBlue)
	# 		xBlue = int(Moments["m10"] / Moments["m00"])
	# 		yBlue = int(Moments["m01"] / Moments["m00"])
	# 		finalx = xBlue - frame.shape[1]/2
	# 		finaly = frame.shape[0] - yBlue
	# 		if xBlue!=0:
	# 			deviation = math.atan(finalx/finaly)
	# 			#sus?
	# 			deviation *= k_p
	# 			deviation = deviation if deviation <= 1 else 1
	# 			deviation = deviation if deviation >= -1 else -1
	# 			rotation = deviation
	# 			journey.append(deviation)

	
	# if retreat:
	# 	if len(journey)>0:
	# 		ser.write(b's' + struct.pack('f', -1) + b'\n')
	# 		print(len(journey))
	# 		rotation = journey[len(journey) - 1]
	# 		journey.pop()
	# 	else:
	# 		ser.write(b's' + struct.pack('f', 1) + b'\n')
	# 		retreat = False

	# print("r", rotation)

	ser.write(b'r' + struct.pack('f', rotation) + b'\n')
	ser.flush()

	if (time.time() - start1_time) > 15:
		# cam.kill()
		# print("broke")
		break

ser.write(b's' + struct.pack('f', 1) + b'\n')
time.sleep(5)
ser.write(b's' + struct.pack('f', 0) + b'\n')

ser.write(b't' + (1).to_bytes(1, "big") + b'\n')
claw.lower().open()
evac_time_start = time.time()
while True:
	# continue
	frame_org = cam.read()
	# frame_org = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	green = cv.inRange(frame_org_hsv, boundaries["green"][0], boundaries["green"][1])
	gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)
	st = see_entry(gray_org, thresh, green, 43, 14000000, 180, 20)
	gt = see_exit(green, 120, 90, 10)
	if st:
		ser.write(b't' + (1).to_bytes(1, "big") + b'\n')
		print("hi1")
	if gt:
		ser.write(b'g' + (1).to_bytes(1, "big") + b'\n')
		print("hi2")
	circles = cv.HoughCircles(gray_org, cv.HOUGH_GRADIENT, 1.2, 70, param1 = 200 , param2 =27)
	balls = []
	if circles is not None:
		for x, y, r in circles[0]:
			if y <= (dim["h"]/2):
				mask = np.zeros(frame_org.shape[:2], dtype=np.uint8)
				# print(type(x), type(y))
				mask = cv.circle(mask, (int(x),int(y)), int(r), 255, -1)
				# cv.imwrite("out.png", mask)
				mean, std = cv.meanStdDev(frame_org, mask=mask)
				balls.append({
					"x": x,
					"y": y,
					"r": r,
					"dev": np.sum(std)
				})
	t, thresh_b = cv.threshold(gray_org, 120, 255 ,cv.THRESH_BINARY_INV)
	majik = 30
	thresh_b = thresh_b[0:(thresh_b.shape[0]-90), majik:(thresh_b.shape[1]-majik)]
	# cv.imwrite("pain.png",thresh_b)
	if len(balls) > 0:
		biggest_ball = max(balls, key=lambda b: b['r'])
		print(biggest_ball['x'] - dim["w"]/2, biggest_ball['r'])
		ser.write(b'r' + struct.pack('f', (-0.30 if (biggest_ball['x'] - dim["w"]/2) < 0 else 0.30)) + b'\n')
	else:
		ser.write(b'r' + struct.pack('f', 0) + b'\n')
	# print(np.sum(thresh_b))
	if np.sum(thresh_b) > 5000000:
		ser.write(b's' + struct.pack('f', 0.7) + b'\n')
		ser.write(b'p' + (1).to_bytes(1, "big") + b'\n')
		claw.close(95).delay(0.5)
		ser.write(b's' + struct.pack('f', 0) + b'\n')
		if np.sum(thresh) > 3000000:
			claw.dead()
		else:
			claw.alive()
		claw.lower().open()
		ser.write(b'p' + (0).to_bytes(1, "big") + b'\n')
		ser.write(b's' + struct.pack('f', 0.7) + b'\n')

	if (time.time() - evac_time_start) > 60:
		break
claw.raisin()
deposit(45)
claw_deposit()
ser.write(b'd' + (0).to_bytes(1, "big") + b'\n')
ser.write(b'd' + (0).to_bytes(1, "big") + b'\n')
leave_evac()
	# print(st, gt)
	# if gt:
	# 	break
	# print(np.max(thresh))
	# cv.imwrite("out.png", cv.bitwise_and(cv.bitwise_and(cv.bitwise_not(thresh), not_green), gray_org))
	# cv.imwrite("out.png", thresh)
	# cv.imwrite("org.png", st_mask)
	# blur = cv.GaussianBlur(gray_org,(5,5),0)
	# circles = cv.HoughCircles(gray_org, cv.HOUGH_GRADIENT, 1.2, 70, param1 = 200 , param2 = 20)
	# plt.imshow(gray_org, cmap='gray')
	# edges = cv.Canny(blur,100, 200)
	# contours,hierarchy = cv.findContours(edges, 1, 2)
	# s_tape = []
	# for cnt in contours:
	# 	rect = cv.boundingRect(cnt)
	# 	x, y, w, h = rect
	# 	w_good = (w >= s_tape_minmax["w"][0] and w <= s_tape_minmax["w"][1])
	# 	h_good = (h >= s_tape_minmax["h"][0] and h <= s_tape_minmax["h"][1])
	# 	if (h_good and w_good):
	# 		s_tape.append(rect)
	# 		mask = np.zeros(frame_org.shape[:2], dtype=np.uint8)
	# 		cv.rectangle(mask, (int(x), int(y)), (int(x+w), int(y+h)), 255, -1)
	# 		# cv.imwrite("out.png", )
	# 		frame_org = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	# 		mean, std = cv.meanStdDev(frame_org, mask=mask)
	# 		print(std)
	# print(s_tape)
	# cv.imwrite("canny_out.png", edges)

ser.write(b's' + struct.pack('f', 1.0) + b'\n')
curr = None
start_time = 0
start1_time = time.time()
journey = []
retreat = False
prev_see_line = False
while True:
	# continue
	frame_org = cam.read()
	gs_frame_org = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	green_org = cv.inRange(gs_frame_org, boundaries["green"][0], boundaries["green"][1])
	# print(ret)
	# frame1 = frame[:, crop_h:(frame.shape[0])]
	# frame_org = cv.rotate(frame_org, cv.ROTATE_90_CLOCKWISE)
	frame = frame_org[crop_h:(frame_org.shape[0] - b_crop_h), :]
	gs_frame = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	green = green_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	blue_frame = frame_org[rk_crop_h:(frame_org.shape[0] - b_rk_crop_h), :]
	# print("gjiorjgi0owjri0o")

	frame_org_gray = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	edges = cv.Canny(frame_org_gray,100, 200)

	gs_frame_edges = edges[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	rotation = 0

	# print(curr)
	if curr:
		green = green[gs_lt_roi:(green.shape[0]) ,:]
		g_sum = np.sum(green)
		if curr == GreenSquare.DOUBLE:
			if (time.time() - start_time) < 2.5:
				rotation = 1
			else:
				curr = None
		else:
			if g_sum > 0:
				ser.write(b's' + struct.pack('f', 0.7) + b'\n')
				if curr == GreenSquare.RIGHT:
					rotation = right_turn_rotation
				elif curr == GreenSquare.LEFT:
					rotation = -right_turn_rotation
			else: 
				ser.write(b's' + struct.pack('f', 1.0) + b'\n')
				# ser.write(b's' + struct.pack('f', 1.0) + b'\n')
				curr = None
	else:
		lines = cv.HoughLines(gs_frame_edges,1,np.pi/180,60)
		curr = green_square(lines, green)
		start_time = time.time()

		gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
		t, thresh = cv.threshold(gray, b_w_thresh, 1 ,cv.THRESH_BINARY_INV)
		
		if see_stop(gs_frame_org, 90, 10):
			ser.write(b's' + struct.pack('f', 0) + b'\n')
		else: 
			ser.write(b's' + struct.pack('f', 1) + b'\n')

		dv = list(d_vector(thresh, h_scaled_v = scaled_v))#, w_scaled_v = w_scaled_v)
		
		deviation = 0
		if dv[1] != 0:
			deviation = math.atan(dv[0]/dv[1])

		deviation *= k_p 
		# deviation *= white_scale #** 10
		# print(deviation)
		deviation = 1 if deviation >= 1 else deviation
		deviation = -1 if deviation <= -1 else deviation
		# print(deviation)
		rotation = deviation
		# ser.write(b's' + struct.pack('f', 1) + b'\n')

		
		# print(b'l' + see_line + b'\n')

	# # blue_frame = cv.GaussianBlur(blue_frame, (5, 5), 0)

	# hsv = cv.cvtColor(blue_frame, cv.COLOR_BGR2HSV)
	# maskBlue = cv.inRange(hsv, boundaries["blue"][0], boundaries["blue"][1])
	# # maskBlue = cv.erode(maskBlue, None, iterations=2)
	# # maskBlue = cv.dilate(maskBlue, None, iterations=2)

	# if np.sum(maskBlue) >= 2000000:
	# 	ser.write(b's' + struct.pack('f', 0) + b'\n')
	# 	time.sleep(2)
	# 	print(journey)
	# 	retreat = True
	# elif np.sum(maskBlue) > 10000:
	# 	if not retreat:
	# 		print("blue")
	# 		Moments = cv.moments(maskBlue)
	# 		xBlue = int(Moments["m10"] / Moments["m00"])
	# 		yBlue = int(Moments["m01"] / Moments["m00"])
	# 		finalx = xBlue - frame.shape[1]/2
	# 		finaly = frame.shape[0] - yBlue
	# 		if xBlue!=0:
	# 			deviation = math.atan(finalx/finaly)
	# 			#sus?
	# 			deviation *= k_p
	# 			deviation = deviation if deviation <= 1 else 1
	# 			deviation = deviation if deviation >= -1 else -1
	# 			rotation = deviation
	# 			journey.append(deviation)

	
	# if retreat:
	# 	if len(journey)>0:
	# 		ser.write(b's' + struct.pack('f', -1) + b'\n')
	# 		print(len(journey))
	# 		rotation = journey[len(journey) - 1]
	# 		journey.pop()
	# 	else:
	# 		ser.write(b's' + struct.pack('f', 1) + b'\n')
	# 		retreat = False

	# print("r", rotation)

	ser.write(b'r' + struct.pack('f', rotation) + b'\n')
	ser.flush()

	# if (time.time() - start1_time) > 17:
	# 	# cam.kill()
	# 	# print("broke")
	# 	break