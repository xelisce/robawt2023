import cv2 as cv
import numpy as np
import math
import serial
import struct
import time
from enum import Enum
from video_stream import VideoStream
from collections import defaultdict
from servo import Claw, Servo

#* Constants

class Servo(Enum):
	ARM = 0
	RIGHT = 1
	LEFT = 2
	COMPART = 3


class GreenSquare(Enum):
	RIGHT = 1
	LEFT = 2
	DOUBLE = 3

ser = serial.Serial('/dev/serial/by-id/usb-DFRobot_www.dfrobot.com__0042_75932313838351201101-if00', 9600, timeout=1, write_timeout=5)
dim = {"w": 160, "h": 120}
dim_e = {"w": 320, "h": 240}
boundaries = {
	"red": ((0,0,0), (0,0,0)),
	"green": ((50,50,50), (80,255,255)),
	"blue": ((94, 50, 50), (125, 255, 255))
}
crop_h = 30
b_crop_h = 10
gs_crop_h = 40
b_gs_crop_h = 0

b_w_thresh = 40 #
k_p = 1.0

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

right_turn_rotation = 0.75
min_area = 23

st_crop_h = 80
st_b_crop_h = 10

min_red = 50000

# element = cv.getStructuringElement(cv.MORPH_CROSS, (3,3))

#* Functions
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


def see_entry(gray, thresh, green, min_std_dev, min_mask_sum, t_crop, b_crop):
	"""Ensure that max(thresh) = 255!"""
	not_green = cv.bitwise_not(green)
	not_black = cv.bitwise_not(thresh)
	st_mask = cv.bitwise_and(not_black, not_green)

	st_mask_c = st_mask[t_crop:(st_mask.shape[0] - b_crop), :]
	gray_c = gray[t_crop:(gray.shape[0] - b_crop), :]

	_, std = cv.meanStdDev(gray_c ,mask = st_mask_c)
	print(std, np.sum(st_mask))
	return (std[0][0] > min_std_dev and np.sum(st_mask_c) > min_mask_sum)

def see_exit(green, min_green, t_crop, b_crop):
	green_c = green[t_crop:(green.shape[0] - b_crop), :]
	# print(np.sum(green_c))
	return (np.sum(green_c)) >= min_green*255


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

scaled_v = scaled_vector(dim["h"] - b_crop_h - crop_h, 1.5)
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

bound = 15
def blackish(frame, i, j):
	h, w = frame.shape
	i = i + bound if i == 0 else i
	i = i - bound if i == (h-1) else i
	j = j + bound if j == 0 else j
	j = j - bound if j == (w-1) else j
	return (frame[(i-bound):(i+bound), (j-bound):(j+bound)]).any()

l = 1
def green_square(lines, g_frame):
	global l
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
		# print(len(contours))
		if len(f_contours) == 2:
			return GreenSquare.DOUBLE
		# else:
		elif len(f_contours) == 1:
			# Cropping isn't perfect and some green still remains on the screen
			# this is to prevent the "specks" of green to count as a green square

			# cv.imwrite(f"out{l}.png", green)
			# print(f"dv{l} =", green_vector)
			l += 1
			if green_vector[0] < 0:
				return GreenSquare.LEFT
			elif green_vector[0] > 0:
				return GreenSquare.RIGHT
	# return (None, 0)

def turn(rotation, timeout):
	timeout_start = time.time()
	ser.write(b'r' + struct.pack('f', rotation) + b'\n')
	while time.time() < timeout_start + timeout:
		pass
	ser.write(b'r' + struct.pack('f', 0) + b'\n')

within_hw = lambda xy, hw: xy <= hw and xy >= 0

def segment_by_angle_kmeans(lines, k=2, **kwargs):
	"""
	Group lines by their angle using k-means clustering.

	Code from here:
	https://stackoverflow.com/a/46572063/1755401
	"""

	# Define criteria = (type, max_iter, epsilon)
	default_criteria_type = cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER
	criteria = kwargs.get('criteria', (default_criteria_type, 10, 1.0))

	flags = kwargs.get('flags', cv.KMEANS_RANDOM_CENTERS)
	attempts = kwargs.get('attempts', 10)

	# Get angles in [0, pi] radians
	angles = np.array([line[0][1] for line in lines])

	# Multiply the angles by two and find coordinates of that angle on the Unit Circle
	pts = np.array([[np.cos(2*angle), np.sin(2*angle)] for angle in angles], dtype=np.float32)

	# Run k-means
	# if sys.version_info[0] == 2:
	# 	# python 2.x
	# 	ret, labels, centers = cv.kmeans(pts, k, criteria, attempts, flags)
	# else: 
	# 	# python 3.x, syntax has changed.
	labels, centers = cv.kmeans(pts, k, None, criteria, attempts, flags)[1:]

	labels = labels.reshape(-1) # Transpose to row vector

	# Segment lines based on their label of 0 or 1
	segmented = defaultdict(list)
	for i, line in zip(range(len(lines)), lines):
		segmented[labels[i]].append(line)

	segmented = list(segmented.values())
	# print("Segmented lines into two groups: %d, %d" % (len(segmented[0]), len(segmented[1])))

	return segmented

claw = Claw(ser)
claw.lower().open()
# cam = VideoStream(resolution=(dim["h"], dim["w"])).start()
# cam.set(cv.CAP_PROP_FRAME_WIDTH, dim["w"])
# cam.set(cv.CAP_PROP_FRAME_HEIGHT,dim["h"])

# out = cv.VideoWriter('outpy.avi',cv.VideoWriter_fourcc('M','J','P','G'), 10, (dim["w"],dim["h"]))
# record_start = time.time()
# time.sleep(3)

# ser.write(b's' + struct.pack('f', 1.0) + b'\n')
# curr = None
# start_time = 0
# journey = []
# retreat = False
# prev_see_line = False
# while True:
# 	# continue
# 	frame_org = cam.read()
# 	gs_frame_org = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
# 	green_org = cv.inRange(gs_frame_org, boundaries["green"][0], boundaries["green"][1])
# 	# print(ret)
# 	# frame1 = frame[:, crop_h:(frame.shape[0])]
# 	# frame_org = cv.rotate(frame_org, cv.ROTATE_90_CLOCKWISE)
# 	frame = frame_org[crop_h:(frame_org.shape[0] - b_crop_h), :]
# 	gs_frame = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
# 	green = green_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
# 	blue_frame = frame_org[rk_crop_h:(frame_org.shape[0] - b_rk_crop_h), :]
# 	# print("gjiorjgi0owjri0o")

# 	frame_org_gray = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
# 	edges = cv.Canny(frame_org_gray,100, 200)

# 	gs_frame_edges = edges[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
# 	rotation = 0

# 	# print(curr)
# 	if curr:
# 		green = green[gs_lt_roi:(green.shape[0]) ,:]
# 		g_sum = np.sum(green)
# 		if curr == GreenSquare.DOUBLE:
# 			if (time.time() - start_time) < 2.5:
# 				rotation = 1
# 			else:
# 				curr = None
# 		else:
# 			if g_sum > 0:
# 				ser.write(b's' + struct.pack('f', 0.7) + b'\n')
# 				if curr == GreenSquare.RIGHT:
# 					rotation = right_turn_rotation
# 				elif curr == GreenSquare.LEFT:
# 					rotation = -right_turn_rotation
# 			else: 
# 				ser.write(b's' + struct.pack('f', 1.0) + b'\n')
# 				# ser.write(b's' + struct.pack('f', 1.0) + b'\n')
# 				curr = None
# 	else:
# 		lines = cv.HoughLines(gs_frame_edges,1,np.pi/180,60)
# 		curr = green_square(lines, green)
# 		start_time = time.time()

# 		gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
# 		t, thresh = cv.threshold(gray, b_w_thresh, 1 ,cv.THRESH_BINARY_INV)
		
# 		if see_stop(gs_frame_org, 90, 10):
# 			ser.write(b's' + struct.pack('f', 0) + b'\n')
# 		else: 
# 			ser.write(b's' + struct.pack('f', 1) + b'\n')

# 		#* to tell the robot when to stop when it sees an obstacle
# 		# obs_frame = thresh[obs_crop_h:thresh.shape[0], :]
# 		# see_line = np.sum(obs_frame) > line_min
# 		# print(np.sum(obs_frame))
# 		# if see_line:#!= prev_see_line:
# 		# 	print("hi")
# 		# 	ser.write(b'l' + (1).to_bytes(1, "big") + b'\n')
# 		# prev_see_line = see_line
# 		#* detecting the silver tape
# 		# st_green_mask = cv.bitwise_not(green_org[crop_h:(frame_org.shape[0] - b_crop_h), :])
# 		# st_green_mask = st_green_mask[st_crop_h:(st_green_mask.shape[0] - st_b_crop_h), :]
# 		# mean, std = cv.meanStdDev(gray ,mask = cv.bitwise_and(cv.bitwise_not(thresh), st_green_mask))
# 		# t, thresh = cv.threshold(gray, 0, 1 ,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
		
# 		#* Scaling based on the amount of white in the image (abit hacky)
# 		# white_scale = (((np.sum(cv.bitwise_not(thresh * 255))/(dim["w"] * dim["h"] * 255))) ** 1.5 ) * 1.2 + 0.2 #the magical number
# 		# white_scale = white_scale if white_scale >= 1 else 1
# 		# white_scale = 2 if white_scale >= 2 else white_scale

# 		#* Main vector line-tracking logic
# 		# dv = list(d_vector(thresh, scaled_m = scaled_m))
# 		dv = list(d_vector(thresh, h_scaled_v = scaled_v))#, w_scaled_v = w_scaled_v)
# 		# print(dv)
# 		#* Reynard's special sauce algorithm thingy
# 		# frame_edges = edges[crop_h:(frame_org.shape[0] - b_crop_h), :]
# 		# line_edges = cv.Canny(thresh*255, 100, 200)
# 		# h, w = gs_frame_edges.shape
# 		# lines = cv.HoughLines(line_edges,1,np.pi/180,30)
# 		# # majik = 30
# 		# # roi_black = thresh[30:(h-30), 30:(w-30)]
# 		# # black_ratio = np.sum(roi_black)/((h-2*30)*(w-2*30))
# 		# # print(white_ratio)
# 		# if lines is not None: #and black_ratio > 0.2:
# 		# 	try:
# 		# 		side = apex_point(lines)
# 		# 		for line in lines:
# 		# 			# print("hi")
# 		# 			v = [0, 0]
# 		# 			y_intercept = y_value(line, 0)
# 		# 			x_intercept = x_value(line, 0)
# 		# 			width_intercept = y_value(line, w)
# 		# 			height_intercept = x_value(line, h)
# 		# 			if not single_line(lines):
# 		# 				if within_hw(y_intercept, h) and blackish(thresh ,int(y_intercept), 0) and side == GreenSquare.LEFT:
# 		# 					# print(p, q)
# 		# 					v[0] -= w/2
# 		# 					v[1] += (h - y_intercept) 
# 		# 				# if within_hw(x_intercept, w) and blackish(thresh, 0, int(x_intercept)):
# 		# 				# 	# print("B")
# 		# 				# 	v[0] += (x_intercept - w/2)
# 		# 				# 	v[1] += h
# 		# 				if within_hw(width_intercept, h) and blackish(thresh, int(width_intercept), w - 1) and side == GreenSquare.RIGHT:
# 		# 					# print(p)
# 		# 					v[0] += w/2
# 		# 					v[1] += (h - width_intercept) 
# 		# 			# if within_hw(height_intercept, w) and blackish(thresh, h - 1, int(height_intercept)):
# 		# 			# 	# print("D")
# 		# 			# 	v[0] += (height_intercept - w/2)
# 		# 			# 	v[1] += h
# 		# 				print(v, side)
# 		# 				dv[0] += v[0] * 375
# 		# 				dv[1] += v[1] * 100
# 		# 	except:
# 		# 		pass
# 			# cv.circle(frame, point[0], 2,(0,0,255), 2)
# 		# print(apex_v)
# 		# cv.imwrite("out.png", frame)
# 			# for line in lines:
# 			# 	rho,theta = line[0]
# 			# 	a = np.cos(theta)
# 			# 	b = np.sin(theta)
# 			# 	x0 = a*rho
# 			# 	y0 = b*rho
# 			# 	x1 = int(x0 + 1000*(-b))
# 			# 	y1 = int(y0 + 1000*(a))
# 			# 	x2 = int(x0 - 1000*(-b))
# 			# 	y2 = int(y0 - 1000*(a))
# 			# 	cv.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
# 			# cv.imwrite("pp.png", frame)
# 		#* Edge detection
# 		# thinned = cv.ximgproc.thinning(thresh * 255, cv.ximgproc.THINNING_GUOHALL)
# 		# corners = cv.goodFeaturesToTrack(thresh ,2,0.1,30)
# 		# if corners is not None:
# 		# 	for p in corners:
# 		# 		x, y = p.ravel()
# 		# 		if not (x < 20 or x > (dim["w"] - 20) or y < 20 or y > (dim["h"] - 20)):
# 		# 			x -= dim["w"] / 2
# 		# 			dv[0] -= x * 1000
# 		# 			dv[1] += y * 1000
# 		# 			cv.circle(gray,(int(x),int(y)),5,255,-1)
# 		# 			# print(x, y)
# 		deviation = 0
# 		if dv[1] != 0:
# 			deviation = math.atan(dv[0]/dv[1])

# 		deviation *= k_p 
# 		# deviation *= white_scale #** 10
# 		# print(deviation)
# 		deviation = 1 if deviation >= 1 else deviation
# 		deviation = -1 if deviation <= -1 else deviation
# 		print(deviation)
# 		rotation = deviation
# 		# ser.write(b's' + struct.pack('f', 1) + b'\n')

		
# 		# print(b'l' + see_line + b'\n')

# 	# # blue_frame = cv.GaussianBlur(blue_frame, (5, 5), 0)

# 	# hsv = cv.cvtColor(blue_frame, cv.COLOR_BGR2HSV)
# 	# maskBlue = cv.inRange(hsv, boundaries["blue"][0], boundaries["blue"][1])
# 	# # maskBlue = cv.erode(maskBlue, None, iterations=2)
# 	# # maskBlue = cv.dilate(maskBlue, None, iterations=2)

# 	# if np.sum(maskBlue) >= 2000000:
# 	# 	ser.write(b's' + struct.pack('f', 0) + b'\n')
# 	# 	time.sleep(2)
# 	# 	print(journey)
# 	# 	retreat = True
# 	# elif np.sum(maskBlue) > 10000:
# 	# 	if not retreat:
# 	# 		print("blue")
# 	# 		Moments = cv.moments(maskBlue)
# 	# 		xBlue = int(Moments["m10"] / Moments["m00"])
# 	# 		yBlue = int(Moments["m01"] / Moments["m00"])
# 	# 		finalx = xBlue - frame.shape[1]/2
# 	# 		finaly = frame.shape[0] - yBlue
# 	# 		if xBlue!=0:
# 	# 			deviation = math.atan(finalx/finaly)
# 	# 			#sus?
# 	# 			deviation *= k_p
# 	# 			deviation = deviation if deviation <= 1 else 1
# 	# 			deviation = deviation if deviation >= -1 else -1
# 	# 			rotation = deviation
# 	# 			journey.append(deviation)

	
# 	# if retreat:
# 	# 	if len(journey)>0:
# 	# 		ser.write(b's' + struct.pack('f', -1) + b'\n')
# 	# 		print(len(journey))
# 	# 		rotation = journey[len(journey) - 1]
# 	# 		journey.pop()
# 	# 	else:
# 	# 		ser.write(b's' + struct.pack('f', 1) + b'\n')
# 	# 		retreat = False

# 	# print("r", rotation)

# 	ser.write(b'r' + struct.pack('f', rotation) + b'\n')
# 	ser.flush()

# cam.kill()
cam = VideoStream(resolution=(dim_e["h"], dim_e["w"])).start()
start_evac = time.time()
#in the evac zone
print("switched loops, in evac zone")
ser.write(b't' + (1).to_bytes(1, "big") + b'\n')

while True:
	# continue
	frame_org = cam.read()
	# frame_org = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	green = cv.inRange(frame_org_hsv, boundaries["green"][0], boundaries["green"][1])
	gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)
	st = see_entry(gray_org, thresh, green, 43, 15000000, 180, 20)
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
	cv.imwrite("pain.png",thresh_b)
	if len(balls) > 0:
		biggest_ball = max(balls, key=lambda b: b['r'])
		print(biggest_ball['x'] - dim["w"]/2, biggest_ball['r'])
		ser.write(b'r' + struct.pack('f', (-0.30 if (biggest_ball['x'] - dim["w"]/2) < 0 else 0.30)) + b'\n')
	else:
		ser.write(b'r' + struct.pack('f', 0) + b'\n')
	# print(count)
	if np.sum(thresh_b) > 3000000:
		ser.write(b'p' + (1).to_bytes(1, "big") + b'\n')
		if np.sum(thresh) > 1000000:
			claw.dead()
		else:
			claw.alive()
		claw.lower().open()
		ser.write(b'p' + (0).to_bytes(1, "big") + b'\n')
	
	if start_evac - time.time() == 240:
		break

evac_zone = True
# after all the balls are done
print("start trying to deposit")
deposit(60)
claw_deposit(claw)

while True:
	pass
# while True:
# 	frame_org = cam.read()
# 	# cv.imwrite("org.png", frame_org)
# 	# frame_org = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
# 	frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
# 	gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
# 	t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)
# 	contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
# 	for contour in contours:
# 		rect = cv.boundingRect(contour)
# 		x, y, w, h = rect
# 		print(w)
# 		claw.raisin().delay(1).reset_compartment()
# 		if w > 150 and h> 90:
# 			print("commence deposit")
# 			ser.write(b's' + struct.pack('f', 0) + b'\n')
# 			ser.write(b'r' + struct.pack('f', 0) + b'\n')
# 			claw.write_claw(Servo.ARM, 80).close(95).delay(0.5)
# 			claw.tilt_compartment()
# 			claw.open().delay(0.5)
# 			claw.raisin().delay(0.5)
# 			claw.reset_compartment()
# 			ser.write(b's' + struct.pack('f', -1) + b'\n')
# 			ser.write(b'r' + struct.pack('f', 0) + b'\n')
# 			time.sleep(1)
# 			claw.lower().open()
# 			evac_zone = False
# 		elif w > 100:
# 			ser.write(b's' + struct.pack('f', 0.8) + b'\n')
# 			ser.write(b'r' + struct.pack('f', 0) + b'\n')
# 			print("straight")
# 		else:
# 			ser.write(b's' + struct.pack('f', 1) + b'\n')
# 			ser.write(b'r' + struct.pack('f', 0) + b'\n')
# 			print("roomba")
# 		print("h:", h)
	
# 	if evac_zone == False:
# 		ser.write(b'e' + (1).to_bytes(1, "big") + b'\n')
# 		break

# after it deposits balls, continue to see green tape
print("looking for green tape")

while True:
	# continue
	frame_org = cam.read()
	# frame_org = frame_org[gs_crop_h:(frame_org.shape[0] - b_gs_crop_h), :]
	frame_org_hsv = cv.cvtColor(frame_org, cv.COLOR_BGR2HSV)
	green = cv.inRange(frame_org_hsv, boundaries["green"][0], boundaries["green"][1])
	gray_org = cv.cvtColor(frame_org, cv.COLOR_BGR2GRAY)
	t, thresh = cv.threshold(gray_org, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)
	gt = see_exit(green, 120, 90, 10)
	if np.sum(green) > 10:
			print("end")
			Moments = cv.moments(green)
			xGreen = int(Moments["m10"] / Moments["m00"])
			yGreen = int(Moments["m01"] / Moments["m00"])
			finalx = xGreen - frame.shape[1]/2
			finaly = frame.shape[0] - yGreen
			if xGreen!=0:
				deviation = math.atan(finalx/finaly)
				#sus?
				deviation *= k_p
				deviation = deviation if deviation <= 1 else 1
				deviation = deviation if deviation >= -1 else -1
				rotation = deviation
	else:
		ser.write(b's' + struct.pack('f', 1) + b'\n')
		ser.write(b'r' + struct.pack('f', 0) + b'\n')
	if gt:
		ser.write(b'g' + (1).to_bytes(1, "big") + b'\n')
		ser.write(b's' + struct.pack('f', 1) + b'\n')
		ser.write(b'r' + struct.pack('f', 0) + b'\n')
		break

# after it sees  green tape
print("commence line tracj again")

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

	t, thresh = cv.threshold(frame_org_gray, b_w_thresh, 255 ,cv.THRESH_BINARY_INV)

	gt = see_entry(frame_org_gray, thresh, green_org, 43, 15000000, 180, 20)
		

	gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
	t, thresh = cv.threshold(gray, b_w_thresh, 1 ,cv.THRESH_BINARY_INV)
		
	ser.write(b'l' + (1).to_bytes(1, "big") + b'\n')
		
		#* detecting the silver tape
		# mean, std = cv.meanStdDev(gray ,mask = cv.bitwise_and(cv.bitwise_not(thresh), st_green_mask))
		# t, thresh = cv.threshold(gray, 0, 1 ,cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
		
		#* Scaling based on the amount of white in the image (abit hacky)

		#* Main vector line-tracking logic
		# dv = list(d_vector(thresh, scaled_m = scaled_m))
	dv = list(d_vector(thresh, h_scaled_v = scaled_v))#, w_scaled_v = w_scaled_v)
		# print(dv)
		#* Reynard's special sauce algorithm thingy
		# frame_edges = edges[crop_h:(frame_org.shape[0] - b_crop_h), :]
		# line_edges = cv.Canny(thresh*255, 100, 200)
		# lines = cv.HoughLines(line_edges,1,np.pi/180,30)
		# # majik = 30
		# # roi_black = thresh[30:(h-30), 30:(w-30)]
		# # black_ratio = np.sum(roi_black)/((h-2*30)*(w-2*30))
		# # print(white_ratio)

			# cv.circle(frame, point[0], 2,(0,0,255), 2)
		# print(apex_v)
		# cv.imwrite("out.png", frame)
			# for line in lines:
			# 	rho,theta = line[0]
			# 	a = np.cos(theta)
			# 	b = np.sin(theta)
			# 	x0 = a*rho
			# 	y0 = b*rho
			# 	x1 = int(x0 + 1000*(-b))
			# 	y1 = int(y0 + 1000*(a))
			# 	x2 = int(x0 - 1000*(-b))
			# 	y2 = int(y0 - 1000*(a))
			# 	cv.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
			# cv.imwrite("pp.png", frame)
		#* Edge detection
		# thinned = cv.ximgproc.thinning(thresh * 255, cv.ximgproc.THINNING_GUOHALL)
		# corners = cv.goodFeaturesToTrack(thresh ,2,0.1,30)
		# if corners is not None:
		# 	for p in corners:
		# 		x, y = p.ravel()
		# 		if not (x < 20 or x > (dim["w"] - 20) or y < 20 or y > (dim["h"] - 20)):
		# 			x -= dim["w"] / 2
		# 			dv[0] -= x * 1000
		# 			dv[1] += y * 1000
		# 			cv.circle(gray,(int(x),int(y)),5,255,-1)
		# 			# print(x, y)
	deviation = 0
	if dv[1] != 0:
		deviation = math.atan(dv[0]/dv[1])

	deviation *= k_p 
	# deviation *= white_scale #** 10
	# print(deviation)
	deviation = 1 if deviation >= 1 else deviation
	deviation = -1 if deviation <= -1 else deviation
	print(deviation)
	rotation = deviation
	ser.write(b'r' + struct.pack('f', rotation) + b'\n')

	if see_stop(gs_frame_org, 90, 10):
		ser.write(b's' + struct.pack('f', 0) + b'\n')
	else: 
		ser.write(b's' + struct.pack('f', 1) + b'\n')

#a