import numpy as np
import cv2
from matplotlib import pyplot as plt
import argparse

def parse_data(string, list_):
	idx = string.find(" ")
	sub_str = string[idx+1:]
	while True:
		idx = sub_str.find(" ")
		if idx == -1:
			break
		sub_str = sub_str[idx+1:]
	try:
		data = float(sub_str)
	except ValueError:
		data = int(sub_str)
	list_.append(data)
	return list_

parser = argparse.ArgumentParser(prog="visual_suck_and_grasp", description="Plot logged data")
parser.add_argument("--episode", type=int, help="Episodes trained")
parser.add_argument("--data_path", type=str, help="Path to logged data")
parser.add_argument("--img_name", type=str, help="Name of saved image")
parser.add_argument("--draw_init", action="store_true", help="If draw first images")
parser.add_argument("--is_testing", action="store_true", help="Test mode if provided")

args = parser.parse_args()

trained_episode = args.episode

iterations = []
return_ = []
loss_mean = []
num_invalid = []
num_valid = []
ratio = [] # Seems redundant
action_ratio = [] # \frac{valid_action}{Total iter}

for i in range(trained_episode):
	file_name = args.data_path + "logger_{}/curve.txt".format(i)
	if args.is_testing:
		file_name = args.data_path + "test_logger_{}/curve.txt".format(i)
	f = open(file_name, 'r')
	s = f.readline()
	iterations = parse_data(s, iterations)
	s = f.readline()
	return_ = parse_data(s, return_)
	s = f.readline()
	loss_mean = parse_data(s, loss_mean)
	s = f.readline()
	num_invalid = parse_data(s, num_invalid)
	s = f.readline()
	num_valid = parse_data(s, num_valid)
	s = f.readline()
	ratio = parse_data(s, ratio)

for idx in range(len(iterations)):
	val = float(num_valid[idx])/iterations[idx]
	action_ratio.append(val)

plt.figure(figsize=(16,9))
plt.subplot(3,2,1)
plt.plot(iterations)
plt.title("Iterations")
plt.subplot(3,2,2)
plt.plot(loss_mean)
plt.title("Mean Loss")
plt.subplot(3,2,3)
plt.plot(num_invalid)
plt.title("Number of Invalid Action")
plt.subplot(3,2,4)
plt.plot(num_valid)
plt.title("Number of Valid Action")
plt.subplot(3,2,5)
plt.plot(return_)
plt.title("Return at t=0")
plt.subplot(3,2,6)
plt.plot(action_ratio)
plt.title("Valid Action / Total Iterations")

plt.savefig("%s.png" %args.img_name, dpi=100)

#######################################################################
# Initial image
if args.draw_init:
	result = np.zeros((112*8, 112*8, 3))
	for i in range(trained_episode):
		col = i % 8
		row = i / 8
		image_name = args.data_path + "logger_{}/images/color_000000.jpg".format(i)
		img = cv2.imread(image_name)
		img_resize = cv2.resize(img, (112, 112))
		result[row*112:(row+1)*112, col*112: (col+1)*112, :] = img_resize

	cv2.imwrite("initial.jpg", result)
