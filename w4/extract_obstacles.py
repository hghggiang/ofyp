import csv
from geometry2 import *

txtfile = open('FD_kout5_corner.txt', 'r')

txtread = csv.reader(txtfile, delimiter=',')
row1 = next(txtread)

def extract_obstacles():

	height_prev = row1[2]
	temp_obsta = [[float(row1[0]),float(row1[1])]]
	temp_obstacles = []

	for row in txtread:
		height = row[2]
		if height == height_prev:
			obs_x = float(row[0])
			obs_y = float(row[1])
			temp_obsta.append([obs_x, obs_y])
			height_prev = row[2]
		elif height != height_prev:
			temp_obstacles.append(temp_obsta)
			temp_obsta = [[float(row1[0]),float(row1[1])]]
			height_prev = row[2]
	temp_obstacles.append(temp_obsta)

	obstacles = []
	for index, i in enumerate(temp_obstacles):
		temp_obs = []
		for j in i:
			temp = point(j[0], i[0], index)
			temp_obs.append(temp)
		obstacles.append(temp_obs)

	print obstacles

extract_obstacles()
txtfile.close()