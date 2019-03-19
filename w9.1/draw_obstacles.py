import csv
from matplotlib import pyplot as plt

txtfile = open('FD_kout5_corner2.txt', 'r') 
'''
file format is x_coordinates, y_coordinates, z_coordinates of all building vertices
'''
txtread = csv.reader(txtfile, delimiter=',')
row1 = next(txtread)

def draw_obstacles():
	#plt.axis([5400000,5600000,4000000,4200000]) # define plot axes
	
	poly_x = []
	poly_y = []
	obs_x = []
	obs_y = []
	height_prev = row1[2] # get the first value of building heights

	for row in txtread:
		#print row
		height = row[2]
		if height == height_prev: # if height of the row is equal to prev
			obs_x.append(float(row[0])) # list of all x_coords of a building
			obs_y.append(float(row[1]))
			height_prev = row[2]
		elif height != height_prev:
			poly_x.append(obs_x)
			poly_y.append(obs_y)
			obs_x = [float(row[0])]
			obs_y = [float(row[1])]
			print height
			height_prev = row[2] # update new value of height

	# add the last building
	poly_x.append(obs_x)
	poly_y.append(obs_y)

	for i in range(len(poly_x)):
		plt.fill(poly_x[i], poly_y[i])

	print poly_x
	print height
	plt.show()

draw_obstacles()
txtfile.close()