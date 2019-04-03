import csv
from matplotlib import pyplot as plt

csvfile = open('buildings(1).csv', 'r')
readcsv = csv.reader(csvfile, delimiter=',')
altitude = 0

def draw_obstacles():
	#plt.axis([5400000,5600000,4000000,4200000]) # define plot axes
	next(readcsv)

	poly_x = []
	poly_y = []


	for row in readcsv:
		print row
		current_height = float(row[9])
		if current_height >= altitude:
			x1 = float(row[1])
			y1 = float(row[2])
			x2 = float(row[3])
			y2 = float(row[4])
			x3 = float(row[5])
			y3 = float(row[6])
			x4 = float(row[7])
			y4 = float(row[8])
			obs_x = [x1,x2,x3,x4]
			obs_y = [y1,y2,y3,y4]
			poly_x.append(obs_x)
			poly_y.append(obs_y)			


	for i in range(len(poly_x)):
		plt.fill(poly_x[i], poly_y[i])

	print poly_x

	plt.show()

draw_obstacles()
txtfile.close()