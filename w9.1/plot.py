import matplotlib.pyplot as plt
import numpy as np 
import csv

csvfile = open('results.csv', 'r')
readcsv = csv.reader(csvfile, delimiter=',')

def plot():
	next(readcsv)
	x_array = []
	y_array1 = []
	y_array2 = []
	for row in readcsv:
		if row[0] == 'ave':
			x_array.append(float(row[4]))
			y_array1.append(round(float(row[1]),3))
			y_array2.append(round(float(row[2]),3))
	print x_array
	'''
	plt.scatter(x_array,y_array)
	plt.xlabel('Step size')
	plt.ylabel('Runtime')
	plt.show()
	'''
	#x_array, y_array1, y_array2 = zip(*sorted(zip(x_array, y_array1, y_array2)))
	fig, ax1 = plt.subplots()

	color = 'tab:red'
	ax1.set_xlabel('goal bias probability')
	ax1.set_ylabel('average path length (m)', color=color)
	ax1.plot(x_array, y_array1, color=color, marker='o')
	ax1.tick_params(axis='y', labelcolor=color)
	
	ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

	color = 'tab:blue'
	ax2.set_ylabel('average runtime (s)', color=color)  # we already handled the x-label with ax1
	ax2.plot(x_array, y_array2, color=color, marker='o')
	ax2.tick_params(axis='y', labelcolor=color)

	fig.tight_layout()  # otherwise the right y-label is slightly clipped
	
	plt.title('altitude = 100.92 m          step size = 150 m')
	plt.show()

plot()
csvfile.close()