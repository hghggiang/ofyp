import csv
from math import *

csvfile1 = open('node_list_connected.csv', 'r')
csvfile2 = open('edge_list_connected.csv', 'r')
csvfile3 = open('network_connected.csv', 'w')

readcsv1 = csv.reader(csvfile1, delimiter=',')
readcsv2 = csv.reader(csvfile2, delimiter=',')
writecsv = csv.writer(csvfile3)

MULT = 100

def csv2txt():
	node_list = []
	next(readcsv1) # skip headings
	edge_row1 = next(readcsv2)
	previous_edge = edge_row1[0]
	neighbor_list = [edge_row1[1]]

	# first, add all nodes x and y
	for row in readcsv1:
		x = str(int(floor(float(row[1])*MULT)))
		y = str(int(floor(float(row[2])*MULT)))
		node_list.append([int(x),int(y)])

	# append neighbors' x and y to nodes if any
	for row in readcsv2:
		current_edge = row[0]
		if current_edge == previous_edge:
			neighbor_list.append(row[1])
			previous_edge = row[0]
			# print neighbor_list
		elif current_edge != previous_edge:
			for item in neighbor_list:
				neighbor = node_list[int(item)-1]
				neighbor_x = node_list[int(item)-1][0]
				neighbor_y = node_list[int(item)-1][1]
				node_list[int(previous_edge)-1].append(neighbor_x)
				node_list[int(previous_edge)-1].append(neighbor_y)
				node_x = node_list[int(previous_edge)-1][0]
				node_y = node_list[int(previous_edge)-1][1]
				node_list[int(item)-1].append(node_x)
				node_list[int(item)-1].append(node_y)
			neighbor_list = [row[1]]
			previous_edge = row[0]
	print len(node_list)
	writecsv.writerows(node_list)
csv2txt()

csvfile1.close()
csvfile2.close()
csvfile3.close()


# write results to txt file -----------------------------------------
csvfile3 = open('network_connected.csv', 'r')
readcsv3 = csv.reader(csvfile3, delimiter=',')
txtfile = open('network_connected.txt', 'w+')

for row in readcsv3:
	for i in range(0, len(row)-1):
		txtfile.write(str(row[i]) + ',')
	txtfile.write(str(row[len(row)-1]) + '\n')

csvfile3.close()
txtfile.close()