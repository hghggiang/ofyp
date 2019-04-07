import csv
from math import *

csv1 = open('node_list.csv', 'r')
readcsv1 = csv.reader(csv1, delimiter=',')
csv2 = open('idx.csv', 'r')
readcsv2 = csv.reader(csv2, delimiter=',')
csv3 = open('node_list_connected.csv', 'w+')
writecsv3 = csv.writer(csv3)

MULT = 100
node_list = []
node_list_connected = []
next(readcsv1) # skip headings

# first, add all nodes x and y
for row in readcsv1:
	x = str(int(floor(float(row[1])*MULT)))
	y = str(int(floor(float(row[2])*MULT)))
	node_list.append([int(x),int(y)])
# print node_list
# print len(node_list)
i=0
for row in readcsv2:
	i+=1
	if row[0] == 'TRUE':
		line = [i,node_list[i-1][0],node_list[i-1][1]]
		node_list_connected.append(line)
	
print node_list_connected
print len(node_list_connected)
print i
writecsv3.writerow(['index','x','y'])
writecsv3.writerows(node_list_connected)

csv1.close()
csv2.close()
csv3.close()