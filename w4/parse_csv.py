import csv

txtfile = open('FD_kout5_corner.txt', 'w')

with open('FD_kout5_corner.csv') as csvfile:
	readcsv = csv.reader(csvfile, delimiter=',')
	for row in readcsv:
		#print(row)
		#print(row[0])
		print(row[0] + "," + row[1] + "," + row[2])
		txtfile.write(row[0] + "," + row[1] + "," + row[2] + "\n")

txtfile.close()
csvfile.close()