import csv
import math

txtfile = open('input_file3', 'w+')
csvfile = open('FD_kout5_corner.csv', 'r')
readcsv = csv.reader(csvfile, delimiter=',')

row1 = next(readcsv)

def write_to_txt():
	txtfile.write('(0, 0), (0, 300), (300, 200), (200, 0) \n')
	prev_height = row1[2]

	for row in readcsv:
		current_height = row[2]
		if current_height == prev_height:
			if math.isnan(float(row[0])) is False:
				x = str(int(float(row[0])))
				y = str(int(float(row[1])))
				txtfile.write('(' + x + ',' + y + ')' + ',')
				prev_height = row[2]
		elif current_height != prev_height:
			x = str(int(float(row[0])))
			y = str(int(float(row[1])))
			txtfile.seek(-1,1)
			txtfile.truncate()
			txtfile.write('\n' + '(' + x + ',' + y + ')' + ',')
			prev_height = row[2]

	txtfile.seek(-1,1)
	txtfile.truncate()

	txtfile.write('\n(552600, 4183000), (553500, 4183500)')


write_to_txt()
txtfile.close()
csvfile.close()