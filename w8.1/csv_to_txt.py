import csv

txtfile = open('obstacles_19', 'w+')
csvfile = open('buildings.csv', 'r')
readcsv = csv.reader(csvfile, delimiter=',')

altitude = 132.51

def write_to_txt():
	txtfile.write('(551967,4181847),(553782,4181847),(553782,4183746),(551967,4183746)\n')
	next(readcsv)
	for row in readcsv:
		current_height = float(row[10])
		if current_height >= altitude:
			x1 = str(int(float(row[2])))
			y1 = str(int(float(row[3])))
			txtfile.write('(' + x1 + ',' + y1 + ')' + ',')
			x2 = str(int(float(row[4])))
			y2 = str(int(float(row[5])))
			txtfile.write('(' + x2 + ',' + y2 + ')' + ',')
			x3 = str(int(float(row[6])))
			y3 = str(int(float(row[7])))
			txtfile.write('(' + x3 + ',' + y3 + ')' + ',')
			x4 = str(int(float(row[8])))
			y4 = str(int(float(row[9])))
			txtfile.write('(' + x4 + ',' + y4 + ')' + ',')
			txtfile.write('(' + x1 + ',' + y1 + ')' + '\n')
	txtfile.write('(552288,4182078),(553004,4183309)')

write_to_txt()
txtfile.close()
csvfile.close()