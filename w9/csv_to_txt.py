import csv

txtfile = open('obstacles_1a', 'w+')
csvfile = open('buildings.csv', 'r')
readcsv = csv.reader(csvfile, delimiter=',')

altitude = 8.79

def write_to_txt():
	#txtfile.write('(551967,4181847),(553782,4181847),(553782,4183746),(551967,4183746)\n')
	txtfile.write('(550000,4180000),(555000,4180000),(555000,4190000),(550000,4190000)\n')
	#txtfile.write('(500000,4170000),(560000,4170000),(560000,4190000),(500000,4190000)\n')


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
	#txtfile.write('(552288,4182078),(553004,4183309)')
	txtfile.write('(552528,4181963),(552751,4183657)')

write_to_txt()
txtfile.close()
csvfile.close()