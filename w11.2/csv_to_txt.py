import csv

txtfile = open('obstacles_15', 'w+')
csvfile = open('buildings(1).csv', 'r')
readcsv = csv.reader(csvfile, delimiter=',')

# altitude = 8.79
# altitude = 11.76
# altitude = 13.1
# altitude = 13.95
# altitude = 15.7
# altitude = 17.55
# altitude = 19.28
# altitude = 21.61
# altitude = 23.94
# altitude = 26.82
# altitude = 30.08
# altitude = 33.05
# altitude = 38.14
# altitude = 43.78
altitude = 51.78 # 15th
# altitude = 65.71 # 16th
# altitude = 82.59 # 17th
# altitude = 100.92 # 18th
# altitude = 132.51 # 19th
# altitude = 258.49 # 20th

def write_to_txt():
	# define search space
	# txtfile.write('(550000,4180000),(555000,4180000),(555000,4190000),(550000,4190000)\n')
	# txtfile.write('(552204,4181833),(553685,4181833),(553685,4183823),(552204,4183823)\n') # no expansion
	txtfile.write('(552058,4181636),(553832,4181636),(553832,4184021),(552058,4184021)\n') # 10% expansion in each direction
	# txtfile.write('(551911,4181438),(553685,4181438),(553685,4184219),(551911,4184219)\n') # 20% expansion in each direction

	# extract obstacle vertices
	next(readcsv)
	for row in readcsv:
		current_height = float(row[9])
		if current_height >= altitude:
			x1 = str(int(float(row[1])))
			y1 = str(int(float(row[2])))
			txtfile.write('(' + x1 + ',' + y1 + ')' + ',')
			x2 = str(int(float(row[3])))
			y2 = str(int(float(row[4])))
			txtfile.write('(' + x2 + ',' + y2 + ')' + ',')
			x3 = str(int(float(row[5])))
			y3 = str(int(float(row[6])))
			txtfile.write('(' + x3 + ',' + y3 + ')' + ',')
			x4 = str(int(float(row[7])))
			y4 = str(int(float(row[8])))
			txtfile.write('(' + x4 + ',' + y4 + ')' + ',')
			txtfile.write('(' + x1 + ',' + y1 + ')' + '\n')

	# specify origin - destination pair
	txtfile.write('(552528,4181963),(552751,4183657)')

write_to_txt()
txtfile.close()
csvfile.close()