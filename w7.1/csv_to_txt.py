import csv

txtfile = open('obstacles', 'w+')
csvfile = open('FD_kout5_corner.csv', 'r')
readcsv = csv.reader(csvfile, delimiter=',')

row1 = next(readcsv)

def write_to_txt():
	prev_height = row1[2]

	for row in readcsv:
		current_height = row[2]
		if current_height == prev_height:
			x = row[0]
			y = row[1]
			txtfile.write('(' + x + ',' + y + ')' + ',')
			prev_height = row[2]
		elif current_height != prev_height:
			x = row[0]
			y = row[1]
			txtfile.seek(-1,1)
			txtfile.truncate()
			txtfile.write('\n' + '(' + x + ',' + y + ')' + ',')
			prev_height = row[2]

	txtfile.seek(-1,1)
	txtfile.truncate()

write_to_txt()
txtfile.close()
csvfile.close()