'''
import pandas
df = pandas.read_csv('FD_kout5_corner.csv', index_col='z_coordinate')
print(df)
'''

import csv
with open('FD_kout5_corner.csv', 'rb') as csvfile:
     spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
     for row in spamreader:
         print ', '.join(row)