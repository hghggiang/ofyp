# Find a path avoiding obstacles using modified RRT
# Author -- Shikhar Dev Gupta

from random import *
from helpers.graph import *
from helpers.geometry import *
import matplotlib.pyplot as plt
import time
import math # for calculating path length

plt.rcParams.update({'font.size': 7})

start_time = time.time()

# Check for empty lines
file_handler = open("input/obstacles_16","r");
raw_data = file_handler.read();
raw_data = raw_data.split("\n");
step_size = 50;
bias_prob = 0.9;
# Uncomment current altitude
# altitude = 8.79 # 1st
# altitude = 11.76 # 2nd
# altitude = 13.1 # 3rd
# altitude = 13.95 # 4th
# altitude = 15.7 # 5th
# altitude = 17.55 # 6th
# altitude = 19.28 # 7th
# altitude = 21.61 # 8th
# altitude = 23.94 # 9th
# altitude = 26.82 # 10th
# altitude = 30.08 # 11th
# altitude = 33.05 # 12th
# altitude = 38.14 # 13th
# altitude = 43.78 # 14th
# altitude = 51.78 # 15th
altitude = 65.71 # 16th
# altitude = 82.59 # 17th
# altitude = 100.92 # 18th
# altitude = 132.51 # 19th
# altitude = 258.49 # 20th

if(len(raw_data) <2):
	print("Incorrect format of the input file");
	exit;

def parse_input_line(line):
	temp2 = [];
	line = [i.strip() for i in line.split(",")];
	vertex = [];
	for index,i in enumerate(line):
		if(i[0] == "("):
			i = i[1:];
		if(i[len(i)-1] == ")"):
			i= i[:-1];
		vertex.append(int(i));
		if(index%2 != 0):
			temp2.append(vertex);
			vertex = [];
	return temp2;	


# Draw the obstacles and point the source and the destination----------------------------------------------
def draw_problem():
	bnd_x = [i.x for i in boundary];
	bnd_x.append(boundary[0].x);
	bnd_y = [i.y for i in boundary];
	bnd_y.append(boundary[0].y);
	poly_x = [];
	poly_y = []

	# Draw the boundary
	plt.plot(bnd_x, bnd_y);
	
	for index, i in enumerate(obstacles):
		poly_x.append([p[0] for p in i]);
		poly_y.append([p[1] for p in i]);
	
		plt.fill( poly_x[index], poly_y[index], color="#512DA8");
	
	plt.plot(source.x, source.y, marker="o");
	plt.plot(dest.x, dest.y, marker="o");
	plt.annotate('Source', xy=(source.x, source.y), xytext=(source.x+5, source.y-6) );
	plt.annotate('Destination', xy=(dest.x, dest.y), xytext=(dest.x-4, dest.y-10) );

# Choose goal with some probability--------------------------------------------
def choose_target():
	if(uniform(0,1) < bias_prob):
		return dest;
	else:
		return random_point([boundary[0].x, boundary[1].x], [boundary[0].y, boundary[2].y]);


# Extract vertices---------------------------------------------------
temp = parse_input_line(raw_data[0]);
boundary = [point(i[0], i[1]) for i in temp];
# To manually select boundary, uncomment desired pair
# boundary = [point(552204,4181833),point(553685,4181833),point(553685,4183823),point(552204,4183823)]; search_space = 'no expansion' # no expansion
# boundary = [point(552058,4181636),point(553832,4181636),point(553832,4184021),point(552058,4184021)]; search_space = '10 percent expansion' # 10% expansion in each direction
boundary = [point(551911,4181438),point(553979,4181438),point(553979,4184219),point(551911,4184219)]; search_space = '20 percent expansion' # 20% expansion in each direction

# Extract source and dest
temp = parse_input_line(raw_data[len(raw_data)-1]);
source = point(temp[0][0], temp[0][1]);
dest = point(temp[1][0], temp[1][1]);
# To manually select source and destination, uncomment desired OD pair
# source = point(552288,    4182078); dest = point(553004,  4183309); # pair 1
# source = point(552182,	4182354); dest = point(553533,	4182497); # pair 2
# source = point(552528,	4181963); dest = point(552751,	4183657); # pair 3
# source = point(552198,	4182276); dest = point(553252,	4183300); # pair 4: almost direct straight line
# source = point(552226,	4183360); dest = point(553782,	4182919); # pair 5: difficult pair, tends to terminate by reaching iteration limit
source = point(552086,	4182792); dest = point(553126,	4182157); # pair 6
# source = point(553546,	4183389); dest = point(553076,	4181847); # pair 7
# source = point(553197,	4181960); dest = point(553140,	4183746); # pair 8
# source = point(552800,	4183680); dest = point(552200,	4182500); # pair 9
# source = point(553690,	4182968); dest = point(552182,	4182354); # pair 10

# Extract obstacles
temp_obstacles = [];
for i in raw_data[1:len(raw_data)-1]:
	temp_obstacles.append(parse_input_line(i) );

obstacles = [];
for index, i in enumerate(temp_obstacles):
	temp_obs = [];
	for j in i:
		temp = point(j[0], j[1], index);
		temp_obs.append(temp);
	obstacles.append(temp_obs);	

#-----------------------------------------------------------
graph_vertices = [];
graph = [];

print dest;
print ([boundary[0].x, boundary[1].x], [boundary[0].y, boundary[2].y]);

# Add source to vertices
graph_vertices.append(source);
graph.append([]);
found = False;

# Run the algorithm ! ---------------------------------------------
# Find the path and save in a graph
while(found is False):
	
	if( len(graph_vertices)>=500 ):
		print "Iteration limit Reached";
		break;
	graph.append([]);
	
	potential_next_vertex = choose_target();
	if(potential_next_vertex.inside_polygon(obstacles) is True ):
		graph = graph[:-1];
		continue;
	
	vertex_index = potential_next_vertex.find_closest_point(graph_vertices);

	if(graph_vertices[vertex_index].equals(potential_next_vertex)):
		graph = graph[:-1];
		continue;
	
	# temp_vertex = find_point_on_line(graph_vertices[vertex_index], potential_next_vertex, step_size);
	temp_vertex = step_from_to(graph_vertices[vertex_index], potential_next_vertex, step_size);
	
	if(temp_vertex.inside_polygon(obstacles) is True ):
		
		graph = graph[:-1];
		continue;
	else:
		potential_next_vertex = temp_vertex;
	 	if(check_obstruction(obstacles, [potential_next_vertex, graph_vertices[vertex_index]]) is False):
	 		graph = graph[:-1];
			continue;
		if(check_obstruction(obstacles,[dest, potential_next_vertex]) is True):
			found = True; print("Reached..!!");
			graph_vertices.append(potential_next_vertex);
			n = len(graph_vertices)-1;

			graph[vertex_index].append(n);
			graph[n].append(vertex_index);

			graph.append([]);
			graph_vertices.append(dest);
			n = len(graph_vertices)-1;

			graph[n-1].append(n);
			graph[n].append(n-1);
			break;		
		else:		
			graph_vertices.append(potential_next_vertex);
			n = len(graph_vertices)-1;

			graph[vertex_index].append(n);
			graph[n].append(vertex_index);
runtime = time.time() - start_time
print('--- %s seconds ---' %runtime)
#euclid_dist = find_dist(point(0,0),point(0,10))
#print euclid_dist

poly_x =[]
poly_y =[]
for index, i in enumerate(obstacles):
	poly_x.append([p.x for p in i]);
	poly_y.append([p.y for p in i]);

	plt.fill( poly_x[index], poly_y[index], color="#512DA8");	

for index,i in enumerate(graph):
	for j in i:
		plt.plot([graph_vertices[index].x, graph_vertices[j].x], [graph_vertices[index].y, graph_vertices[j].y]);

# Find the path by running BFS on the graph
path = bfs(graph, 0, len(graph_vertices)-1);

x_array = [graph_vertices[i].x for i in path];
y_array = [graph_vertices[i].y for i in path];

# Calculate path length
path_length = 0
for count in range(1, len(x_array)):
	path_length += math.sqrt(math.pow(x_array[count]-x_array[count-1],2) + math.pow(y_array[count]-y_array[count-1],2))

print('Final path length = %s' %path_length)

plt.plot(x_array, y_array, color="#000000", linewidth=2);
plt.plot(graph_vertices[0].x, graph_vertices[0].y, marker="o")
plt.plot(graph_vertices[-1].x, graph_vertices[-1].y, marker="o")
plt.plot(dest.x, dest.y, marker="o")
plt.title('Step size = {0} m     Goal bias = {1}' .format(step_size, bias_prob))
plt.xlim(boundary[0].x,boundary[1].x)
plt.ylim(boundary[0].y,boundary[2].y)
plt.xlabel('x/m')
plt.ylabel('y/m')
plt.savefig('./figures/{0}.png' .format(start_time))
#plt.show();	


# Output into a file -----------------------------------------------
import csv

results = [
	start_time,
	path_length,
	runtime,
	step_size,
	bias_prob,
	altitude,
	source,
	dest,
	search_space
	]

headings = [
	'id',
	'path length',
	'runtime',
	'step size',
	'goal bias',
	'altitude',
	'origin',
	'destination',
	'search space'
	]

with open('results.csv','a') as f:
	writer = csv.writer(f)
	# writer.writerow(headings) # uncomment on first run
	writer.writerow(results)