# Find a path avoiding obstacles using modified RRT, including path smoothing
# Authors -- Shikhar Dev Gupta, Hoang Huong Giang

from random import *
from helpers.graph import *
from helpers.geometry import *
import matplotlib.pyplot as plt
import time
import numpy as np 

plt.rcParams.update({'font.size': 7})


# input parameters
step_size = 50;
bias_prob = 0.05; # probability that the goal is chosen as node
altitude = 65.71 # manual input: 15th = 51.78, 16th = 65.71, 17th = 82.59, 18th = 100.92, 19th = 132.51, 20th = 258.49

start_time = time.time()

# Check for empty lines
file_handler = open("obstacles_16","r");
raw_data = file_handler.read();
raw_data = raw_data.split("\n");
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

# Extract source and dest
temp = parse_input_line(raw_data[len(raw_data)-1]);
source = point(temp[0][0], temp[0][1]);
dest = point(temp[1][0], temp[1][1]);

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
	
	if( len(graph_vertices)>=1500 ):
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
array = np.array(list(zip(x_array,y_array))).tolist()

# Path smoothing --------------------------------------------------
maxIter = 1000
smoothedPath = PathSmoothing(array, maxIter, obstacles)

runtime_smooth = time.time() - start_time
print('--- %s seconds ---' %runtime_smooth)

path_length = GetPathLength(array)
print('Final path length = %s' %path_length)
smoothedPath_length = GetPathLength(smoothedPath)
print('Smoothed path length = %s' %smoothedPath_length)
print('id: %s' %start_time)

# Plot results -----------------------------------------------------
plt.plot(x_array, y_array, color="#000000", linewidth=2);
plt.plot([x for (x, y) in smoothedPath], [
            y for (x, y) in smoothedPath], '-b', linewidth=2);
plt.plot(graph_vertices[0].x, graph_vertices[0].y, marker="o")
plt.plot(graph_vertices[-1].x, graph_vertices[-1].y, marker="o")
plt.title('Step size = {0} m     Goal bias = {1}' .format(step_size, bias_prob))
# plt.xlim(boundary[0].x,boundary[1].x)
# plt.ylim(boundary[0].y,boundary[2].y)
plt.savefig('./figures/{0}.png' .format(start_time))
plt.show()

# Output into a file -----------------------------------------------
import csv

results = [
	start_time,
	path_length,
	smoothedPath_length,
	runtime,
	runtime_smooth,
	step_size,
	bias_prob,
	altitude,
	source.x,
	source.y,
	dest.x,
	dest.y
	]

headings = [
	'id',
	'path length',
	'smoothed path length',
	'runtime',
	'runtime with smoothing',
	'step size',
	'goal bias',
	'altitude',
	'origin_x',
	'origin_y',
	'destination_x',
	'destination_y'
	]

with open('results.csv','a') as f:
	writer = csv.writer(f)
	# writer.writerow(headings) # uncomment on first run
	writer.writerow(results)