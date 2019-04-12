import numpy as np
import argparse
from xml.dom import minidom
from svg.path import parse_path
from os.path import splitext
# import multirobot as mr
import matplotlib.pyplot as plt
from constants import MULT
"""Conversion of maps in SVG into TXT.
The original code was from Hung.

Convention: Node positions will be converted to integers. A coordinate
is multiplied by MULT and then floored to eliminate remaining decimal
places. The default value of MULT is 100.

When MULT = 100, a value x = 123.4567 is converted to x_ = 12345.

"""


def Float2Int(x):
    return int(np.floor(MULT*x))


if __name__ == "__main__":
    # Parse the arguments
    parser = argparse.ArgumentParser(description="Convert a map in SVG into TXT.")
    parser.add_argument("--input", "-i", dest="input_filename", type=str,
                        help="SVG file to be converted to TXT")
    parser.add_argument("--output", "-o", dest="output_filename", type=str,
                        default=None,
                        help="The output text will be saved in this file")
    args = parser.parse_args()
    if not args.input_filename.lower().endswith('.svg'):
        raise ValueError("The given file name is not in SVG extension.")

    # Generate the description file from SVG
    svg_dom = minidom.parse(args.input_filename)

    # Extract nodes
    node_strings = [[path.getAttribute('cx'), path.getAttribute('cy')]
                 for path in svg_dom.getElementsByTagName('circle')]
    node_positions = []
    for s in node_strings:
        # The current version of the planner needs the positions to be integers
        node_positions.append((Float2Int(float(s[0])), Float2Int(float(s[1]))))

    # Extract paths
    path_strings = [path.getAttribute('d') for path in svg_dom.getElementsByTagName('path')]
    lines = []
    for s in path_strings:
        path_data = parse_path(s)
        for line in path_data:
            start = (Float2Int(line.start.real), Float2Int(line.start.imag))
            end = (Float2Int(line.end.real), Float2Int(line.end.imag))
            lines.append((start, end))

    # Match each path with a pair of nodes
    dist_threshold = 2
    line_ids = []
    for line in lines:
        line_id = []
        for point in line:
            point_id = None # assume no matching by default
            min_dist = np.inf
            for i, node in enumerate(node_positions):
                diff = np.subtract(node, point) / MULT
                dist = np.dot(diff, diff)
                if dist < min(dist_threshold, min_dist):
                    min_dist = dist
                    point_id = i
            if point_id is None:
                import IPython; IPython.embed(header='id is none')
            line_id.append(point_id)
        line_ids.append(line_id)

    # Obtain node adjacency information
    adj = {i: [] for i in xrange(len(node_positions))}
    for start_id, end_id in line_ids:
        if end_id not in adj[start_id]:
            adj[start_id].append(end_id)
        if start_id not in adj[end_id]:
            adj[end_id].append(start_id)

    datastring = ''
    linesep = ''
    sep = ','
    for node, neighbors in adj.items():
        datastring += linesep
        linesep = '\n'
        datastring += '{0},{1}'.format(node_positions[node][0], node_positions[node][1])
        for neighbor in neighbors:
            datastring += sep
            datastring += '{0},{1}'.format(node_positions[neighbor][0], node_positions[neighbor][1])

    if (args.output_filename is None) or (not args.output_filename.lower().endswith('.txt')):
        fname, fext = splitext(args.input_filename)
        ofilename = fname + '.txt'
    else:
        ofilename = args.output_filename
    with open(ofilename, 'w') as f:
        f.write(datastring)
    print "Data is successfully written to {0}".format(ofilename)
