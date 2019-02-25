#!/usr/bin/env python

# rrt.py
# This program generates a simple rapidly
# exploring random tree (RRT) in a rectangular region.
#
# Written by Steve LaValle
# May 2011

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
import networkx as nx 

#constants
XDIM = 300
YDIM = 200
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 5000

G = nx.grid_2d_graph(300,200)
# pos = dict((n,n) for n in G.nodes())

def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

def main():
    #initialize and prepare screen
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRT      S. LaValle    May 2011')
    white = 255, 240, 200
    black = 20, 20, 40
    screen.fill(black)

    nodes = []

#    nodes.append((XDIM/2.0,YDIM/2.0)) # Start in the center
    nodes.append((0.0,0.0)) # Start in the corner

#    print 'A random node:', random.choice(list(G.nodes))

    for i in range(NUMNODES):
#	rand = random.random()*XDIM, random.random()*YDIM
        rand = random.choice(list(G.nodes))
	nn = nodes[0]
        for p in nodes:
	   if dist(p,rand) < dist(nn,rand):
	      nn = p
	newnode = step_from_to(nn,rand)
 	nodes.append(newnode)
	pygame.draw.line(screen,white,nn,newnode)
        pygame.display.update()
#        print i, "    ", nodes
#        print rand

        for e in pygame.event.get():
	   if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
	      sys.exit("Leaving because you requested it.")
	

# if python says run, then we should run
if __name__ == '__main__':
    main()



