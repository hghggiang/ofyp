
# coding: utf-8

# Original Rapidly-explorting Random Tree (RRT) algorithm for practice
# 
# March 2019

# In[4]:


import sys, random, math, pygame


# In[5]:


from pygame.locals import *


# In[6]:


from math import sqrt,cos,sin,atan2


# In[7]:


#constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM,YDIM]
EPSILON = 7.0
NUMNODES = 500


# In[8]:


def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]))


# In[9]:


def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1], p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)


# In[10]:


def main():
    #initialise and prepare screen
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRT     based on S. LaValle     G. Hoang     March 2019')
    white = 255,240,200
    black = 20,20,40
    screen.fill(black)
    
    nodes = []
    
    nodes.append((XDIM/2.0,YDIM/2.0)) #start in the center
#    nodes.append((0.0,0.0)) #start in the corner

    for i in range(NUMNODES):
        rand = random.random()*640.0, random.random()*480.0
        nn = nodes[0]
        for p in nodes:
            if dist(p,rand) < dist(nn,rand):
                nn = p
        newnode = step_from_to(nn,rand)
        nodes.append(newnode)
        pygame.draw.line(screen,white,nn,newnode)
        pygame.display.update()
        #print i, "     ", nodes
        
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")


# In[11]:


# if python says run, then we should run
if __name__ == '__main__':
    main()

