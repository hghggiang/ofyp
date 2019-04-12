import sys
sys.path.append('../src')

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
rng = random.SystemRandom()

import rrt2 as rrt
import maps2 as maps
import misc
from constants import MULT, NMAXROBOTS
_hsv = plt.get_cmap('hsv')

mapfilename = "../src/network_connected.txt"
pm = maps.PhysicalMap()
pm.InitFromFile(mapfilename)

nrobots = 10
params = rrt.Parameters(backoffDistance=20.,
                        ccStepSize=1.,
                        collisionRadius=1,
                        preventionDistance=10,
                        expansionCoeff=1.2,
                        nn=1,
                        rrtStepSize=10.,
                        useVirtualBoxes=True)
cd = maps.CoordinationDiagram(pm, params)

passed = False
ids = nx.get_node_attributes(pm.graph, 'idx')
while not passed:
    allnodeids = []
    allnodes = []
    cd_temp = maps.CoordinationDiagram(pm, params) # temp map for collision checking
    for irobot in xrange(nrobots):
        node1 = rng.choice(ids.keys())
        # We want to select an unselected node with degree 1
        while (ids[node1] in allnodeids) or (len(pm.graph[node1]) > 1):
            node1 = rng.choice(ids.keys())
        allnodes.append(node1)
        nodeid1 = ids[node1]
        allnodeids.append(nodeid1)

        node2 = rng.choice(ids.keys())
        # We want to select an unselected node with degree 1
        while (ids[node2] in allnodeids) or (len(pm.graph[node2]) > 1):
            node2 = rng.choice(ids.keys())
        allnodes.append(node2)
        nodeid2 = ids[node2]
        allnodeids.append(nodeid2)

        path = pm.FindShortestPathFromIDs(nodeid1, nodeid2)
        robot = misc.SimpleRobot(path, pm)
        cd.AddRobot(robot)

        robot_temp1 = misc.SimpleRobot([node1], pm)
        robot_temp2 = misc.SimpleRobot([node2], pm)
        cd_temp.AddRobot(robot_temp1)
        cd_temp.AddRobot(robot_temp2)
        
    if ((not cd.CheckCollisionConfig(rrt.Config([0.]*nrobots), range(nrobots))) and
        (not cd_temp.CheckCollisionConfig(rrt.Config([0.]*(2*nrobots)), range(2*nrobots)))):
        passed = True
    else:
        cd.robots = []
        cd.conflictzones = nx.Graph()

        cd_temp.robots = []
        cd_temp.conflictzones = nx.Graph()

cd.DetectComplexConflicts()

for conflict in cd.conflicts:
    if len(conflict.indices) > 4:
        import IPython; IPython.embed(header="X-conflict; X > 4")

planner = rrt.RRTPlanner(cd)
status = planner.Plan(600)

assert(status)


def PlayBack(traj, cd, repeat=True):
    """Convenient function to do playback.

    Usually it is necessary to run `plt.pause(0.01)` after calling
    this function for the animation to start. This is because of the
    interactive flag.

    Args:
        traj: A Trajectory, store trajectory of all robots.
        cd: A CoordinationDiagram.
        repeat: A Boolean, True if animation is to be repeat.
    """
    ax = cd.physmap.Plot(showids=False)
    handles = []
    for robot in cd.robots:
        # Visualize the path of this robot
        robot.PlotPath(ax)

        # Generate a plot handle for this robot
        # color = _hsv(float(robot.id) / NMAXROBOTS)
        color = plt.cm.Set1(float(robot.id) / NMAXROBOTS)
        handles.append(ax.plot([], [], "o", markersize=12, color=color, zorder=100)[0])

    def PlaceRobots(t):
        svect = traj.Eval(t)
        for i, s in enumerate(svect):
            pos, _, _ = cd.robots[i].Locate(s)
            handles[i].set_data(pos[0], pos[1])
        return handles

    T = np.arange(0, traj.duration, 50)

    ani = animation.FuncAnimation(ax.get_figure(), PlaceRobots,
                                  frames=T,
                                  interval=10,
                                  repeat=repeat)
    ani.save('animation(2).mp4', fps=10)
    
    return ani

def PlotNetwork(cd):
    ax = cd.physmap.Plot(showids=False)
    for robot in cd.robots:
        robot.PlotPath(ax)
    return ax

# ax = PlotNetwork(cd)
        
traj = planner.ExtractTrajectory()
ani = PlayBack(traj, cd)

plt.show()

# import IPython
# if IPython.get_ipython() is None:
#     IPython.embed()

"""

Script for plotting robot positions

# rindices is a list of robot indices which we want to plot

ax = cd.physmap.Plot(showids=False)
for irobot in rindices:
    cd.robots[irobot].PlotPath(ax)
handles = []

# config is Config that we want to plot robot positions

for irobot in rindices:
    handles.append(cd.robots[irobot].Plot(config[irobot], ax))

"""
