import numpy as np
import networkx as nx
from os import path
import csv
import matplotlib.pyplot as plt
from constants import MULT, EPSILON, NMAXROBOTS, ALPHA
import misc

import logging
loglevel = logging.DEBUG
logging.basicConfig(format='[%(levelname)s] [%(name)s: %(funcName)s] %(message)s', level=loglevel)
log = logging.getLogger(__name__)


class PhysicalMap(object):
    """
    An abstraction of a physical map showing 2D `world`.
    """

    def __init__(self):
        self.graph = nx.Graph()
        self._initialized = False


    def InitFromFile(self, filename):
        """Return True if successfully initialized.

        """
        if not path.isfile(filename):
            log.info("File does not exists: {0}".format(filename))
            return False

        with open(filename, 'r') as f:
            content = csv.reader(f)
            content = list(content)

        # Retrieve data from file

        # The first two numbers in each line is the position of the
        # current node. The next pairs indicate the node's neighbors
        # (if any).

        # First iteration: add all nodes
        # Second iteration: add all (non-redundant) edges
        # TODO: implement a better way of extracting information from the file content
        nnodes = 0
        for line in content:
            npairs = len(line) / 2
            if npairs < 2:
                # Exclude disconnected nodes
                continue
            x, y = map(int, line[0:2])
            cur_pos = (x, y)
            self.graph.add_node(cur_pos, idx=nnodes)
            nnodes += 1
        # Now all nodes have been added to the graph
        for line in content:
            npairs = len(line) / 2
            x, y = map(int, line[0:2])
            cur_pos = (x, y)
            
            for i in xrange(1, npairs):
                x, y = map(int, line[2*i:2*i + 2])
                neigh_pos = (x, y)
                if not self.graph.has_edge(cur_pos, neigh_pos):
                    diff = np.subtract(cur_pos, neigh_pos) / MULT
                    d = np.sqrt(np.dot(diff, diff))
                    # undirected graph by default
                    self.graph.add_edge(cur_pos, neigh_pos, dist=d)
        self._initialized = True


    def FindShortestPathFromIDs(self, startid, goalid):
        startpos = list(set(pos for pos, data in self.graph.nodes_iter(data=True)
                            if data['idx'] == startid))[0]
        goalpos = list(set(pos for pos, data in self.graph.nodes_iter(data=True)
                          if data['idx'] == goalid))[0]
        return self.FindShortestPath(startpos, goalpos)


    def FindShortestPath(self, startpos, goalpos):
        return nx.shortest_path(self.graph, source=startpos, target=goalpos, weight='dist')


    def Plot(self, fignum='Physical Map', figsize=(15, 8), markersize=8, linewidth=2,
             color='0.5', showids=True):
        fig = plt.figure(num=fignum, figsize=figsize)
        ax = fig.add_subplot(111)
        plt.axis('equal')
        plt.tight_layout()

        # Graph nodes are stored as integers. When plotting, we need
        # to scale their positions accordingly.        
        for edge in self.graph.edges_iter():
            (x1, y1), (x2, y2) = edge # unpack the edge
            ax.plot([x1 / MULT, x2 / MULT], [y1 / MULT, y2 / MULT],
                    color=color, linewidth=linewidth)

        for node in self.graph.nodes_iter():
            x, y = node
            ax.plot(x / MULT, y / MULT, "o", markersize=markersize, color=color)

        if showids:
            annotationOffset = np.array([1.0, 1.0])
            for node in self.graph.nodes_iter(data=True):
                pos, data = node
                pos = np.asarray(pos) / MULT
                ax.annotate("{0}".format(data['idx']),
                            xy=pos, xycoords='data',
                            xytext=pos + annotationOffset, textcoords='data')
        return ax


class ConflictType:
    DEADLOCK = 0
    SHARED = 1
    JUNCTION = 2
    WORDS = {0: 'DEADLOCK', 1: 'SHARED', 2: 'JUNCTION'}
    
    
class ConflictZone(object):
    """A class for storing information regarding a conflict zone between a
    pair of robots. A conflict zone is represented as if the robots
    were points (i.e., their collision radii are zero).

    Attributes
    ----------
    rid1 : integer
        robot ID
    rid2 : integer
        robot ID
    intervals : dictionary
        A dictionary whose keys are robot IDs. The corresponding value
        for each key is the interval (a tuple) of path parameter for 
        that robot where this conflict occurs

    """
    def __init__(self, rid1, rid2, interval1, interval2, ctype):
        self.ids = sorted([rid1, rid2])
        self.intervals = dict()
        self.intervals[rid1] = interval1
        self.intervals[rid2] = interval2
        self.type = ctype


    def __repr__(self):
        report = "Conflict Zone information:\n"
        report += "Type {0} ({1})\n".format(self.type, ConflictType.WORDS[self.type])
        report += "Robot ID1: {0}; s = [{1}, {2}]\n".format(self.ids[0],
                                                            self.intervals[self.ids[0]][0],
                                                            self.intervals[self.ids[0]][1])
        report += "Robot ID2: {0}; s = [{1}, {2}]\n".format(self.ids[1],
                                                            self.intervals[self.ids[1]][0],
                                                            self.intervals[self.ids[1]][1])
        return report


class HighOrderDeadlock(object):
    def __init__(self, indices, intervals):
        self.indices = indices
        self.intervals = dict()
        for (idx, it) in zip(indices, intervals):
            self.intervals[idx] = it


    def __repr__(self):
        report = "High-Order Deadlock information:\n"
        for (key, val) in self.intervals.iteritems:
            report += "Robot {0}: s = [{1}, {2}]\n".format(key, val[0], val[1])
        return report

        
class CoordinationDiagram(object):
    """A class for coordination diagrams. This class is the configuration
    space in which planning occurs. The main purpose of this class is
    for collision checking in planning (for example, detecting
    deadlocks, etc.)

    """
    def __init__(self, physmap, params):
        self.physmap = physmap
        self.params = params # rrt.Parameters()
        self.robots = [] # list of SimpleRobots
        self.conflictzones = nx.Graph() # conflict zone information are kept as a graph

        self._a = 2*self.params.collisionRadius / np.sin(ALPHA)
        self._b = 2*self.params.collisionRadius / np.tan(ALPHA)
        

    def AddRobot(self, robot):
        """Add a new robot into this coordination diagram as well as update
        deadlock information.

        """
        robot.id = len(self.robots)
        self.robots.append(robot)
        self._UpdateConflictZoneInfo()


    def _UpdateConflictZoneInfo(self):
        """Update information on conflict zones by checking whether the newly
        added robot (self.robots[-1]) induces any conflict zone with
        any of the existing robots in self.robots[:-1].

        """
        if len(self.robots) < 2:
            return

        newrobot = self.robots[-1]
        # Iterate through all the previously existing robots
        for existingRobot in self.robots[:-1]:
            # sharedSegmentIndices and deadlockSegmentIndices are
            # lists of tuples. The first number of a tuple is the
            # segment index of the new robot where the deadlock or
            # shared segment occurs. The second number is that of the
            # other robot. For example, a tuple (x, y) means the
            # segment is newrobot.path[x] and existingRobot.path[y].
            sharedSegmentIndices = []
            deadlockSegmentIndices = []

            # junctions is a list of common nodes from paths of the
            # new robots and the existing robot.
            junctions = list(set(newrobot.path) & set(existingRobot.path))
            
            for i, segment in enumerate(newrobot.pathSegments):
                haveCommonSegment = False
                if segment in existingRobot.pathSegments:
                    haveCommonSegment = True
                    sharedSegmentIndices.append((i, existingRobot.pathSegments.index(segment)))
                        
                elif segment[::-1] in existingRobot.pathSegments:
                    haveCommonSegment = True
                    deadlockSegmentIndices.append((i, existingRobot.pathSegments.index(segment[::-1])))

                if haveCommonSegment:
                    # A node which is part of a shared segment or a
                    # deadlock is not a junction.
                    try:
                        junctions.remove(segment[0])
                    except:
                        pass
                    try:
                        junctions.remove(segment[1])
                    except:
                        pass

            if (len(sharedSegmentIndices) == 0 and
                len(deadlockSegmentIndices) == 0 and
                len(junctions) == 0):
                # There is no part in common whatsoever
                continue

            conflictzones = []

            if len(junctions) > 0:
                for node in junctions:
                    nodeIndex1 = newrobot.path.index(node)
                    s1 = newrobot.cumulatedLength[nodeIndex1]
                    interval1 = (s1, s1)

                    nodeIndex2 = existingRobot.path.index(node)
                    s2 = existingRobot.cumulatedLength[nodeIndex2]
                    interval2 = (s2, s2)

                    conflict = ConflictZone(newrobot.id, existingRobot.id, interval1, interval2,
                                            ConflictType.JUNCTION)
                    conflictzones.append(conflict)
            
            if len(sharedSegmentIndices) > 0:
                # shared1 and shared2 are lists of sublists. Each sublist
                # stores indices of path segments contained in the same
                # shared region.
                shared1 = [[sharedSegmentIndices[0][0]]] # for the new robot
                shared2 = [[sharedSegmentIndices[0][1]]] # for the existing robot
                for indices in sharedSegmentIndices[1:]:
                    if indices[0] == shared1[-1][-1] + 1:
                        # This shared segment continues from the previous one.
                        assert indices[1] == shared2[-1][-1] + 1
                        shared1[-1].append(indices[0])
                        shared2[-1].append(indices[1])
                    else:
                        # This is a new shared segment
                        shared1.append([indices[0]])
                        shared2.append([indices[1]])
                # Check soundness
                assert len(shared1) == len(shared2)

                for sublist1, sublist2 in zip(shared1, shared2):
                    sstart1 = newrobot.cumulatedLength[sublist1[0]]
                    send1 = newrobot.cumulatedLength[sublist1[-1] + 1]
                    interval1 = (sstart1, send1)

                    sstart2 = existingRobot.cumulatedLength[sublist2[0]]
                    send2 = existingRobot.cumulatedLength[sublist2[-1] + 1]
                    interval2 = (sstart2, send2)

                    conflict = ConflictZone(newrobot.id, existingRobot.id, interval1, interval2,
                                            ConflictType.SHARED)
                    conflictzones.append(conflict)

            if len(deadlockSegmentIndices) > 0:
                # deadlocks1 and deadlock2 are lists of sublists. Each
                # sublist stores indices of path segments contained in the
                # same deadlock.
                deadlocks1 = [[deadlockSegmentIndices[0][0]]] # for the new robot
                deadlocks2 = [[deadlockSegmentIndices[0][1]]] # for the existing robot
                for indices in deadlockSegmentIndices[1:]:
                    if indices[0] == deadlocks1[-1][-1] + 1:
                        # This deadlock continues from the previous one
                        assert indices[1] == deadlocks2[-1][0] - 1
                        deadlocks1[-1].append(indices[0])
                        deadlocks2[-1].insert(0, indices[1])
                    else:
                        # This is a new deadlock segment
                        deadlocks1.append([indices[0]])
                        deadlocks2.append([indices[1]])
                # Check soundness
                assert len(deadlocks1) == len(deadlocks2)

                for sublist1, sublist2 in zip(deadlocks1, deadlocks2):
                    sstart1 = newrobot.cumulatedLength[sublist1[0]]
                    send1 = newrobot.cumulatedLength[sublist1[-1] + 1]
                    interval1 = (sstart1, send1)

                    sstart2 = existingRobot.cumulatedLength[sublist2[0]]
                    send2 = existingRobot.cumulatedLength[sublist2[-1] + 1]
                    interval2 = (sstart2, send2)

                    conflict = ConflictZone(newrobot.id, existingRobot.id, interval1, interval2,
                                            ConflictType.DEADLOCK)
                    conflictzones.append(conflict)

            # Check soundness
            assert len(conflictzones) > 0

            # Each edge of the graph stores a list of conflict zones
            self.conflictzones.add_edge(newrobot.id, existingRobot.id,
                                        conflictzones=conflictzones)

        
    def CheckCollisionConfig(self, config, activeIndices, vnear=None):
        """Check if any pair of robots are in collision with physical obstacles (other
        robots) or virtual obstacles (deadlocks).

        """
        inCollision = False
        positions = [self.robots[i].Locate(config[i])[0] for i in activeIndices]

       
        for i in xrange(len(activeIndices)):
            index1 = activeIndices[i]
            for j in xrange(i + 1, len(activeIndices)):
                index2 = activeIndices[j]

                # TODO: for virtual collision checking, need to also handle
                # high-order virtual obstacles.
                
                # Check virtual collisions (deadlocks)
                if (index1 in self.conflictzones) and (index2 in self.conflictzones[index1]):
                    # Iterate through all the conflict zones there are between
                    # robot1 and robot2 and focus on only deadlocks.
                    for conflictzone in self.conflictzones[index1][index2]['conflictzones']:
                        if conflictzone.type != ConflictType.DEADLOCK:
                            continue

                        s1_start, s1_end = conflictzone.intervals[index1]
                        s2_start, s2_end = conflictzone.intervals[index2]

                        s1_start -= self._a
                        s2_start -= self._a
                        s1_end += self._b
                        s2_end += self._b

                        if ((s1_start <= config[index1] <= s1_end) and
                            (s2_start <= config[index2] <= s2_end)):
                            log.debug("    Config in deadlock: Robot {0} & Robot {1}".
                                      format(index1, index2))
                            if vnear is not None:
                                log.debug("    vnear.index = {0}".format(vnear.index))
                            inCollision = True
                            return inCollision
                        
                # Check physical collisions
                diff = positions[i] - positions[j]
                dist = np.sqrt(np.dot(diff, diff))
                if dist <= 2 * self.params.collisionRadius:
                    log.debug("    Config in collision: Robot {0} & Robot {1}".
                              format(index1, index2))
                    if vnear is not None:
                        log.debug("    vnear.index = {0}".format(vnear.index))
                    inCollision = True

                    if vnear is not None:
                        if index1 in vnear.waitingIndices or index2 in vnear.waitingIndices:
                            # If one of the colliding robot is a waiting
                            # robot, the other robot should also wait.
                            if index1 in vnear.waitingIndices:
                                vnear.waitingIndices.append(index2)
                            elif index2 in vnear.waitingIndices:
                                vnear.waitingIndices.append(index1)
                        else:
                            # Check if robot1 and robot2 are currently
                            # moving through a shared segment
                            for conflictzone in self.conflictzones[index1][index2]['conflictzones']:
                                if conflictzone.type == ConflictType.SHARED:
                                    s1_start, s1_end = conflictzone.intervals[index1]
                                    s2_start, s2_end = conflictzone.intervals[index2]
                                    rem1 = s1_end - config[index1]
                                    rem2 = s2_end - config[index2]
                                    if rem1 < rem2:
                                        vnear.waitingIndices.append(index2)
                                    else:
                                        vnear.waitingIndices.append(index1)
                        vnear.waitingIndices = sorted(list(set(vnear.waitingIndices)))
                    return inCollision
                
        return inCollision


    def CheckCollisionSegment(self, config1, config2, activeIndices, vnear=None):
        inCollision = False

        dist = misc.Distance(config1, config2, activeIndices)
        diff = config2.Subtract(config1, activeIndices)
        direction =  diff / misc.Norm(diff)
        steps = np.arange(0, dist, self.params.ccStepSize)
        for s in steps:
            inCollision = self.CheckCollisionConfig(config1 + direction * s, activeIndices, vnear)
            if inCollision:
                return inCollision
            
        return inCollision
