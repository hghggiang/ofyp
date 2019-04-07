import numpy as np
import networkx as nx
from os import path
import csv
import matplotlib.pyplot as plt
from constants import MULT, EPSILON, NMAXROBOTS, ALPHA
import misc
from misc import Config, find_all_cycles
import logging
loglevel = logging.DEBUG
logging.basicConfig\
    (format='[%(levelname)s] [%(name)s: %(funcName)s] %(message)s', level=loglevel)
log = logging.getLogger(__name__)

import time
from interval import interval as iv
from box import Box
from itertools import product, combinations

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
#        startpos = list(set(pos for pos, data in self.graph.nodes_iter(data=True)
#                            if data['idx'] == startid))[0]
#        goalpos = list(set(pos for pos, data in self.graph.nodes_iter(data=True)
#                          if data['idx'] == goalid))[0]
#		 The above are original codes. Below is modified by Giang on Feb 2019 for newer version of Python 3.
        startpos = list(set(pos for pos, data in self.graph.nodes(data=True)
                            if data['idx'] == startid))[0]
        goalpos = list(set(pos for pos, data in self.graph.nodes(data=True)
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
        for edge in self.graph.edges():
            (x1, y1), (x2, y2) = edge # unpack the edge
            ax.plot([x1 / MULT, x2 / MULT], [y1 / MULT, y2 / MULT],
                    color=color, linewidth=linewidth)

        for node in self.graph.nodes():
            x, y = node
            ax.plot(x / MULT, y / MULT, "o", markersize=markersize, color=color)

        if showids:
            annotationOffset = np.array([1.0, 1.0])
            for node in self.graph.nodes(data=True):
                pos, data = node
                pos = np.asarray(pos) / MULT
                ax.annotate("{0}".format(data['idx']),
                            xy=pos, xycoords='data',
                            xytext=pos + annotationOffset, textcoords='data')
        return ax


class ConflictType:
    DEADLOCK = 0
    SHARED   = 1
    JUNCTION = 2
    WORDS    = {0: 'DEADLOCK', 1: 'SHARED', 2: 'JUNCTION'}


class Conflict(object):
    """A class for storing information regarding a conflict,
    i.e., a conflict involving two robots.

    Note
    ---- 
    Currently, we assume zero collision radius.

    Attributes
    ----------
    intervals : dictionary
        A dictionary whose keys are robot IDs. The corresponding value
        for each key is the interval (a tuple) of path parameter for 
        that robot where this conflict occurs

    """
    def __init__(self, indices, intervals, ctype):
        self.indices = indices[:]
        self.intervals = dict()
        for index, interval in zip(indices, intervals):
            self.intervals[index] = interval
        self.type = ctype


    def __repr__(self):
        report = "Conflict information:\n"
        report += "Type {0} ({1})\n".format(self.type, ConflictType.WORDS[self.type])
        for (index, interval) in self.intervals.iteritems():
            report += "Robot {0}: s = [{1}, {2}]\n".format(index, interval[0], interval[1])
        return report


class CoordinationDiagram(object):
    """
    """
    def __init__(self, physmap, params):
        self.physmap = physmap
        self.params = params # instance of rrt.Parameters()
        self.robots = []     # list of SimpleRobot

        # parameters for collision checking
        self._a = 2*self.params.collisionRadius / np.sin(ALPHA)
        # self._b = 2*self.params.collisionRadius / np.tan(ALPHA)
        self._b = self._a # safer although more constrained

        # FOR SAFETY
        self._a *= 1.1
        self._b *= 1.1

        self.conflicts = []  # list of all conflicts in the given physical map
        self.conflictgraph = nx.Graph()
        self.conflictgraph.add_nodes_from(self.physmap.graph.nodes())
        # Current version: edges of conflictgraph are edges with deadlocks only.

        self._initialized = False # toggled to True when calling complex conflict detection

        
    def AddRobot(self, robot):
        """Add a new robot into the coordination diagram and also update
        conflict information.

        """
        robot.id = len(self.robots)
        self.robots.append(robot)
        self._UpdateConflictInfo()


    def _UpdateConflictInfo(self):
        """Detect and store information of elementary conflicts between the newly added
        robot and the existing ones.

        """
        if len(self.robots) < 2:
            return

        newRobot = self.robots[-1]
        # Iterate through the list of all existing robots
        for existingRobot in self.robots[:-1]:
            # Now checking conflicts between newRobot and existingRobot.

            # sharedSegmentIndices and deadlockSegmentIndices are lists of
            # tuples. The first number of a tuple is the segment index of the
            # new robot where the conflict occurs. The second number is that of
            # the other robot.

            # For example, a tuple (x, y) indicates that the segment is
            # newRobot.path[x] and existingRobot.path[y].
            
            sharedSegmentIndices = []
            deadlockSegmentIndices = []

            # junctions is a list of common nodes from paths of the two robots.
            junctions = list(set(newRobot.path) & set(existingRobot.path))

            # Iterate through the paths to detect segments that are in common.
            for (i, segment) in enumerate(newRobot.pathSegments):
                haveCommonSegment = False
                
                if segment in existingRobot.pathSegments:
                    # Shared segment: both robots move in the same direction.
                    haveCommonSegment = True
                    j = existingRobot.pathSegments.index(segment)
                    sharedSegmentIndices.append((i, j))

                elif segment[::-1] in existingRobot.pathSegments:
                    # Deadlock: two robots move in the opposite directions.
                    haveCommonSegment = True
                    j = existingRobot.pathSegments.index(segment[::-1])
                    deadlockSegmentIndices.append((i, j))

                if haveCommonSegment:
                    # Remove nodes that are part of the common segment from junctions.
                    try:
                        junctions.remove(segment[0])
                    except:
                        pass
                    try:
                        junctions.remove(segment[1])
                    except:
                        pass

            # Process all the detected common segments
            if ((len(sharedSegmentIndices) == 0) and
                (len(deadlockSegmentIndices) == 0) and
                (len(junctions) == 0)):
                # Two paths do not intersect.
                continue

            # Now investigate the list of common segments. Store conflicts
            # as instances of Conflict (see above).
            if len(junctions) > 0:
                for node in junctions:
                    nodeidx1 = newRobot.path.index(node)
                    s1 = newRobot.cumulatedLength[nodeidx1]
                    interval1 = (s1 - self._a, s1)
                    
                    nodeidx2 = existingRobot.path.index(node)
                    s2 = existingRobot.cumulatedLength[nodeidx2]
                    interval2 = (s2 - self._a, s2)
                    
                    conflict = Conflict([newRobot.id, existingRobot.id],
                                        [interval1, interval2],
                                        ConflictType.JUNCTION)
                    self.conflicts.append(conflict)
                    
            if len(sharedSegmentIndices) > 0:
                # shared1 and shared2 are lists of sublists. Each sublist stores
                # indices of path segments contained in the same shared region.
                shared1 = [[sharedSegmentIndices[0][0]]] # for newRobot
                shared2 = [[sharedSegmentIndices[0][1]]] # for existingRobot
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
                for (sublist1, sublist2) in zip(shared1, shared2):
                    s1_start  = newRobot.cumulatedLength[sublist1[0]]
                    s1_end    = newRobot.cumulatedLength[sublist1[-1] + 1]
                    # interval1 = (s1_start, s1_end)
                    interval1 = (s1_start - self._a, s1_end + self._a)

                    s2_start  = existingRobot.cumulatedLength[sublist2[0]]
                    s2_end    = existingRobot.cumulatedLength[sublist2[-1] + 1]
                    # interval2 = (s2_start, s2_end)
                    interval2 = (s2_start - self._a, s2_end + self._a)

                    conflict = Conflict([newRobot.id, existingRobot.id],
                                        [interval1, interval2],
                                        ConflictType.SHARED)
                    self.conflicts.append(conflict)

                    # conflictid is a tuple of robot indices and the index of
                    # this conflict (as stored in self.conflicts).
                    conflictid = (conflict.indices, len(self.conflicts) - 1)

            if len(deadlockSegmentIndices) > 0:
                # deadlocks1 and deadlocks2 are lists of sublists. Each sublist
                # stores indices of path segments contained in the same
                # deadlock.
                deadlocks1 = [[deadlockSegmentIndices[0][0]]] # for newRobot
                deadlocks2 = [[deadlockSegmentIndices[0][1]]] # for existingRobot
                for indices in deadlockSegmentIndices[1:]:
                    if indices[0] == deadlocks1[-1][-1] + 1:
                        # This deadlock segment continues from the previous one
                        assert indices[1] == deadlocks2[-1][0] - 1
                        deadlocks1[-1].append(indices[0])
                        deadlocks2[-1].insert(0, indices[1])
                    else:
                        # This is a new deadlock
                        deadlocks1.append([indices[0]])
                        deadlocks2.append([indices[1]])
                # Check soundness
                assert len(deadlocks1) == len(deadlocks2)
                for (sublist1, sublist2) in zip(deadlocks1, deadlocks2):
                    s1_start  = newRobot.cumulatedLength[sublist1[0]]
                    s1_end    = newRobot.cumulatedLength[sublist1[-1] + 1]
                    # interval1 = (s1_start, s1_end)
                    interval1 = (s1_start - self._a, s1_end + self._b)

                    s2_start  = existingRobot.cumulatedLength[sublist2[0]]
                    s2_end    = existingRobot.cumulatedLength[sublist2[-1] + 1]
                    # interval2 = (s2_start, s2_end)
                    interval2 = (s2_start - self._a, s2_end + self._b)

                    conflict = Conflict([newRobot.id, existingRobot.id],
                                        [interval1, interval2],
                                        ConflictType.DEADLOCK)
                    self.conflicts.append(conflict)
                    
                    # conflictid is a tuple of robot indices and the index of
                    # this conflict (as stored in self.conflicts).
                    conflictid = (conflict.indices, len(self.conflicts) - 1)
                    # import IPython; IPython.embed(header="in deadlock")
                    for segmentIndex in sublist1:
                        node0, node1 = newRobot.pathSegments[segmentIndex]
                        try:
                            # There are already conflicts along this edge
                            self.conflictgraph[node0][node1]['conflicts'].append(conflictid)
                        except:
                            # Create a new edge in conflictgraph
                            self.conflictgraph.add_edge(node0, node1, conflicts=[conflictid])


    def DetectComplexConflicts(self):
        if len(self.robots) < 1:
            log.info("nrobots = 0: no robot added to the map.")
            return
        if len(self.robots) < 3:
            log.info("nrobots < 3: no complex conflict.")
            return

        tstart = time.time()

        # First, detect deadlock conneted component subgraph from the physical map
        C = [] # list of subgraphs. Each subgraph is a cluster of deadlocks
        for subgraph in nx.connected_component_subgraphs(self.conflictgraph):
            if len(subgraph.nodes()) == 1:
                continue
            C.append(subgraph)

        print "There are {0} subgraphs".format(len(C))

        # Examine each cluster more closely. Now we iterate through the list of
        # nodes then see if deadlocks adjacent to this node have potential of
        # creating complex deadlocks. To check the potential to create a complex
        # deadlock, we check if the number of (elementary) deadlock is greater
        # or equal to the number of aircraft (pigoen hole principle).
        for (isubgraph, subgraph) in enumerate(C):
            print "subgraph {0}".format(isubgraph)
            examined_deadlockIndices = []
            
            for node in subgraph.nodes():
                deadlockIndices = [] # store indices of Conflicts adjacent to this node
                aircraft = [] # store indices of relevant aircraft

                for (neighborNode, conflicts) in subgraph[node].iteritems():
                    for conflict in conflicts['conflicts']:
                        # conflict is a tuple of robotindices and the conflict index.
                        aircraft += conflict[0]
                        deadlockIndices.append(conflict[1])
                deadlockIndices.sort()

                deadlockIndicesSet = set(deadlockIndices)
                # Check if this set of indices has already been examined
                if any([deadlockIndicesSet.issubset(deadlockCluster)
                        for deadlockCluster in examined_deadlockIndices]):
                    continue
                else:
                    examined_deadlockIndices.append(deadlockIndicesSet)
                    
                aircraft = set(aircraft)
                if len(aircraft) > len(deadlockIndicesSet):
                    # This set of deadlocks has no potential of creating a
                    # complex deadlock.
                    continue

                boxes = dict() # key: deadlock index, values: the box
                _graph = nx.Graph() # for detecting deadlock cycles
                for deadlockIndex in deadlockIndicesSet:
                    deadlock = self.conflicts[deadlockIndex]
                    intervals = dict() # a dictionary to be input when creating a box
                    for key, val in deadlock.intervals.iteritems():
                        intervals[key] = iv(val)
                    box = Box(intervals)
                    boxes[deadlockIndex] = box

                    _graph.add_edge(*box.indices, deadlockIndex=deadlockIndex)

                cycles = find_all_cycles(_graph)
                print "    There are {0} cycles".format(len(cycles))

                for (icycle, cycle) in enumerate(cycles):
                    # Each cycle is an ordered sequence of nodes. (Each node is
                    # actually an aircraft index.)
                    print "        cycle {0} = {1}".format(icycle, cycle)

                    # deadlockCycleIndices keeps indices of deadlocks in this cycle.
                    deadlockCycleIndices = [_graph[cycle[i]][cycle[i + 1]]['deadlockIndex']
                                            for i in xrange(len(cycle) - 1)]
                    deadlockCycleIndices.append(_graph[cycle[-1]][cycle[0]]['deadlockIndex'])
                    deadlockCycleIndicesSet = set(deadlockCycleIndices)

                    # Check if this set of deadlocks has already been
                    # examined. Note the index [:-1] of examined_deadlockIndices.
                    if any([deadlockCycleIndicesSet.issubset(deadlockCluster)
                            for deadlockCluster in examined_deadlockIndices[:-1]]):
                        print "          duplicated"
                        continue

                    # Retrieve the related boxes
                    _boxes = [boxes[deadlockIndex] for deadlockIndex in deadlockCycleIndices]
                    _aircraft = set(cycle)

                    print "        examining {0} cases".format\
                        (len(list(product(*[b.indices for b in _boxes]))))
                    
                    for indices in product(*[b.indices for b in _boxes]):
                        if not _aircraft.issubset(set(indices)):
                            continue

                        shadows = []
                        for index, box in zip(indices, _boxes):
                            shadows.append(box.GenerateShadow(index))
                            if len(shadows) == 1:
                                intersection = shadows[0]
                            else:
                                intersection = intersection.IntersectOpen(shadows[-1])
                        if intersection.IsEmpty():
                            continue
                        complexdeadlock = Conflict(intersection.intervals.keys(),
                                                   [[i[0][0], i[0][1]]
                                                    for i in intersection.intervals.values()],
                                                   ConflictType.DEADLOCK)
                        self.conflicts.append(complexdeadlock)
                        print "    complex deadlock generated at node {0}".format(node)
                        print "        complex deadlock index = {0}".format(len(self.conflicts) - 1)
                   

                # # Iterate through all possible cases of shadow
                # # intersections. For each box, choose one of the two possible
                # # shadows.
                # hasComplexDeadlock = False
                # indiceslist = []
                # print "    node {0}: {1} boxes; {2} iterations".format\
                #     (node, len(boxes), len(list(product(*[b.indices for b in boxes]))))
                # # if len(boxes) > 10:
                # #     import IPython; IPython.embed(header="len(boxes) > 10")

                # for indices in product(*[b.indices for b in boxes]):
                #     if not aircraft.issubset(set(indices)):
                #         # Even though the resulting intersection of this case is
                #         # not empty, it cannot block a configuration in every
                #         # direction.
                #         continue

                #     shadows = []
                #     for index, box in zip(indices, boxes):
                #         # Generate a shadow from a box with the specified direction
                #         shadows.append(box.GenerateShadow(index))

                #     # Iterate through the list of groups of len(aircraft)
                #     # shadows and see which group can create a deadlock.
                #     S = zip(shadows, indices)
                #     for shadowsList in combinations(S, len(aircraft)):
                #         shadowIndices = [s[1] for s in shadowsList]
                #         if set(shadowIndices) != aircraft:
                #             continue

                #         intersection = shadowsList[0][0]
                #         for s in shadowsList[1:]:
                #             intersection = intersection.IntersectOpen(s[0])
                #         if intersection.IsEmpty():
                #             # This intersection is void.
                #             continue

                #         complexdeadlock = Conflict(intersection.intervals.keys(),
                #                                    [[i[0][0], i[0][1]]
                #                                     for i in intersection.intervals.values()],
                #                                    ConflictType.DEADLOCK)
                #         self.conflicts.append(complexdeadlock)
                #         print "    complex deadlock generated at node {0}".format(node)
                #         print "        complex deadlock index = {0}".format(len(self.conflicts) - 1)

        tend = time.time()
        self._complexConflictDetectionTime = tend - tstart
                        

    def CheckCollisionConfig(self, config, activeIndices, vnear=None):
        """Check for both virtual and actual collisions.
        """
        inCollision = False
        positions = [self.robots[i].Locate(config[i])[0] for i in activeIndices]

        # Check for virtual collisions (trapped in deadlocks).
        a = self._a
        b = self._b
        for (iconflict, conflict) in enumerate(self.conflicts):
            if conflict.type != ConflictType.DEADLOCK:
                continue

            if not set(conflict.indices).issubset(set(activeIndices)):
                continue
            indices = conflict.indices

            svect = Config([config[rindex] for rindex in indices])
            # svect_start = Config([conflict.intervals[rindex][0] - a
            #                       for rindex in indices])
            # svect_end = Config([conflict.intervals[rindex][1] + b
            #                     for rindex in indices])
            svect_start = Config([conflict.intervals[rindex][0] for rindex in indices])
            svect_end = Config([conflict.intervals[rindex][1] for rindex in indices])

            if svect.Dominates(svect_start) and svect_end.Dominates(svect):
                msg = "    Config in deadlock {0}: Robot {1}".format\
                      (iconflict, indices[0])
                for index in indices[1:]:
                    msg += " & Robot {0}".format(index)
                log.debug(msg)
                inCollision = True
                if vnear is not None:
                    log.debug("    vnear.index = {0}".format(vnear.index))
                return inCollision

        # Check for actual collisions
        for i in xrange(len(activeIndices)):
            index1 = activeIndices[i]
            for j in xrange(i + 1, len(activeIndices)):
                index2 = activeIndices[j]

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
                            for conflict in self.conflicts:
                                # if conflict.type != ConflictType.SHARED:
                                #     continue
                                
                                # Consider both shared segments and junctions. 
                                if conflict.type == ConflictType.DEADLOCK:
                                    continue
                                if (not (index1 in conflict.indices and
                                         index2 in conflict.indices)):
                                    continue
                                s1_start, s1_end = conflict.intervals[index1]
                                s2_start, s2_end = conflict.intervals[index2]
                                s1 = config[index1]
                                s2 = config[index2]
                                rem1 = s1_end - s1
                                rem2 = s2_end - s2

                                # The robot that is behind the other shall wait.
                                if rem1 > rem2:
                                    vnear.waitingIndices.append(index1)
                                else:
                                    vnear.waitingIndices.append(index2)
                                                                        
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
            inCollision = self.CheckCollisionConfig(config1 + direction * s,
                                                    activeIndices, vnear)
            if inCollision:
                return inCollision
            
        return inCollision


    def CheckCollisionConfig2(self, config, activeIndices, vnear=None):
        """Check if any pair of robots are in collision with ONLY physical
        obstacles.

        """
        inCollision = False
        positions = [self.robots[i].Locate(config[i])[0] for i in activeIndices]

       
        for i in xrange(len(activeIndices)):
            index1 = activeIndices[i]
            for j in xrange(i + 1, len(activeIndices)):
                index2 = activeIndices[j]

                # Check physical collisions
                diff = positions[i] - positions[j]
                dist = np.sqrt(np.dot(diff, diff))
                if dist <= 2 * self.params.collisionRadius:
                    log.debug("    Config in collision: Robot {0} & Robot {1}".
                              format(index1, index2))
                    if vnear is not None:
                        log.debug("    vnear.index = {0}".format(vnear.index))
                    inCollision = True

                    return inCollision
                
        return inCollision


    def CheckCollisionSegment2(self, config1, config2, activeIndices, vnear=None):
        inCollision = False

        dist = misc.Distance(config1, config2, activeIndices)
        diff = config2.Subtract(config1, activeIndices)
        direction =  diff / misc.Norm(diff)
        steps = np.arange(0, dist, self.params.ccStepSize)
        for s in steps:
            inCollision = self.CheckCollisionConfig2(config1 + direction * s,
                                                     activeIndices, vnear)
            if inCollision:
                return inCollision
            
        return inCollision
