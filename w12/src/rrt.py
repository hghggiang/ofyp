import numpy as np
import time
from constants import EPSILON, RANDSEED
from misc import Norm, Distance, Distance2
from maps import ConflictType
# Use numpy's random number generator instead of random.SystemRandom()
# for reproducibility (better for debugging).
np.random.seed(RANDSEED)

from TOPP import Trajectory

import logging
loglevel = logging.DEBUG
logging.basicConfig(format='[%(levelname)s] [%(name)s: %(funcName)s] %(message)s', level=loglevel)
log = logging.getLogger(__name__)

TRAPPED = 0
ADVANCED = 1
REACHED = 2

_NNN = 0
_NNNTIME = 0.0

class Config(list):
    """List with arithmatic operators.
    
    """
    def __init__(self, q, activeIndices=None):
        super(Config, self).__init__(q[:])
        if activeIndices is None:
            self.activeIndices = range(len(self))
        else:
            assert isinstance(activeIndices, list)
            self.activeIndices = activeIndices[:]
            self.activeIndices.sort()
        
    
    def __add__(self, x):
        if self.activeIndices != x.activeIndices:
            raise ValueError("Lists have different active indices.")
        res = [self[i] + x[i] for i in xrange(len(self))]
        return Config(res, self.activeIndices)

    def __sub__(self, x):
        if self.activeIndices != x.activeIndices:
            raise ValueError("Lists have different active indices.")
        res = [self[i] - x[i] for i in xrange(len(self))]
        return Config(res, self.activeIndices)


    def __mul__(self, z):
        res = [y*z for y in self]
        return Config(res, self.activeIndices)
    
    
    def __rmul__(self, z):
        return self.__mul__(z)
    

    def __div__(self, z):
        res = [y/z for y in self]
        return Config(res, self.activeIndices)


    def __lt__(self, x):
        if self.activeIndices != x.activeIndices:
            raise ValueError("Lists have different active indices.")
        return all(a < b + EPSILON if i in self.activeIndices else True
                   for i, (a, b) in enumerate(zip(self, x)))

    
    def __le__(self, x):
        if self.activeIndices != x.activeIndices:
            raise ValueError("Lists have different active indices.")
        return all(a <= b + EPSILON if i in self.activeIndices else True
                   for i, (a, b) in enumerate(zip(self, x)))


    def __gt__(self, x):
        if self.activeIndices != x.activeIndices:
            raise ValueError("Lists have different active indices.")
        return all(a + EPSILON > b if i in self.activeIndices else True
                   for i, (a, b) in enumerate(zip(self, x)))

    
    def __ge__(self, x):
        if self.activeIndices != x.activeIndices:
            raise ValueError("Lists have different active indices.")
        return all(a + EPSILON >= b if i in self.activeIndices else True
                   for i, (a, b) in enumerate(zip(self, x)))


    def Subtract(self, x, activeIndices=None):
        if activeIndices is None:
            activeIndices = self.activeIndices
        return Config([a - b if i in activeIndices else 0.0 for i, (a, b) in enumerate(zip(self, x))],
                      activeIndices=activeIndices)


class Vertex(object):
    """A vertex of RRT.
    
    """
    def __init__(self, config):
        self.config = config
        self.index = None
        self.parentindex = None
        self.path = None

        # waitingIndices stores the indices of the robots that are
        # waiting
        self.waitingIndices = []

        # constrainedIndexSets stores lists. Each list keeps the
        # indices of the robots that are in the same shared
        # segment. All robots in the same shared segment are
        # constrained to have the same velocity so that they do not
        # collide with each other.
        self.constrainedIndexSets = []
        

class Tree(object):
    """A planning tree.
    
    """
    def __init__(self, vroot):
        self.verticeslist = []
        self.AddVertex(None, vroot, None)


    def __len__(self):
        return len(self.verticeslist)


    def __getitem__(self, index):
        return self.verticeslist[index]


    def AddVertex(self, parentindex, vnew, cdpath=None):
        vnew.index = len(self)
        vnew.path = cdpath
        vnew.parentindex = parentindex
        self.verticeslist.append(vnew)


    def GetKNNIndices(self, k, config, offsetIndex):
        """Return k indices of nearest neighbors to config. The distance
        metric is defined by the function Distance2. The number k is
        defined in constants.py.

        Parameters
        ----------
        config : Config
        offsetIndex : integer
            When the tree changes its dimensionality, specifically
            when one or more aircraft have arrived at their
            destinations, at a vertex v with v.index = i, the planning
            continues without considering the finished aircraft
            again. Therefore, there is no need to ever consider again
            vertices with their indices < i. (That is, the planning is
            considered to be initiated with a new tree rooted at v.)
            This offsetIndex is a flag of this function telling it
            from which index to consider for computing nearest
            neighbors.

        Returns
        -------
        nnindices : list
            List of vertex indices, sorted in ascending order by their
            distances to config.

        """
        global _NNN, _NNNTIME
        _NNN += 1
        t0 = time.time()
        nv = len(self)
        if k < 0:
            k = nv
        else:
            k = min(k, nv)
        distanceList = [Distance2(config, v.config, activeIndices=config.activeIndices)
                        for v in self.verticeslist[offsetIndex:]]

        sortedIndices = sorted(range(len(distanceList)), key=lambda i: distanceList[i])
        nnIndices = []
        
        while True:
            index = sortedIndices.pop(0) + offsetIndex
            if index > len(self) - 1:
                break
            if self[index].config <= config:
                # This neighbor satisfies monotonicity requirement.
                nnIndices.append(index)
                if len(nnIndices) == k:
                    break
            if len(sortedIndices) == 0:
                break
            
        assert len(nnIndices) > 0
        _NNNTIME += time.time() - t0
        return nnIndices


    def ExtractPath(self, index):
        """Retrieve a sequence of Configs along the path on the tree,
        backtracing from self[index].

        """
        path = []
        path.append(self[index].config)
        while self[index].parentindex is not None:
            index = self[index].parentindex
            path.append(self[index].config)
        return path[::-1]


################################################################################

class Parameters(object):

    def __init__(self, **kwargs):
        self._backoffDistance = kwargs.get("backoffDistance", 40)
        self._ccStepSize = kwargs.get("ccStepSize", 1.)
        self._collisionRadius = kwargs.get("collisionRadius", 10)
        self._preventionDistance = kwargs.get("preventionDistance", 20)
        self._expansionCoeff = kwargs.get("expansionCoeff", 1.2)
        self._nn = kwargs.get("nn", 1)
        self._rrtStepSize = kwargs.get("rrtStepSize", 10.)
        self._useVirtualBoxes = kwargs.get("useVirtualBoxes", True)

    @property
    def backoffDistance(self):
        """
        """
        return self._backoffDistance
        
    @property
    def ccStepSize(self):
        """ccStepSize is a step size used in collsiion checking. A path (in
        the coordination diagram) is discretized into segments, each
        of length ccStepSize units.

        """
        return self._ccStepSize

    @property
    def collisionRadius(self):
        """collisionRadius determines how close two or more aircraft can be
        between one another. Aircraft 1 with configuration q1 and
        Aircraft 2 with configuration q2 are considered to be in
        collision if Distance(q1, q2) < collisionRadius.

        """
        return self._collisionRadius

    @property
    def preventionDistance(self):
        """This value specifies the maximum clearance from a CD configuration
        to any obstacle.

        """
        return self._preventionDistance
    
    @property
    def expansionCoeff(self):
        """
        """
        return self._expansionCoeff

    @property
    def nn(self):
        """
        """
        return self._nn

    @property
    def rrtStepSize(self):
        """rrtStepSize is the step size used in rrt extension step. A new
        configuration added to the tree must be within the distance of
        rrtStepSize from its parent.

        """
        return self._rrtStepSize

    @property
    def useVirtualBoxes(self):
        """
        """
        return self._useVirtualBoxes


class RRTPlanner(object):

    def __init__(self, cd):
        self.cd = cd # coordination diagram
        self.params = self.cd.params
        self._totaltime = 0.0
        self._iterations = 0
        self._solved = False
        self._Initialize()


    def _Initialize(self):
        """
        """
        self.nrobots = len(self.cd.robots)
        
        configStart = Config([0.] * self.nrobots)
        self.treeStart = Tree(Vertex(configStart))
        self.currentRoot = configStart
        self.currentActiveIndices = self.currentRoot.activeIndices
        # nearest neighbor computation considers only vertices with
        # their indices greater than _offsetIndex. This is basically a
        # way to removed finished robots.
        self._offsetIndex = 0

        self.configGoal = Config([robot.pathLength for robot in self.cd.robots])


    def Plan(self, allotedtime):
        if len(self.cd.robots) < 1:
            log.debug("No robot has been added to Coordination Diagram yet.")
            return False
        
        tPlan = 0.0
        self.CheckAndPreventConflicts()
        while tPlan < allotedtime:
            self._iterations += 1
            log.debug("Iteration {0}: nrobots = {1}".
                      format(self._iterations, len(self.currentActiveIndices)))
            tloopStart = time.time()

            cnew = self.SampleConfig()
            status = self.Extend(cnew)
            if status == REACHED:
                tloopEnd = time.time()
                tElapsed = tloopEnd - tloopStart
                tPlan += tElapsed
                self._totaltime += tPlan
                self._solved = True
                msg =  "Path found."
                msg += "    Total running time : {0} sec".format(self._totaltime)
                log.debug(msg)
                print "nn: time = {0}/{1}".format(_NNNTIME, _NNN)
                return True
            log.debug("    extension status: {0}; nvertices = {1}".format(status, len(self.treeStart)))

            tloopEnd = time.time()
            tElapsed = tloopEnd - tloopStart
            tPlan += tElapsed

        self._totaltime += tPlan
        msg =  "Alloted time of {0} sec. has been reached.".format(allotedtime)
        msg += "Total running time : {0} sec".format(self._totaltime)
        log.debug(msg)
        return False
 

    def _RNG(self, u, l=None):
        """Generate a random number. If l is not given, a number generated is
        between 0 and u. If l is also given, a number generated is
        between l and u.

        This function assumes correct inputs.

        """
        if l is None:
            l = 0
        x = (u - l) * np.random.random_sample() + l
        return x


    def _Crop(self, c, cnear):
        """Go from cnear in the direction of (c - cnear) until either
        reaching c or one of the robot reaching its goal.
        
        Returns
        -------
        cnew : Config
        cropped : bool
            True if some robots have reached their goals, else False.

        """
        dc = (c - cnear) / Norm(c - cnear) # unit vector from cnear to c
        
        cropped = False # indicates if any DOF has been cropped
        reachingIndex = None # the index of the robot which reaches its goal first
        minMult = np.infty
        # Check if any robot has reached its goal
        for i in c.activeIndices:
            if c[i] >= self.configGoal[i]:
                if abs(dc[i]) < EPSILON:
                    continue
                
                cropped = True                
                k = (self.configGoal[i] - cnear[i]) / dc[i]
                if k < minMult:
                    minMult = k
                    reachingIndex = i

        activeIndices = c.activeIndices
        if not cropped:
            cnew = Config(c, activeIndices)
        else:
            activeIndices.remove(reachingIndex)
            cnew = Config(cnear + dc * minMult, sorted(activeIndices))
        return cnew, cropped
            
    
    def SampleConfig(self):
        """Sample a new configuration (in the coordination diagram). Elements
        at inactive indices are filled with the corresponding elements
        in the goal configuration.

        """
        if self.params.useVirtualBoxes:
            upperBounds = [max(self.configGoal) * self.params.expansionCoeff] * self.nrobots
        else:
            upperBounds = self.configGoal
        config = Config([self._RNG(u, l) if i in self.currentActiveIndices else self.configGoal[i]
                         for i, (u, l) in enumerate(zip(upperBounds, self.currentRoot))],
                        activeIndices=self.currentActiveIndices)
        return config


    def Extend(self, crand):
        status = TRAPPED
        nnIndices = self.treeStart.GetKNNIndices(self.params.nn, crand, self._offsetIndex)
        # Iterate though the list of nearest neighbors
        for index in nnIndices:
            vnear = self.treeStart[index]
            cnear = vnear.config

            # Manage spacial robots
            dc = crand - cnear
            # Robots with velocity constraints
            for indices in vnear.constrainedIndexSets:
                vmin = min([dc[i] for i in indices])
                for i in indices:
                    dc[i] = vmin

            # Waiting robots
            for i in vnear.waitingIndices:
                dc[i] = 0
                
            q = [x + y for (x, y) in zip(cnear, dc)]

            crand_new = Config(q, crand.activeIndices)
            # print "dc = {0}".format(dc)
            # import IPython; IPython.embed(header="crand_new")
            dist = Distance(cnear, crand_new)
            # Limite the new Config to be within rrtStepsize units from its neighbor
            if dist <= self.params.rrtStepSize:
                ctest = crand_new
            else:
                dc = crand_new - cnear # direction from cnear to cnew
                if Norm(dc) <= EPSILON:
                    import IPython; IPython.embed(header="Extend (norm(dc) == 0)")
                ctest = cnear + dc * (self.params.rrtStepSize / Norm(dc))

            if Norm(ctest - cnear) <= EPSILON:
                import IPython; IPython.embed(header="Extend (norm(ctest - cnear) == 0)")
            # Check if any robot has reached its goal
            cnew, cropped = self._Crop(ctest, cnear)

            # Check collision. Note that vnear is also an input to
            # this collision checking procedure in order for us to be
            # able to update waitingIndices.
            if self.cd.CheckCollisionConfig(cnew, self.currentActiveIndices, vnear):
                continue
            if self.cd.CheckCollisionSegment(cnear, cnew, self.currentActiveIndices, vnear):
                continue

            # Check if the new Config is actually the goal Config
            if Distance(cnew, self.configGoal) <= EPSILON:
                status = REACHED
            else:
                status = ADVANCED

            # Add the new vertex to the tree
            vnew = Vertex(cnew)
            self.treeStart.AddVertex(index, vnew)
            if cropped:
                self._offsetIndex = vnew.index
                self.currentActiveIndices = cnew.activeIndices
                self.currentRoot = cnew

            # Check if the newly added vertex is near any conflict zone
            self.CheckAndPreventConflicts()
            
            break

        return status


    def CheckAndPreventConflicts(self):
        """Check if the newly added config is near any conflict zones. If so,
        apply the conflict prevention policy.

        When a CD configuration is near an obstacle, there is a narrow
        passage which is difficult to get through without any aids by
        a conflict prevention policy.

        POLICY
        ------
        DEADLOCKS: 

            The deadlock-induced obstacle is described by
                DL = {(s1, s2) | s1 \in [s1_start - a, s1_end + b] and 
                                 s2 \in [s2_start - 1, s2_end + b]}.
            If robot1 has already entered the deadlock, i.e., 
                s1 \in [s1_start - a, s1_end + b],
            and robot2 is closer than 'preventionDistance' to this deadlock,        
            robot2 will need to wait until this robot has left the conflict zone.

        SHARED SEGMENTS:
        
        JUNCTIONS:            
        
        """
        v = self.treeStart[-1] # the newly added vertex
        c = v.config

        waitingIndices = []
        constrainedIndexSets = []
        a = self.cd._a
        b = self.cd._b
        d = self.params.preventionDistance

        conflictEdges = self.cd.conflictzones.edges(data=True)
        for rindex1, rindex2, data in conflictEdges:
            # Iterate through every possible pair of aircraft
            
            if ((rindex1 not in self.currentActiveIndices) or
                (rindex2 not in self.currentActiveIndices)):
                # Not relevant: at least one of the robot is inactive.
                continue

            s1 = c[rindex1]
            s2 = c[rindex2]
            
            for conflictzone in data['conflictzones']:
                # Iterate through all conflict zones there are.
                s1_start, s1_end = conflictzone.intervals[rindex1]
                s2_start, s2_end = conflictzone.intervals[rindex2]

                if conflictzone.type == ConflictType.DEADLOCK:
                    if (s1_start - a <= s1 <= s1_end + b):
                        # Robot 1 is in this segment
                        if (s2_start - a - d <= s2 <= s2_start - a):
                            # Robot 2 is near the entrance to this deadlock so it needs to wait
                            waitingIndices.append(rindex2)
                        else: # DO NOTHING HERE
                            pass
                    elif (s2_start - a <= s2 <= s2_end + b):
                        # Robot 2 is in this segment
                        if (s1_start - a - d <= s1 <= s1_start - a):
                            # Robot 1 is near the entrance to this deadlock so it needs to wait
                            waitingIndices.append(rindex1)
                        else: # DO NOTHING HERE
                            pass
                    
                elif conflictzone.type == ConflictType.SHARED:
                    if ((s1_start - a <= s1 <= s1_start) and
                        (s2_start - a - d <= s2 <= s2_start - a)):
                        # Robot 2 should wait until Robot 1 has already entered the segment.
                        waitingIndices.append(rindex2)
                    elif ((s2_start - a <= s2 <= s2_start) and
                          (s1_start - a - d <= s1 <= s1_start - a)):
                        # Robot 1 should wait until Robot 2 has already entered the segment.
                        waitingIndices.append(rindex1)
                    elif (((s1_start <= s1 <= s1_end + a) and
                          (s2_start - a <= s2 <= (s1 + s2_start - a - s1_start)))
                          or
                          ((s2_start <= s2 <= s2_end + a) and
                           (s1_start - a <= s1 <= (s2 + s1_start - a - s2_start)))):
                        indices = [rindex1, rindex2]
                        appeared = False # check if robot1/robot2 is already part of any other
                                         # shared segment conflict
                        for i in xrange(len(constrainedIndexSets)):
                            if rindex1 in constrainedIndexSets[i] or rindex2 in constrainedIndexSets[i]:
                                constrainedIndexSets[i] += indices
                                constrainedIndexSets[i] = list(set(constrainedIndexSets[i]))
                                appeared = True
                                break
                            
                        if not appeared:
                            constrainedIndexSets.append(indices)
                        
                else: # JUNCTION
                    if ((s1_start - a - d <= s1 <= s1_start + a) and
                        (s2_start - a - d <= s2 <= s2_start - a)):
                        waitingIndices.append(rindex2)
                    elif ((s2_start - a <= s2 <= s2_start + a) and
                          (s1_start - a - d <= s1 <= s1_start - a)):
                        waitingIndices.append(rindex1)

                        
        v.waitingIndices = sorted(list(set(waitingIndices)))
        v.constrainedIndexSets = constrainedIndexSets
        return
    

    def CheckAndPreventConflicts_(self):
        """Check if the newly added config is near any conflict zones. If so,
        prevent the potential conflict(s) by making some robots wait
        before entering the conflict zone(s).

        """
        foundPotentialConflicts = False
        v = self.treeStart[-1]
        c = v.config
        waitingIndices = []
        conflictEdges = self.cd.conflictzones.edges(data=True)
        for rindex1, rindex2, data in conflictEdges:
            if ((rindex1 not in self.currentActiveIndices) or
                (rindex2 not in self.currentActiveIndices)):
                # Not relevant
                continue
            
            for conflictzone in data['conflictzones']:
                if conflictzone.type == ConflictType.DEADLOCK:
                    s1_start, s1_end = conflictzone.intervals[rindex1]
                    s2_start, s2_end = conflictzone.intervals[rindex2]

                    s1_start -= self.params.backoffDistance
                    s1_end   += self.params.backoffDistance
                    s2_start -= self.params.backoffDistance
                    s2_end   += self.params.backoffDistance

                    if ((s1_start <= c[rindex1] <= s1_end) and
                        (s2_start - self.params.preventionDistance <= c[rindex2] <= s2_start)):
                        waitingIndices.append(rindex2) # robot2 needs to wait until robot1 gets out
                    elif ((s2_start <= c[rindex2] <= s2_end) and
                          (s1_start - self.params.preventionDistance <= c[rindex1] <= s1_start)):
                        waitingIndices.append(rindex1) # robot1 needs to wait until robot2 gets out

                elif conflictzone.type == ConflictType.SHARED:
                    s1_start, s1_end = conflictzone.intervals[rindex1]
                    s2_start, s2_end = conflictzone.intervals[rindex2]

                    if ((s1_start - self.params.backoffDistance <= c[rindex1] <= s1_start) and
                        (s2_start - self.params.backoffDistance - self.params.preventionDistance <=
                         c[rindex2] <= s2_start - self.params.backoffDistance)):
                        waitingIndices.append(rindex2) # robot2 needs to wait until robot1 gets in
                    elif ((s2_start - self.params.backoffDistance <= c[rindex2] <= s2_start) and
                          (s1_start - self.params.backoffDistance - self.params.preventionDistance
                           <= c[rindex1] <= s1_start - self.params.backoffDistance)):
                        waitingIndices.append(rindex1) # robot1 needs to wait until robot2 gets in
                    elif ((s1_start - self.params.backoffDistance <= c[rindex1] <=
                           s1_end + self.params.backoffDistance) and
                          (s2_start - self.params.backoffDistance <= c[rindex2] <=
                           s2_end + self.params.backoffDistance)):
                        # Both robots are inside this shared segment
                        pos1, _, _ = self.cd.robots[rindex1].Locate(c[rindex1])
                        pos2, _, _ = self.cd.robots[rindex2].Locate(c[rindex2])
                        diff = pos1 - pos2
                        dist = np.sqrt(np.dot(diff, diff))
                        if dist <= self.params.collisionRadius + self.params.preventionDistance:
                            # The two robots are too close, make one wait.
                            if (s1_end + self.params.backoffDistance - c[rindex1] <
                                s2_end + self.params.backoffDistance - c[rindex2]):
                                # robot1 is closer to the exit
                                waitingIndices.append(rindex2)
                            else:
                                waitingIndices.append(rindex1)
                                
                else:
                    s1_start, s1_end = conflictzone.intervals[rindex1]
                    s2_start, s2_end = conflictzone.intervals[rindex2]

                    if ((s1_start - self.params.backoffDistance <= c[rindex1] <= s1_start) and
                        (s2_start - self.params.backoffDistance - self.params.preventionDistance <=
                         c[rindex2] <= s2_start - self.params.backoffDistance)):
                        waitingIndices.append(rindex2) # robot2 needs to wait until robot1 gets in
                    elif ((s2_start - self.params.backoffDistance <= c[rindex2] <= s2_start) and
                          (s1_start - self.params.backoffDistance - self.params.preventionDistance
                           <= c[rindex1] <= s1_start - self.params.backoffDistance)):
                        waitingIndices.append(rindex1) # robot1 needs to wait until robot2 gets in
        v.waitingIndices = sorted(list(set(waitingIndices)))
        return


    def ExtractTrajectory(self):
        if not self._solved:
            log.debug("Solution path has not been found.")
            return None
        # rawpath is a list of Config.
        rawpath = self.treeStart.ExtractPath(-1)
        return CDPath.FromConfigsList(rawpath)
                
        
class CDPath(Trajectory.PiecewisePolynomialTrajectory):

    @staticmethod
    def FromConfigsList(path):
        """
        """
        dim = len(path[0])
        trajstring = ""

        chunksep = ""
        prevconfig = path[0]
        for config in path[1:]:
            dur = Distance(prevconfig, config, prevconfig.activeIndices)
            if dur < EPSILON:
                continue
            
            trajstring += chunksep
            chunksep = "\n"
            trajstring += "{0}\n{1:d}\n".format(dur, dim)
            dofsep = ""
            for idof in xrange(dim):
                a, b = ComputeLinearCoeffs(prevconfig[idof], config[idof], dur)
                trajstring += dofsep
                dofsep = "\n"
                trajstring += "{0} {1}".format(b, a)
            prevconfig = config
        return Trajectory.PiecewisePolynomialTrajectory.FromString(trajstring)


def ComputeLinearCoeffs(x1, x2, duration):
    """Generate a set of linear polynomial coefficients for the line
    connecting x1 and x2. (x1 --> x2)

    Returns
    -------
    a : float
        Coefficient of the term of degree 1.
    b : float
        Coefficient of the term of degree 0.

    """
    assert(duration > 0)
    b = x1
    a = (x2 - b)/duration
    return a, b
    
    
        
    
            
