import numpy as np
import time
from constants import EPSILON, RANDSEED
from misc import Norm, Distance, Distance2, Config, CDPath
from maps2 import ConflictType
# Use numpy's random number generator instead of random.SystemRandom()
# for reproducibility (better for debugging).
np.random.seed(RANDSEED)
try:
    from TOPP import Trajectory
except:
    import Trajectory
import logging
loglevel = logging.DEBUG
logging.basicConfig\
    (format='[%(levelname)s] [%(name)s: %(funcName)s] %(message)s', level=loglevel)
log = logging.getLogger(__name__)

TRAPPED = 0
ADVANCED = 1
REACHED = 2
TERMINATED = -1

_NNN = 0
_NNNTIME = 0.0

from operator import itemgetter
import IPython
        
class Vertex(object):
    """
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
    """
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
        self._goalBias = kwargs.get("goalBias", 0.2)

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
    def goalBias(self):
        """goalBias determines how often a "vanilla" RRT planner would connect
        a newly created vertex to the goal.

        """
        return self._goalBias
    
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
            #####
            if status == TERMINATED:
                return False
            #####
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
            log.debug("    extension status: {0}; nvertices = {1}".\
                      format(status, len(self.treeStart)))

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
        config = Config([self._RNG(u, l) if i in self.currentActiveIndices
                         else self.configGoal[i]
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

            ################################################################################
            # Manage spacial robots
            dc = crand - cnear
            # Robots with velocity constraints
            for indices in vnear.constrainedIndexSets:
                # vmin = min([dc[i] for i in indices])

                # Sometimes, vmin is too little such that robots get
                # stuck. Maybe we need to add some randomness when selecting a
                # value.
                vconstrained = max([dc[i] for i in indices])
                for i in indices:
                    dc[i] = vconstrained

            # Waiting robots
            for i in vnear.waitingIndices:
                dc[i] = 0
                
            q = [x + y for (x, y) in zip(cnear, dc)]

            crand_new = Config(q, crand.activeIndices)
            ################################################################################
            
            # print "dc = {0}".format(dc)
            # import IPython; IPython.embed(header="crand_new")
            dist = Distance(cnear, crand_new)
            # Limit the new Config to be within rrtStepsize units from its neighbor
            if dist <= self.params.rrtStepSize:
                ctest = crand_new
            else:
                dc = crand_new - cnear # direction from cnear to cnew
                if Norm(dc) <= EPSILON:
                    # import IPython; IPython.embed(header="Extend (norm(dc) == 0)")
                    return TERMINATED
                ctest = cnear + dc * (self.params.rrtStepSize / Norm(dc))

            if Norm(ctest - cnear) <= EPSILON:
                # import IPython; IPython.embed(header="Extend (norm(ctest - cnear) == 0)")
                return TERMINATED
            # Check if any robot has reached its goal
            cnew, cropped = self._Crop(ctest, cnear)

            # Check for collisions.
            # Note that vnear is also an input to this collision checking
            # procedure in order for us to be able to update waitingIndices.
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
        """Check if the newly added config is near any conflict. If so, apply the
        conflict prevention policy.

        """
        v = self.treeStart[-1] # the newly added vertex
        c = v.config

        waitingIndices = []
        constrainedIndexSets = []
        a = self.cd._a
        b = self.cd._b
        d = self.params.preventionDistance

        # When there is (seems to be) freedom to choose which robot to
        # wait, we need to be more careful since the choice we make
        # can affect the problem. The order in which we iterate
        # through the conflict also affects the problem.

        # decisionSets is a list of sublists. Each sublist contains
        # robot indices among which we can choose them to
        # wait. Suppose a sublist is [rindex1, rindex2]. There might
        # be some future conflict that forces, e.g., rindex1 to
        # wait. Then we remove the sublist.
        decisionSets = []

        # movingIndices is a list of sublists. Each sublist contains robot
        # indices among which at least one robot MUST BE MOVING. (Otherwise, a
        # configuration will be stuck in a deadlock.)
        movingIndices = []

        for conflict in self.cd.conflicts:

            if conflict.type == ConflictType.DEADLOCK:
                # indices store active indices that are related to this deadlock.
                if not set(conflict.indices).issubset(set(self.currentActiveIndices)):
                    continue
                indices = sorted(conflict.indices)
                # indices = sorted(list(set(conflict.indices) & set(self.currentActiveIndices)))
                # if len(indices) < 2:
                #     continue

                svect = Config([c[rindex] for rindex in indices])
                # svect_inner = Config([conflict.intervals[rindex][0] - a
                #                       for rindex in indices])
                svect_inner = Config([conflict.intervals[rindex][0] for rindex in indices])

                svect_outer = Config([s - d for s in svect_inner])
                # svect_max   = Config([conflict.intervals[rindex][1] + b
                #                       for rindex in indices])
                svect_max   = Config([conflict.intervals[rindex][1] for rindex in indices])

                if not (svect.Dominates(svect_outer) and svect_max.Dominates(svect)):
                    # This config is not near this deadlock obstacle
                    continue

                if svect_inner.Dominates(svect):
                    # All robots that are related to this deadlock have not
                    # entered the deadlock yet.
                    intersect = list(set(indices) & set(waitingIndices))
                    if len(intersect) > 0:
                        # At least one robot is waiting so that's fine.
                        continue
                    else:
                        # In this case, we will decide later which one to wait.
                        decisionSets.append(indices)
                else:
                    # Some robots have already entered the deadlock. See first
                    # which robots have already entered or not entered.
                    diff = svect - svect_inner
                    decisionSet = [] # this list keeps the indices of robots
                                     # outside the deadlock

                    for (ds, index) in zip(diff, indices):
                        if ds < 0:
                            # This robot has not entered the deadlock.
                            decisionSet.append(index)
                    intersect = list(set(decisionSet) & set(waitingIndices))
                    if len(intersect) > 0:
                        # At least one robot outside the deadlock is already
                        # waiting. Do nothing here.
                        continue
                    else:
                        if len(decisionSet) == 1:
                            # # If we reach this case, the robot with index
                            # # decisionSet[0] is forced to wait since it is THE
                            # # ONLY ONE that has not entered the deadlock yet. We
                            # # also need to make sure that at least one of the
                            # # other robots MUST BE moving.
                            # waitingIndices.append(decisionSet[0])
                            # if len(indices) > 2:
                            #     # We may need to examine this case more closely.
                            #     # IPython.embed(header="len(conflict.indices) > 2")
                            #     moving = list(set(indices) - set(decisionSet))
                            #     movingIndices.append(moving)
                            # else:
                            #     # index is the index of the robot that must be moving.
                            #     index = list(set(indices) - set(decisionSet))[0]
                            #     for decisionSet in decisionSets:
                            #         # Remove any appearance of index in decision sets.
                            #         try:
                            #             decisionSet.remove(index)
                            #         except:
                            #             pass

                            # newDecisionSets = []
                            # for decisionSet in decisionSets:
                            #     if len(decisionSet) == 0:
                            #         # This should not be the case
                            #         raise ValueError
                            #     elif len(decisionSet) == 1:
                            #         # The decision has been made.
                            #         waitingIndices.append(decisionSet[0])
                            #     else:
                            #         intersect = list(set(waitingIndices) & set(decisionSet))
                            #         if len(intersect) > 0:
                            #             continue
                            #         else:
                            #             newDecisionSets.append(decisionSet)
                            # decisionSets = newDecisionSets                           
                            if decisionSet[0] not in waitingIndices:
                                waitingIndices.append(decisionSet[0])
                            
                        else:
                            # In this case, we decide later
                            decisionSets.append(decisionSet)
                        
            else:
                # Elementary conflict (shared segment or junction)
                rindex1, rindex2 = conflict.indices
                if ((rindex1 not in self.currentActiveIndices) or
                    (rindex2 not in self.currentActiveIndices)):
                    # At least one robot is inactive: this conflict is not relevant
                    continue
                s1 = c[rindex1]
                s2 = c[rindex2]
                s1_start, s1_end = conflict.intervals[rindex1]
                s2_start, s2_end = conflict.intervals[rindex2]

                if conflict.type == ConflictType.SHARED:
                    # SHARED SEGMENT
                    # if ((s1_start - a - d <= s1 <= s1_start) and
                    #     (s2_start - a - d <= s2 <= s2_start - a)):
                    #     # Both robots have not entered the shared segment
                    #     # yet. So we keep them as candidates.
                    #     decisionSet = [rindex1, rindex2]
                    #     decisionSets.append(decisionSet)
                    # elif ((s1_start - a - d <= s1 <= s1_start - a) and
                    #       (s2_start - a <= s2 <= s2_start)):
                    #     # Both robots have not entered the shared segment
                    #     # yet. So we keep them as candidates.
                    #     decisionSet = [rindex1, rindex2]
                    #     decisionSets.append(decisionSet)
                    if ((s1_start - d <= s1 <= s1_start) and
                        (s2_start - d <= s2 <= s2_start)):
                        # Both robots have not entered the shared segment
                        # yet. So we keep them as candidates.
                        decisionSet = [rindex1, rindex2]
                        decisionSets.append(decisionSet)
                    elif ((s1_start - d <= s1 <= s1_start + a) and
                          (s2_start - d <= s2 <= s2_start + a)):
                        if s1 >= s1_start:
                            waitingIndices.append(rindex2)
                        else:
                            waitingIndices.append(rindex1)                            
                    # elif ((s1_start - a - d <= s1 <= s1_end + a) and
                    #       (s2_start - a - d <= s2 <= s2_end + a)):
                    elif ((s1_start - d <= s1 <= s1_end) and
                          (s2_start - d <= s2 <= s2_end)):
                        # This configuration is inside a velocity constrained
                        # region. (Both robots are inside a shared segment so
                        # they should move at the same speed.)
                        indices = [rindex1, rindex2]

                        # Now we need to check if robot1/robot2 is already part
                        # of any other shared segment conflict.
                        appeared = False
                        for i in xrange(len(constrainedIndexSets)):
                            if ((rindex1 in constrainedIndexSets[i]) or
                                (rindex2 in constrainedIndexSets[i])):
                                newIndices = list(set(constrainedIndexSets[i] + indices))
                                constrainedIndexSets[i] = newIndices
                                appeared = True
                                break

                        if not appeared:
                            constrainedIndexSets.append(indices)
                else:
                    # JUNCTION
                    # if ((s1_start - d <= s1 <= s1_end) and
                    #     (s2_start - d <= s2 <= s2_end)):
                    #     # Both robots almost reach the junction.
                    #     decisionSet = [rindex1, rindex2]
                    #     decisionSets.append(decisionSet)
                    if ((s1_start - d <= s1 <= s1_start) and
                        (s2_start - d <= s2 <= s2_start)):
                        # Both robots are about to cross this junction.
                        decisionSet = [rindex1, rindex2]
                        decisionSets.append(decisionSet)
                
                    elif ((s1_start - d <= s1 <= s1_end) and
                          (s2_start - d <= s2 <= s2_end)):
                        rem1 = s1_end - s1
                        rem2 = s2_end - s2
                        
                        # The robot that is behind the other shall wait.
                        if rem1 > rem2:
                            waitingIndices.append(rindex1)
                        else:
                            waitingIndices.append(rindex2)
                            

        # Now we have already iterated through all conflicts.

        # Manage the pending decisions.
        newDecisionSets = []
        for decisionSet in decisionSets:
            intersect = list(set(waitingIndices) & set(decisionSet))
            if len(intersect) > 0:
                continue
            else:
                newDecisionSets.append(decisionSet)
        decisionSets = newDecisionSets

        if len(decisionSets) > 0:
            appearance = dict()
            for decisionSet in decisionSets:
                for el in decisionSet:
                    if el not in appearance:
                        appearance[el] = 1
                    else:
                        appearance[el] += 1

            # Sort the dictionary `appearance` by its values (descending order).
            sorted_appearance = sorted(appearance.items(), key=itemgetter(1))

            # Idea: indices that appear most often should be waiting so as to
            # minimize the waiting robots.
            for item in sorted_appearance:
                index, _ = item
                newDecisionSets = []
                for decisionSet in decisionSets:
                    try:
                        decisionSet.remove(index)
                        waitingIndices.append(index)
                    except:
                        pass
                    if len(decisionSet) > 1:
                        newDecisionSets.append(decisionSet)
                decisionSets = newDecisionSets
                if len(decisionSets) == 0:
                    break
        if len(decisionSets) > 0:
            # All pending decisions should have already been made.
            raise ValueError

        for movingSet in movingIndices:
            if len(list(set(movingSet) - set(waitingIndices))) == 0:
                import IPython; IPython.embed(header="error with moving indices")

        # Manage constrained index sets
        
        # Idea: if there are three robot in the shared segment but the one
        # behind is waiting, that robot should not affect the other two. If the
        # one waiting is not the one behind, the moving one will eventually
        # collide with it and be made waiting anyway.
        waitingSet = set(waitingIndices)
        newconstrainedset = []
        for constrainedset in constrainedIndexSets:
            constrainedset = list(set(constrainedset) - waitingSet)
            if len(constrainedset) > 1:
                newconstrainedset.append(constrainedset)
                
        v.waitingIndices = sorted(list(set(waitingIndices)))
        v.constrainedIndexSets = newconstrainedset # constrainedIndexSets
        return


    def ExtractTrajectory(self):
        if not self._solved:
            log.debug("Solution path has not been found.")
            return None
        # rawpath is a list of Config.
        rawpath = self.treeStart.ExtractPath(-1)
        return CDPath.FromConfigsList(rawpath)
                
            
    
