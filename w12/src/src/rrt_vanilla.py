import numpy as np
import time
from constants import EPSILON, RANDSEED
from misc import Norm, Distance, Distance2, Config, CDPath
from maps2 import ConflictType
# Use numpy's random number generator instead of random.SystemRandom()
# for reproducibility (better for debugging).
np.random.seed(RANDSEED)


from TOPP import Trajectory

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

from rrt2 import Vertex, Tree, RRTPlanner

class Parameters(object):

    def __init__(self, **kwargs):
        self._ccStepSize = kwargs.get("ccStepSize", 1.)
        self._collisionRadius = kwargs.get("collisionRadius", 10)
        self._preventionDistance = kwargs.get("preventionDistance", 20)
        self._nn = kwargs.get("nn", 1)
        self._rrtStepSize = kwargs.get("rrtStepSize", 10.)
        self._goalBias = kwargs.get("goalBias", 0.2)
        self._useVirtualBoxes = False
        self._expansionCoeff = kwargs.get("expansionCoeff", 1.2)
        
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
    def nn(self):
        """
        """
        return self._nn

    @property
    def preventionDistance(self):
        """This value specifies the maximum clearance from a CD configuration
        to any obstacle.

        """
        return self._preventionDistance

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

    
class VanillaRRTPlanner(RRTPlanner):

    def Extend(self, crand):
        status = TRAPPED
        nnIndices = self.treeStart.GetKNNIndices(self.params.nn, crand, self._offsetIndex)
        # Iterate though the list of nearest neighbors
        for index in nnIndices:
            vnear = self.treeStart[index]
            cnear = vnear.config

            dist = Distance(cnear, crand)
            # Limit the new Config to be within rrtStepsize units from its neighbor
            if dist <= self.params.rrtStepSize:
                cnew = crand
            else:
                dc = crand - cnear # direction from cnear to cnew
                if Norm(dc) <= EPSILON:
                    # import IPython; IPython.embed(header="Extend (norm(dc) == 0)")
                    return TERMINATED
                cnew = cnear + dc * (self.params.rrtStepSize / Norm(dc))

            if Norm(cnew - cnear) <= EPSILON:
                # import IPython; IPython.embed(header="Extend (norm(ctest - cnear) == 0)")
                return TERMINATED

            # Check for collisions.
            if self.cd.CheckCollisionConfig2(cnew, self.currentActiveIndices, vnear):
                continue
            if self.cd.CheckCollisionSegment2(cnear, cnew, self.currentActiveIndices, vnear):
                continue

            # Add the new vertex to the tree
            vnew = Vertex(cnew)
            self.treeStart.AddVertex(index, vnew)
            status = ADVANCED
            
            if np.random.random() <= self.params.goalBias:
                # Attempt connection
                if not self.cd.CheckCollisionSegment2(cnew, self.configGoal,
                                                      self.currentActiveIndices):
                    vgoal = Vertex(self.configGoal)
                    self.treeStart.AddVertex(vnew.index, vgoal)
                    status = REACHED

            return status

        return status
