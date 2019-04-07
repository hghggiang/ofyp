import numpy as np
import bisect
from constants import EPSILON, MULT, NMAXROBOTS
try:
    from TOPP import Trajectory
except:
    import Trajectory
import matplotlib.pyplot as plt
_hsv = plt.get_cmap('hsv')


def Norm(x):
    """Compute the Euclidean norm of x.

    """
    return np.sqrt(sum([y**2 if i in x.activeIndices else 0 for i, y in enumerate(x)]))


def Distance(a, b, activeIndices=None):
    """Compute Euclidean distance between configurations stored in MyList
    a and MyList b.

    """
    return np.sqrt(Distance2(a, b, activeIndices))


def Distance2(a, b, activeIndices=None):
    """Compute Euclidean distance squared.

    """
    if activeIndices is None:
        activeIndices = a.activeIndices
    
    return sum((x - y)**2 if i in activeIndices else 0 for i, (x, y) in enumerate(zip(a, b)))


class Config(list):
    """CD configuration: list with arithmatic operators.

    """
    def __init__(self, lst, activeIndices=None):
        super(Config, self).__init__(lst[:])
        if activeIndices is None:
            self.activeIndices = range(len(self))
        else:
            assert isinstance(activeIndices, list)
            self.activeIndices = sorted(activeIndices)


    # Built-in arithmatic operators require identical active indices.
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
        return Config([a - b if i in activeIndices else 0.0
                       for i, (a, b) in enumerate(zip(self, x))],
                      activeIndices=activeIndices)


    def Dominates(self, x, indices=None):
        if indices is None:
            indices = self.activeIndices
        return all(a + EPSILON >= b if i in indices else True
                   for i, (a, b) in enumerate(zip(self, x)))


class SimpleRobot(object):
    """A class for robot. Each robot contains a description of the path it
    needs to traverse (in the form of a sequence of nodes).

    Attributes
    ----------
    id : integer
        ID of this robot; assigned when added to a coordination diagram
    path : list
        list of graph (physical map) nodes along the path
    cumulatedLength : list
        list of path parameters at each node along the path. 
        Therefore, the first in the list is zero and the last in the list 
        is the path length.
    pathLength : float
    pathSegments: list
        list of edges along the path. Each edge is a tuple of two 
        consecutive nodes.
    
    """
    def __init__(self, path, physmap):
        """        
        Note
        ----
        
        A physical map is needed at initialization time only for
        calculating cumulated path length.

        """
        self.id = None # to be assigned when added to a coordination diagram
        self.path = path # a sequence of physmap's node
        l = 0
        self.cumulatedLength = [l]
        for i, node in enumerate(path[1:]):
            l += physmap.graph[path[i]][node]['dist']
            self.cumulatedLength.append(l)
        self.pathLength = self.cumulatedLength[-1]

        self.pathSegments = [(self.path[i], path) for (i, path) in enumerate(self.path[1:])]
        

    def Locate(self, s):
        """Return the world coordinate of this robot when the robot is placed
        at s, the index of the path segment the robot is in, and the
        residue path parameter in that segment. The path parameter s
        ranges from 0 to pathLength.

        Returns
        -------
        position : numpy.ndarray (2D)
            (x, y) coordinate of the robot
        index : integer
            index of the path segment the robot is in
        srem : float
            path parameter; 0 if at the beginning of self.path[index]
        
        """
        if s < -EPSILON or s > self.pathLength + EPSILON:
            raise ValueError('Invalid path parameter value: s = {0}/{1}'.
                             format(s, self.pathLength))
        if s <= EPSILON:
            position = np.asarray(self.path[0]) / MULT
            index = 0
            srem = 0
        elif s >= self.pathLength - EPSILON:
            position = np.asarray(self.path[-1]) / MULT
            index = len(self.path) - 2
            srem = self.cumulatedLength[-1] - self.cumulatedLength[-2]
        else:
            index = bisect.bisect_left(self.cumulatedLength, s) - 1
            srem = s - self.cumulatedLength[index]
            start = np.asarray(self.path[index]) / MULT
            end = np.asarray(self.path[index + 1]) / MULT
            l = self.cumulatedLength[index + 1] - self.cumulatedLength[index]
            position = start + (end - start)*(srem / l)
        return position, index, srem


    def Plot(self, s, ax, showlabel=True):
        """Plot a dot representing this robot at the location s.

        Parameters
        ----------
        s : float
            path parameter
        ax : matplotlib.axes._subplots.AxesSubplot
            plot axes generated at the time PhysicalMap was drawn
        showlabel : bool, optional
            if True, annotate the plot with "Robot {0}".format(self.id)

        Returns
        -------
        handles : list
            list of plot handles
        
        """
        if self.id is None:
            log.info("This robot has not been added to a coordination diagram yet.")
            return None

        # color = _hsv(float(self.id) / NMAXROBOTS)
        color = plt.cm.Set1(float(self.id) / NMAXROBOTS)
        pos, _, _ = self.Locate(s)
        pos = np.asarray(pos)
        handles = ax.plot(pos[0], pos[1], "h", markersize=18, color=color)
        if showlabel:
            annotationOffset = np.array([1.0, -1.0]) # lower right
            handles.append(ax.annotate("Robot {0}".format(self.id),
                                       weight='bold', color=color,
                                       xy=pos, xycoords='data',
                                       xytext=pos + annotationOffset, textcoords='data'))
        return handles


    def PlotPath(self, ax, showlabels=True, showlegends=True):
        """Plot the path of this robot. The initial position is plotted out as
        a square. The final position is plotted out as a star.

        """
        if self.id is None:
            log.info("This robot has not been added to a coordination diagram yet.")
            return None

        # color = _hsv(float(self.id) / NMAXROBOTS)
        color = plt.cm.Set1(float(self.id) / NMAXROBOTS)
        startpos = np.asarray(self.path[0]) / MULT
        goalpos = np.asarray(self.path[-1]) / MULT
        
        handles = []
        for i, nextpos in enumerate(self.path[1:]):
            curpos = self.path[i]
            handles.append(ax.plot([curpos[0] / MULT, nextpos[0] / MULT],
                                   [curpos[1] / MULT, nextpos[1] / MULT],
                                   "--", linewidth=8, color=color)[0])

        # Squares are initial positions
        handles.append(ax.plot(startpos[0], startpos[1], "s", markersize=14, color=color)[0])
        # Stars are final positions
        if showlegends:
            handles.append(ax.plot(goalpos[0], goalpos[1], "*", markersize=20, color=color,
                                   label="Robot {0}".format(self.id))[0]) 
            plt.legend()
        else:
            handles.append(ax.plot(goalpos[0], goalpos[1], "*", markersize=20, color=color)[0])

        if showlabels:
            annotationOffset = np.array([0.0, -1.0]) # below
            handles.append(ax.annotate("Robot {0}".format(self.id),
                                       weight='bold', color=color,
                                       xy=startpos, xycoords='data',
                                       xytext=startpos + annotationOffset, textcoords='data'))
            handles.append(ax.annotate("Robot {0}".format(self.id),
                                       weight='bold', color=color,
                                       xy=goalpos, xycoords='data',
                                       xytext=goalpos + annotationOffset, textcoords='data'))
            
        return handles
    


################################################################################

def find_all_cycles(G, source=None, cycle_length_limit=None):
    """forked from networkx dfs_edges function. Assumes nodes are integers, or at least
    types which work with min() and > ."""
    if source is None:
        # produce edges for all components
        nodes=G.nodes()
    else:
        # produce edges for components with source
        nodes=[source]
    # extra variables for cycle detection:
    cycle_stack = []
    output_cycles = set()
    
    def get_hashable_cycle(cycle):
        """cycle as a tuple in a deterministic order."""
        m = min(cycle)
        mi = cycle.index(m)
        mi_plus_1 = mi + 1 if mi < len(cycle) - 1 else 0
        if cycle[mi-1] > cycle[mi_plus_1]:
            result = cycle[mi:] + cycle[:mi]
        else:
            result = list(reversed(cycle[:mi_plus_1])) + list(reversed(cycle[mi_plus_1:]))
        return tuple(result)
    
    for start in nodes:
        if start in cycle_stack:
            continue
        cycle_stack.append(start)
        
        stack = [(start,iter(G[start]))]
        while stack:
            parent,children = stack[-1]
            try:
                child = next(children)
                
                if child not in cycle_stack:
                    cycle_stack.append(child)
                    stack.append((child,iter(G[child])))
                else:
                    i = cycle_stack.index(child)
                    if i < len(cycle_stack) - 2: 
                      output_cycles.add(get_hashable_cycle(cycle_stack[i:]))
                
            except StopIteration:
                stack.pop()
                cycle_stack.pop()
    
    return [list(i) for i in output_cycles]


################################################################################


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


def ShortcutTrajectory(traj, ccConfigFunction, ccSegmentFunction, numIter=100):
    """Shortcut a CD trajectory.

    ccConfigFunction: collision checking function for a configuration
        This function has template ccConfigFunction(config, activeIndices).
    
    ccSegmentFunction: collision checking function for a segment
        This function has template ccSegmentFunction(config1, config2, activeIndices)
    """
    nsuccessful = 0
    originalDuration = traj.duration
    newTraj = traj
    for i in xrange(numIter):
        T = newTraj.duration
        print "Shortcut iteration {0}:".format(i + 1),

        t0 = T * np.random.random_sample()
        t1 = T * np.random.random_sample()
        if t0 > t1:
            _t = t1
            t1 = t0
            t0 = _t

        c0 = Config(newTraj.Eval(t0))
        c1 = Config(newTraj.Eval(t1))
        if Distance(c0, c1) >= (t1 - t0):
            # Not shorter
            print "not shorter"
            continue

        if ccSegmentFunction(c0, c1, c0.activeIndices):
            # Segment in collision
            continue

        print "successful"
        nsuccessful += 1

        newSegment = CDPath.FromConfigsList([c0, c1])
        newTraj = Trajectory.InsertIntoTrajectory(newTraj, newSegment, t0, t1, order=1)

    print "Successful shortcuts = {0} times".format(nsuccessful)
    print "Original duration = {0}".format(originalDuration)
    print "     New duration = {0}".format(newTraj.duration)
    print "             diff = {0}".format(originalDuration - newTraj.duration)
    return newTraj
        
        
