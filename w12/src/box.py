import numpy as np
from interval import interval as iv
from constants import EPSILON

class Box(object):
    """
    Attributes
    ----------
    interval : dictionary
        keys: robot indices; values: corresponding intervals (pyinterval instances).
    
    """
    def __init__(self, intervals=dict()):
        self.intervals = intervals.copy()
        self.indices = self.intervals.keys()

    @staticmethod
    def FromLists(indices, intervals):
        """Create a Box from a list of robot indices and a list of corresponding
        intervals.

        Note: intervals is a list of pyinterval instances.

        """
        intervalsdict = dict()
        for (index, interval) in zip(indices, intervals):
            intervalsdict[index] = interval
        return Box(intervalsdict)
        

    def Intersect(self, other):
        """Compute the intersection of self and other.
        """
        intervals = dict()
        allkeys = list(set(self.intervals.keys() + other.intervals.keys()))
        for k in allkeys:
            if k in self.intervals and k in other.intervals:
                interval = self.intervals[k] & other.intervals[k]
                # if len(interval) == 0:
                #     return Box()
                intervals[k] = interval
            else:
                if k in self.intervals:
                    intervals[k] = self.intervals[k]
                else:
                    intervals[k] = other.intervals[k]
        return Box(intervals)


    def IntersectOpen(self, other):
        """Compute the intersection of int(self) and int(other), where int(X) is the
        interior of the set X. This function is necessary when computing the
        intersection of shadows as we don't count shadow that only `touch` each
        other as intersecting each other.

        """        
        intervals = dict()
        allkeys = list(set(self.intervals.keys() + other.intervals.keys()))
        for k in allkeys:
            if k in self.intervals and k in other.intervals:
                interval = self.intervals[k] & other.intervals[k]
                if len(interval) == 0:
                    # return Box()
                    pass
                elif abs(interval[0][0] - interval[0][1]) <= EPSILON:
                    # Two boxes only touch each other.
                    # return Box()
                    interval = iv()
                intervals[k] = interval
            else:
                if k in self.intervals:
                    intervals[k] = self.intervals[k]
                else:
                    intervals[k] = other.intervals[k]
        return Box(intervals)


    def IsEmpty(self):
        return any([len(interval) == 0 for interval in self.intervals.values()])
        # return len(self.intervals) == 0


    def GenerateShadow(self, direction):
        """Generate a shadow of this box as if there is a light source placed at inf in
        the direction `direction` pointing towards -inf.

        """
        assert direction in self.indices

        newmax = self.intervals[direction][0][0]
        assert newmax > EPSILON

        newinterval = iv([0.0, newmax])
        newintervals = self.intervals.copy()
        newintervals[direction] = newinterval
        return Box(newintervals)
        

    def __repr__(self):        
        return self.intervals.__repr__()
