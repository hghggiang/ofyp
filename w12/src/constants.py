"""Constants used in various places, listed alphabetically.

"""

# Computational error tolerance
EPSILON = 1e-6

# Scaling factor for map's node locations. For more information see
# comments svg2txt.py
MULT = 100.

# Maximum number of robots allowed in planning. We only need this when
# assigning a color to a robot when plotting it.
NMAXROBOTS = 10

# Seed for a random number generator. A seed should be controlled so
# that the planning process is reproducible.
RANDSEED = 0

# Minimum angle between two physical path segments that are adjacent
# to a deadlock/shared segment. This angle is used to estimate the
# clearange between two robots. The lesser the angle, the farther
# before the deadlock one robot needs to wait before entering the
# deadlock.
ALPHA = 0.5235987755982988 # arbitrarily set as PI/6 = 30 degree
# ALPHA = 1.0471975511965976
