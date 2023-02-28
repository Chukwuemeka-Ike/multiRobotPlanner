from itertools import combinations
from z3 import *

planHorizon = 60

# Set up the Z3 solver.
solver = Solver()

# Position variables for the robots.
# There are 3 robots, so we have x1-3 for each timestep.
numRobots = 4
X = [Ints(f"x1{i} x2{i} x3{i} x4{i}") for i in range(planHorizon)]

N = 3
robotIdx = range(numRobots)
combos = combinations(robotIdx, N)

# print(robotIdx)
# [print(i) for i in combos]
# print()

[print(j) for i in combos for j in i]
        