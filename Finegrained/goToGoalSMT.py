#!/usr/lib/python

# Rensselaer Polytechnic Institute - Julius Lab
# ARM Project
# Author - Chukwuemeka Osaretin Ike
# 
# Description:
# We formulate and solve the following MILP model:
# minimize
#             sum of input energy,
# subject to
#             system dynamics, and
#             MTL specification of the task.
# The optimization variables are - u1 u2 x1 x2 y1 y2 z1 z2 z3 z4.
# There is an optimization variable for each u1(k), u2(k), etc.
# i.e. for a horizon of 50, we have (10 variables) * (50 iterations) + (4
# states) = 504 variables.
# Variables are ordered [u11, u21, u12, u22, ..., x11, x21, y11, y21, ...].


from z3 import *
import numpy as np
import scipy.io
import time

# Simulation horizon for the variables.
u_horizon = 100
z_horizon = u_horizon
x_horizon = u_horizon + 1
max_accel = 5

# Sample step size.
dt = 0.1
bi = (dt**2) / 2

# Big M parameter.
M = -(10**10)

initPose = [0, 50, 0, 50]
obstacle = [60, 70, 40, 70]
station = [40, 60, 180, 220]
goal = [100, 120, 60, 100]
workspace = [0, 200, 0, 250]

# Create the optimization variables for the appropriate horizons.
U = [Reals('ux%s uy%s' % (i,i)) for i in range(u_horizon)]
X = [Reals('x1%s x2%s y1%s y2%s' % (i,i,i,i)) for i in range(x_horizon)]
# Z = [Ints('z1%s z2%s z3%s z4%s' % (i,i,i,i)) for i in range(z_horizon)]

# Set up the optimizer object
o = Solver()

# For any given k, we want:
# x2k + z1k*M < obstacle(0)
# x2k + z2k*M > obstacle(1)
# y2k + z3k*M < obstacle(2)
# y2k + z4k*M > obstacle(3)
# z1k + z2k + z3k + z4k <= 3

# Equality constraints for initial state.
o.add(X[0][0] == initPose[0])
o.add(X[0][1] == initPose[1])
o.add(X[0][2] == initPose[2])
o.add(X[0][3] == initPose[3])

# Equality constraints for the state transitions.
for k in range(u_horizon):

    # # Slack variables.
    # z1, z2, z3, z4 = Z[k]
    # # print(z1, z2, z3, z4)

    # Obstacle avoidance.
    padding = 2
    o.add(Or(
        X[k][1] < obstacle[0] - padding, 
        X[k][1] > obstacle[1] + padding, 
        X[k][3] < obstacle[2] - padding, 
        X[k][3] > obstacle[3] + padding
        ))
    # o.add(X[k][1] + z1*M < obstacle[0] - 1)
    # o.add(-X[k][1] + z2*M < -obstacle[1] - 1)
    # o.add(X[k][3] + z3*M < obstacle[2] - 1)
    # o.add(-X[k][3] + z4*M < -obstacle[3] - 1)

    # # o.add(AtMost(z1, z2, z3, z4, 3))
    # o.add(And(z1 <= 1, z2 <= 1, z3 <= 1, z4 <= 1))
    # o.add(And(z1 >= 0, z2 >= 0, z3 >= 0, z4 >= 0))
    # o.add(z1 + z2 + z3 + z4 <= 3)

    # Equality constraints for state transition.
    if k > 0:
        o.add( X[k][0] == X[k-1][0] + (dt*U[k-1][0]) )
        o.add( X[k][1] == (dt*X[k-1][0]) + (X[k-1][1]) + (bi*U[k-1][0]) )
        o.add( X[k][2] == X[k-1][2] + (dt*U[k-1][1]) )
        o.add( X[k][3] == (dt*X[k-1][2]) + X[k-1][3] + (bi*U[k-1][1]) )

    # Acceleration bounds.
    o.add(And(
            (U[k][0] > -max_accel), (U[k][0] < max_accel),
            (U[k][1] > -max_accel), (U[k][1] < max_accel))
        )

    # Position bounds.
    o.add( X[k][1] > workspace[0] )
    o.add( X[k][1] < workspace[1] )
    o.add( X[k][3] > workspace[2] )
    o.add( X[k][3] < workspace[3] )

# Final state.
# Relationship to previous state.
k = u_horizon
o.add( X[k][0] == X[k-1][0] + (dt*U[k-1][0]) )
o.add( X[k][1] == (dt*X[k-1][0]) + (X[k-1][1]) + (bi*U[k-1][0]) )
o.add( X[k][2] == X[k-1][2] + (dt*U[k-1][1]) )
o.add( X[k][3] == (dt*X[k-1][2]) + X[k-1][3] + (bi*U[k-1][1]) )

# Final pose.
o.add( X[-1][0] == 0)
o.add( X[-1][1] > goal[0])
o.add( X[-1][1] < goal[1])
o.add( X[-1][2] == 0)
o.add( X[-1][3] > goal[2])
o.add( X[-1][3] < goal[3])


startTime = time.time()
# Solve the constraints.
print(o.check())
m = o.model()
print(f"Runtime: {time.time() - startTime} seconds")

# Extract the path into an NP array.
path = []
path = np.zeros((x_horizon, 7))
for k in range(x_horizon):
    x1 = float(m[X[k][0]].numerator_as_long())/float(m[X[k][0]].denominator_as_long())
    x2 = float(m[X[k][1]].numerator_as_long())/float(m[X[k][1]].denominator_as_long())
    y1 = float(m[X[k][2]].numerator_as_long())/float(m[X[k][2]].denominator_as_long())
    y2 = float(m[X[k][3]].numerator_as_long())/float(m[X[k][3]].denominator_as_long())

    u1, u2, cost = 0, 0, 0
    if k < u_horizon-1:
        u1 = float(m[U[k][0]].numerator_as_long())/float(m[U[k][0]].denominator_as_long())
        u2 = float(m[U[k][1]].numerator_as_long())/float(m[U[k][1]].denominator_as_long())
        cost = ((u1**2) + (u2**2))

    path[k, :] = [x1, x2, y1, y2, u1, u2, cost]

print(path.shape)
scipy.io.savemat('./goToGoalSMT.mat', mdict={'path': path})
print('Path generated and saved.')