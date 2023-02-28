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






# x2k + z5k*M > station(0)
# x2k + z5k*M < station(1)
# y2k + z5k*M > station(2)
# y2k + z5k*M < station(3)