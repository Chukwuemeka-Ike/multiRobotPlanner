# T1 and T2 completely separate. This setup separates T1 and T2 completely.
# It makes it so that if we want robots to transport something somewhere and
# then stay, we have no guarantee that the same set would stay.
# There's also the possibility that a group goes there before the item is even
# transported there.


# # Job 1.
# singleTime = time.time()

# numVisitors = 3

# # Cutter -> Mega Stitch.
# n_robots_transport(solver, X, Oc, 0, 5, numVisitors, Cutter, Mega_Stitch)

# # Mega Stitch for 10.
# n_robots_visit_station_for_duration(solver, X, Oc, 0, 15, numVisitors, Mega_Stitch, 10)

# # Mega Stitch -> Long Arm.
# n_robots_transport(solver, X, Oc, 15, 20, numVisitors, Mega_Stitch, Long_Arm)

# # Long Arm for 15.
# n_robots_visit_station_for_duration(solver, X, Oc, 15, 35, numVisitors, Long_Arm, 15)

# # Long Arm -> Grommet.
# n_robots_transport(solver, X, Oc, 35, 40, numVisitors, Long_Arm, Grommet)

# # Grommet for 10.
# n_robots_visit_station_for_duration(solver, X, Oc, 35, 50, numVisitors, Grommet, 10)

# # Grommet -> Mega Stitch.
# n_robots_transport(solver, X, Oc, 45, 50, numVisitors, Long_Arm, Grommet)

# # Mega Stitch for 10.
# n_robots_visit_station_for_duration(solver, X, Oc, 45, 60, numVisitors, Mega_Stitch, 10)

# print(f"Job 1 setup runtime: {time.time() - singleTime} seconds.")