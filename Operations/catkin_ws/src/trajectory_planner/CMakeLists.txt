cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planner)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_package()

catkin_install_python(
    PROGRAMS scripts/trajectoryPlanner.py scripts/trajectoryPlannerTester.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)