cmake_minimum_required(VERSION 3.0.2)
project(arm_gui)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    object_visualizer
)

catkin_package()
catkin_install_python(
    PROGRAMS scripts/supervisor_gui.py scripts/operator_gui.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)