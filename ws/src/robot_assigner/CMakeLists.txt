cmake_minimum_required(VERSION 3.0.2)
project(robot_assigner)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS nodes/robot_assigner_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)