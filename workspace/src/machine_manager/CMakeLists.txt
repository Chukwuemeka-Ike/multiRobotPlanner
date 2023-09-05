cmake_minimum_required(VERSION 3.0.2)
project(machine_manager)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS nodes/machine_manager_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)