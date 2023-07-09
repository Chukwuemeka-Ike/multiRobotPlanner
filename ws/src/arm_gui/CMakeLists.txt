cmake_minimum_required(VERSION 3.0.2)
project(arm_gui)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS nodes/supervisor_gui_node.py nodes/operator_gui_node.py nodes/tester.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)