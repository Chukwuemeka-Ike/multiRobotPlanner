cmake_minimum_required(VERSION 3.0.2)
project(ticket_manager)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS nodes/ticket_manager_node testing/add_tickets.py testing/ticket_manager_tester_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)