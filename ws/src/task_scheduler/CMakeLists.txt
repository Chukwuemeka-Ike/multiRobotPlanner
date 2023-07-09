cmake_minimum_required(VERSION 3.0.2)
project(task_scheduler)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_package()
catkin_python_setup()
catkin_install_python(
    PROGRAMS nodes/task_scheduler_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)