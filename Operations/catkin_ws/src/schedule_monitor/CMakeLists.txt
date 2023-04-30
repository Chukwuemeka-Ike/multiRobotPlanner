cmake_minimum_required(VERSION 3.0.2)
project(schedule_monitor)

find_package(catkin REQUIRED COMPONENTS
    rospy
    # schedule_monitor_msgs
)

catkin_package()

catkin_install_python(
    PROGRAMS scripts/scheduleMonitor.py scripts/tester.py scripts/scheduleMonitorTester.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)