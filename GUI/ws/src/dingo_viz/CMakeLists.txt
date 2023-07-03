cmake_minimum_required(VERSION 3.0.2)
project(dingo_viz)

find_package(catkin REQUIRED)

catkin_package()


install(DIRECTORY launch rviz rqt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/view_model.launch)
  roslaunch_add_file_check(launch/view_robot.launch)
  roslaunch_add_file_check(launch/view_diagnostics.launch)
endif()
