cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planner_msgs)

set(MSG_DEPS
    std_msgs
    geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    ${MSG_DEPS}
)

set(MSG_FILES
    Desired_Locations.msg
)

add_message_files(
    DIRECTORY msg
    FILES ${MSG_FILES}
)

generate_messages(
    DEPENDENCIES ${MSG_DEPS}
)

catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})