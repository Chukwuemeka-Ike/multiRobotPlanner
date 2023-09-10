cmake_minimum_required(VERSION 3.0.2)
project(arm_msgs)

set(MSG_DEPS
    std_msgs
)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    ${MSG_DEPS}
)

set(MSG_FILES 
    IntList.msg
    RobotEnableStatus.msg
    StringList.msg
    Ticket.msg
    Tickets.msg
)

add_message_files(
    DIRECTORY msg 
    FILES ${MSG_FILES}
)

add_service_files(
    FILES
    FleetInformation.srv
    MachinesOverview.srv
    MachineStatus.srv
    RobotAssignments.srv
    RobotReplacement.srv
    Schedule.srv
    TicketList.srv
    UnboundMachines.srv
)

generate_messages(
    DEPENDENCIES ${MSG_DEPS}
)

catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})