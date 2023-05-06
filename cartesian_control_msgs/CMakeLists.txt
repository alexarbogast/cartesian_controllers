cmake_minimum_required(VERSION 3.4)
project(cartesian_control_msgs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    geometry_msgs
    message_generation
)

add_message_files(
    DIRECTORY msg
    FILES
        CartesianTrajectoryPoint.msg
        CartesianTrajectory.msg
        KinematicLimits.msg
)

add_action_files(
    DIRECTORY action
    FILES TrajectoryExecution.action
)

generate_messages(
    DEPENDENCIES
        actionlib_msgs
        geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
        actionlib_msgs
        geometry_msgs
        message_runtime
)