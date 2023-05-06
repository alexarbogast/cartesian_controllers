cmake_minimum_required(VERSION 3.4)
project(cartesian_trajectory_adapter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
	controller_manager
	hardware_interface
	pluginlib
	actionlib
	roscpp
	cartesian_control_msgs
	cartesian_trajectory
	trajectory_generation
)

catkin_package(
	INCLUDE_DIRS include 
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
		controller_manager
		hardware_interface
		pluginlib
		actionlib
		roscpp
		cartesian_control_msgs
		cartesian_trajectory
		trajectory_generation
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/cartesian_trajectory_adapter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})