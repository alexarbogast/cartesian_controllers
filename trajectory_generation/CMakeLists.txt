cmake_minimum_required(VERSION 3.4)
project(trajectory_generation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ruckig REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cartesian_control_msgs
	cartesian_trajectory
)

catkin_package(
	INCLUDE_DIRS include 
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
		roscpp
		cartesian_control_msgs
		cartesian_trajectory
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(${ruckig_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/trajectory_generator.cpp)
target_link_libraries(${PROJECT_NAME} ruckig::ruckig ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

## Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)