cmake_minimum_required(VERSION 2.8.3)
project(astar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
  roscpp
  std_msgs
)
catkin_package(
  LIBRARIES astar
  CATKIN_DEPENDS std_msgs roscpp
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(astar_path src/PathGen.cpp)
add_dependencies(astar_path ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
target_link_libraries(
  astar_path
  ${catkin_LIBRARIES}
)
