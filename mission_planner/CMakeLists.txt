cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

find_package(Boost COMPONENTS program_options REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

add_definitions(-std=c++11)# -m64) #-Wall)


###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

###########
## Build ##
###########
include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

link_directories(
	${Boost_LIBRARY_DIRS}
)

add_executable(mission_plan_node src/mission_plan_node.cpp)


target_link_libraries(
	mission_plan_node
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)


