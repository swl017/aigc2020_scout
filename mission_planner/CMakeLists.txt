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

install(TARGETS mission_plan_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY mission_csv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mission_csv)