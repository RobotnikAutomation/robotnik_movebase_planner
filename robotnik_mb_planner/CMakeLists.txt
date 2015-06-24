cmake_minimum_required(VERSION 2.8.3)
project(robotnik_mb_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  diagnostic_updater
  robotnik_mb_msgs
  interactive_markers
  visualization_msgs
  std_srvs
  actionlib
  move_base_msgs
)


catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES movebase_planner
  CATKIN_DEPENDS roscpp actionlib nav_msgs roscpp std_msgs tf geometry_msgs move_base_msgs robotnik_mb_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(robotnik_mb_planner_node src/movebase_planner.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(robotnik_mb_planner_node movebase_planner_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robotnik_mb_planner_node
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(PROGRAMS
   scripts/movebase_planner/path_marker.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
 )


