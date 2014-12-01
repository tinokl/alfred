cmake_minimum_required(VERSION 2.8.3)
project(ra1_pro_gripper_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  std_msgs
  sensor_msgs
##  baxter_core_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #   INCLUDE_DIRS include
  #  LIBRARIES baxter_gripper_action
  CATKIN_DEPENDS
  actionlib
  control_msgs
  roscpp
  std_msgs
  sensor_msgs
 ## baxter_core_msgs
  #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(gripper_action_server src/gripper_action_server.cpp)
add_dependencies(gripper_action_server ${catkin_EXPORTED_TARGETS}) # don't build until necessary msgs are finished

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(gripper_action_server ra1_pro_gripper_action_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gripper_action_server ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

