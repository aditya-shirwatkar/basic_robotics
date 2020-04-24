cmake_minimum_required(VERSION 2.8.3)
project(pid_turtle)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  turtlesim
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_pose_hear_node src/listen.cpp)
add_executable(${PROJECT_NAME}_pose_send_node src/send.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_pose_hear_node
  ${catkin_LIBRARIES}
 )
target_link_libraries(${PROJECT_NAME}_pose_send_node
 ${catkin_LIBRARIES}
)
