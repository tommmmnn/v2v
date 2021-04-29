cmake_minimum_required(VERSION 2.8.3)
project(usbcan_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  can_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES usbcan_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  lib
  ${catkin_INCLUDE_DIRS}
)
# message("${catkin_INCLUDE_DIRS}........................................")

link_directories(lib)

add_executable(${PROJECT_NAME}_node src/usbcan_driver.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} controlcan)

