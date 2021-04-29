cmake_minimum_required(VERSION 2.8.3)
project(driverless_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
    FILES
    Lane.msg
    ControlCmd.msg
    Point.msg
    Points.msg
    DrawArea.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
