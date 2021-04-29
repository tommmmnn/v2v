cmake_minimum_required(VERSION 2.8.3)
project(driverless_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
    FILES
    BaseControlState.msg
    ControlCmd.msg
    PathTrackingInfo.msg
    SteerMotorDebug.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    message_runtime 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
