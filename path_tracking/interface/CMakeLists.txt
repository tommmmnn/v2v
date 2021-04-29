cmake_minimum_required(VERSION 2.8.3)
project(interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
	can2serial
	driverless_msgs
	gps_msgs
	roscpp
	std_msgs
	gps_msgs
	message_generation
)

add_service_files(
	FILES
	RecordPath.srv
	Driverless.srv
	DriverlessStatus.srv
)

generate_messages()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES interface
#  CATKIN_DEPENDS can2serial driverless_msgs roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/interface.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



