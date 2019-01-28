cmake_minimum_required(VERSION 2.8.3)
project(embla_hardware)

find_package(catkin REQUIRED COMPONENTS 
	controller_manager
	hardware_interface 
	diagnostic_updater
	roslaunch
	roscpp
	sensor_msgs
	serial)

find_package(Boost REQUIRED COMPONENTS chrono system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roboclaw
  CATKIN_DEPENDS diagnostic_updater hardware_interface roscpp sensor_msgs serial
  DEPENDS Boost system_lib
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_compile_options(-std=c++11)

add_library(roboclaw src/roboclaw/roboclaw.cpp)

add_executable(embla_hardware_node
  src/embla_hardware_node.cpp
  src/embla_hardware.cpp)

target_link_libraries(embla_hardware_node roboclaw ${catkin_LIBRARIES} ${Boost_LIBRARIES})
