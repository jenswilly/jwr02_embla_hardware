cmake_minimum_required(VERSION 2.8.3)
project(embla_hardware)

find_package(catkin REQUIRED COMPONENTS 
	controller_manager
	hardware_interface 
	diagnostic_updater
	roslaunch
	roscpp
	sensor_msgs
	std_msgs
	message_generation)

find_package(Boost REQUIRED COMPONENTS chrono system)

add_message_files(
  FILES
  EmblaEMCUStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roboclaw
  CATKIN_DEPENDS diagnostic_updater hardware_interface roscpp sensor_msgs std_msgs message_runtime i2c_service
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_compile_options(-std=c++11)

# Library: roboclaw (used in embla_hardware_node)
add_library(roboclaw src/roboclaw/roboclaw_driver.cpp src/roboclaw/TimeoutSerial.cpp)

# Node: embla_hardware_node
add_executable(embla_hardware_node
  src/embla_hardware_node.cpp
  src/embla_hardware.cpp
  src/embla_diagnostics.cpp)
target_link_libraries( embla_hardware_node roboclaw ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies( embla_hardware_node 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS} 
)

## Node: diagnostics_led
add_executable( diagnostics_led src/diagnostics_led.cpp )
target_link_libraries( diagnostics_led ${catkin_LIBRARIES} )
add_dependencies(
	diagnostics_led
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
