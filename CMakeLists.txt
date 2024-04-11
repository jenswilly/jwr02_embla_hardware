cmake_minimum_required(VERSION 3.8)
project(embla_hardware)

# C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find and dependencies for all node executables
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono system)

set(common_dependencies "rclcpp" "geometry_msgs" "Boost")

include_directories(include/embla_hardware include/roboclaw include)

# Generate interfaces
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EmblaEMCUStatus.msg"
  DEPENDENCIES std_msgs
)

# Add executable and link dependencies
add_executable(embla_hardware_node src/embla_hardware_node.cpp src/roboclaw/roboclaw_driver.cpp src/roboclaw/TimeoutSerial.cpp)
ament_target_dependencies(embla_hardware_node ${common_dependencies})

# Add custom types
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(embla_hardware_node "${cpp_typesupport_target}")

# Install executables in lib
install(
	TARGETS embla_hardware_node
	DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_package()

# --------------------------------
#
# OLD sutff below
# cmake_minimum_required(VERSION 2.8.3)
# project(embla_hardware)

# find_package(catkin REQUIRED COMPONENTS 
# 	controller_manager
# 	hardware_interface 
# 	diagnostic_updater
# 	roslaunch
# 	roscpp
# 	sensor_msgs
# 	std_msgs
# 	message_generation
# 	kdl_parser
# 	robot_state_publisher)

# find_package(Boost REQUIRED COMPONENTS chrono system)
# find_package(orocos_kdl REQUIRED)

# add_message_files(
#   FILES
#   EmblaEMCUStatus.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES roboclaw
#   CATKIN_DEPENDS diagnostic_updater hardware_interface roscpp sensor_msgs std_msgs message_runtime i2c_service robot_state_publisher
#   DEPENDS Boost
# )

# include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add_compile_options(-std=c++11)

# # Library: roboclaw (used in embla_hardware_node)
# add_library(roboclaw src/roboclaw/roboclaw_driver.cpp src/roboclaw/TimeoutSerial.cpp)

# # Node: embla_hardware_node
# add_executable(embla_hardware_node
#   src/embla_hardware_node.cpp
#   src/embla_hardware.cpp
#   src/embla_diagnostics.cpp)
# target_link_libraries( embla_hardware_node roboclaw ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
# add_dependencies( embla_hardware_node 
# 	${${PROJECT_NAME}_EXPORTED_TARGETS}
# 	${catkin_EXPORTED_TARGETS} 
# )

# ## Node: diagnostics_led
# add_executable( diagnostics_led src/diagnostics_led.cpp )
# target_link_libraries( diagnostics_led ${catkin_LIBRARIES} )
# add_dependencies(
# 	diagnostics_led
# 	${${PROJECT_NAME}_EXPORTED_TARGETS}
# 	${catkin_EXPORTED_TARGETS}
# )
