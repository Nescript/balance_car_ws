cmake_minimum_required(VERSION 3.0.2)
project(balance_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  control_toolbox
  sensor_msgs
  geometry_msgs
  filters
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp controller_interface hardware_interface pluginlib realtime_tools control_toolbox sensor_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/balance_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})