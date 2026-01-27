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
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  filters
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp controller_interface hardware_interface pluginlib realtime_tools control_toolbox sensor_msgs geometry_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/balance_controller.cpp
  src/filters/filters.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})