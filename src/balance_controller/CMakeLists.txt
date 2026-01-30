cmake_minimum_required(VERSION 3.0.2)
project(balance_controller)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# 显式查找 Eigen3
find_package(Eigen3 REQUIRED)
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
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/balance_controller.cpp
  src/filters/filters.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(keyboard_control_node src/keyboard_control_node.cpp)
target_link_libraries(keyboard_control_node ${catkin_LIBRARIES})

install(TARGETS keyboard_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)