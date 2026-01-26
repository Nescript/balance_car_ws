# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;controller_interface;hardware_interface;pluginlib;realtime_tools;control_toolbox;sensor_msgs;geometry_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lbalance_controller".split(';') if "-lbalance_controller" != "" else []
PROJECT_NAME = "balance_controller"
PROJECT_SPACE_DIR = "/home/nesc/balance_car_ws/install"
PROJECT_VERSION = "0.0.0"
