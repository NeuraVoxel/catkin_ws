cmake_minimum_required(VERSION 3.0.2)
project(rtk_bridge)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

## catkin specific configuration
catkin_package()

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/gga_publisher.py
  scripts/rtcm_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

## Mark other files for installation (e.g. launch files)
# install(FILES
#   launch/rtk_bridge.launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
