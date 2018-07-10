cmake_minimum_required(VERSION 2.8.3)
project(rb2_localization)

find_package(catkin REQUIRED
  geometry_msgs
  mavros_msgs
  message_filters
  robot_localization
  robotnik_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rb1_base_localization
  CATKIN_DEPENDS 
    geometry_msgs 
    mavros_msgs 
    message_filters 
    robot_localization 
    robotnik_msgs 
    roscpp 
    std_msgs 
    std_srvs 
    tf 
    tf2 
    tf2_geometry_msgs 
    tf2_ros
#  DEPENDS system_lib
)



#############
## Install ##
#############

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
