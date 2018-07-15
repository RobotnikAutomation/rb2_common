cmake_minimum_required(VERSION 2.8.3)
project(rb2_navigation)

find_package(catkin REQUIRED COMPONENTS  
  move_base
  teb_local_planner
)


catkin_package()

#############
## Install ##
#############

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

