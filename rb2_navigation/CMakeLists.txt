cmake_minimum_required(VERSION 2.8.3)
project(rb2_navigation)

find_package(catkin REQUIRED COMPONENTS  
  move_base
  teb_local_planner
	costmap_prohibition_layer
)


catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

