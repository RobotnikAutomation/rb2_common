cmake_minimum_required(VERSION 2.8.3)
project(rb2_pad)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  robotnik_msgs
  roscpp
  sensor_msgs
  diagnostic_updater
)


add_service_files(DIRECTORY srv 
   FILES
    enable_disable_pad.srv
)


generate_messages(
   DEPENDENCIES
   robotnik_msgs sensor_msgs
 )


catkin_package(

  CATKIN_DEPENDS 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    message_runtime
    diagnostic_updater

)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)


add_executable(rb2_pad src/rb2_pad.cpp)

add_dependencies(rb2_pad rb2_pad_generate_messages_cpp robotnik_msgs_generate_messages_cpp)

target_link_libraries(rb2_pad
   ${catkin_LIBRARIES}
 )

install(
  TARGETS rb2_pad
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install( DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)


