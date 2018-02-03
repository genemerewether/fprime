cmake_minimum_required(VERSION 2.8.3)
project(fprime)

find_package(catkin REQUIRED COMPONENTS genfprime message_generation roscpp
  std_msgs geometry_msgs sensor_msgs)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
  LANGS genfprime
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs roscpp
  CFG_EXTRAS genmodmk.cmake
)

install(DIRECTORY cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN ".svn" EXCLUDE)
