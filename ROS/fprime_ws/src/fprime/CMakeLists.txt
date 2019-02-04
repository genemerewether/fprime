cmake_minimum_required(VERSION 2.8.3)
project(fprime)

find_package(catkin REQUIRED COMPONENTS
  genfprime message_generation roscpp
  std_msgs geometry_msgs sensor_msgs rosgraph_msgs
  mav_msgs mav_planning_msgs
  actionlib_msgs actionlib
  trac_ik_lib
  )
find_package(orocos_kdl)

#generate_messages(
#  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
#  LANGS genfprime
#)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs geometry_msgs sensor_msgs rosgraph_msgs
  mav_msgs mav_planning_msgs
  actionlib_msgs
  trac_ik_lib
  roscpp
)

genmodmk()
