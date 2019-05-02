cmake_minimum_required(VERSION 2.8.3)
project(fprime)

find_package(catkin REQUIRED COMPONENTS
  genfprime message_generation roscpp
  std_msgs geometry_msgs sensor_msgs rosgraph_msgs
  mav_msgs mav_planning_msgs
  ackermann_msgs
  fprime_msgs
  actionlib
  trac_ik_lib
  image_transport
  tf
  )
find_package(orocos_kdl)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs geometry_msgs sensor_msgs rosgraph_msgs
  mav_msgs mav_planning_msgs
  ackermann_msgs
  fprime_msgs
  trac_ik_lib
  image_transport
  roscpp
  tf
)

genmodmk()
