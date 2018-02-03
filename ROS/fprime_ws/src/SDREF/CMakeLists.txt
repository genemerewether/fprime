cmake_minimum_required(VERSION 2.8.3)
project(SDREF)

#TODO(mereweth) - something is funky about the message dependencies

find_package(catkin REQUIRED COMPONENTS
  genfprime message_generation fprime
  mav_msgs planning_msgs
  # any new message packages here
)

#generate_messages(
#  DEPENDENCIES # any new message packages here
#  LANGS genfprime
#)

catkin_package(
  CATKIN_DEPENDS
  message_runtime fprime
  mav_msgs planning_msgs
  # any new message packages here
)

genmodmk()
