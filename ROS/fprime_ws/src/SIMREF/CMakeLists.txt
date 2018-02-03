cmake_minimum_required(VERSION 2.8.3)
project(SIMREF)

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

FILE(
    WRITE ${CMAKE_BINARY_DIR}/modmk.cmake
"
string (REPLACE \";\" \" \\\\ \\n\\t\" incs \"${catkin_INCLUDE_DIRS}\")\n
string (REPLACE \";\" \" \\\\ \\n\\t\" libs \"${catkin_LIBRARIES}\")\n

FILE(WRITE \${MODMK} \"INC = \")
FILE(APPEND \${MODMK} \${incs})
FILE(APPEND \${MODMK} \"\\nLIB = \")
FILE(APPEND \${MODMK} \${libs})
"
)

ADD_CUSTOM_COMMAND(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/mod.mk
    COMMAND ${CMAKE_COMMAND}
            -DMODMK=${CMAKE_CURRENT_SOURCE_DIR}/mod.mk
            -P ${CMAKE_BINARY_DIR}/modmk.cmake
    DEPENDS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
ADD_CUSTOM_TARGET(modmk ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/mod.mk)
