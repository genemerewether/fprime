cmake_minimum_required(VERSION 2.8.3)
project(SDREF)

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

# TODO(mereweth) - support simultaneous builds with multiple targets
FILE(
    WRITE ${CMAKE_BINARY_DIR}/modmk.cmake
"
string (REPLACE \";\" \" \\\\ \\n\\t-I \" incs \"${catkin_INCLUDE_DIRS}\")
string (REPLACE \";\" \" \\\\ \\n\\t\" libs \"${catkin_LIBRARIES}\")

FILE(APPEND \${MODMK} \"EXTRA_INC_DIRS = -I \")
FILE(APPEND \${MODMK} \${incs})

FILE(APPEND \${MODMK} \"\\nEXTRA_LIBS = \")
FILE(APPEND \${MODMK} \${libs})
FILE(APPEND \${MODMK} \"\\n\")
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
