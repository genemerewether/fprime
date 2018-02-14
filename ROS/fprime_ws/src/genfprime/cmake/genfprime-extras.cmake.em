@[if DEVELSPACE]@
# location of scripts in develspace
set(GENFPRIME_BIN_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
set(GENFPRIME_SRC_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/src/genfprime")
@[else]@
# location of scripts in installspace
set(GENFPRIME_BIN_DIR "${genfprime_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)")
# TODO(mereweth) - check this Python src installed dir
set(GENFPRIME_SRC_DIR "${genfprime_DIR}/../../../@(CATKIN_PACKAGE_PYTHON_DESTINATION)")
@[end if]@

set(GEN_OUTPUT_BASE_DIR @(CMAKE_CURRENT_SOURCE_DIR)/../../../Gen/)
set(GENFPRIME_BIN ${GENFPRIME_BIN_DIR}/gen_fprime.py)
set(GENFPRIME_SRC
  ${GENFPRIME_SRC_DIR}/__init__.py
  ${GENFPRIME_SRC_DIR}/generate_modmk.py
  ${GENFPRIME_SRC_DIR}/generator.py
  ${GENFPRIME_SRC_DIR}/genfprime_main.py
  ${GENFPRIME_SRC_DIR}/name.py
  ${GENFPRIME_SRC_DIR}/types.py
)

# Generate FPrime Serializable and Port xml from .msg or .srv
# The generated xml files should be added ALL_GEN_OUTPUT_FILES_fprime
macro(_generate_fprime ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_LOCAL_GEN_OUTPUT_DIR ARG_LOCAL_GEN_OUTPUT_FILES)
  file(MAKE_DIRECTORY ${ARG_LOCAL_GEN_OUTPUT_DIR}/Ports)
  file(MAKE_DIRECTORY ${ARG_LOCAL_GEN_OUTPUT_DIR}/Types)

  #Create input and output filenames
  #get_filename_component(MSG_NAME ${ARG_MSG} NAME)
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  assert(CATKIN_ENV)
  add_custom_command(OUTPUT ${ARG_LOCAL_GEN_OUTPUT_FILES}
    DEPENDS ${GENFPRIME_BIN} ${GENFPRIME_SRC} ${ARG_MSG} ${ARG_MSG_DEPS}
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENFPRIME_BIN} ${ARG_MSG}
    ${ARG_IFLAGS}
    -p ${ARG_PKG}
    -o ${ARG_LOCAL_GEN_OUTPUT_DIR}
    COMMENT "Generating FPrime XML from MSG ${ARG_PKG}/${MSG_SHORT_NAME}"
    )

  list(APPEND ALL_GEN_OUTPUT_FILES_fprime ${ARG_LOCAL_GEN_OUTPUT_FILES})

endmacro()

#genfprime uses the same program to generate srv and msg files, so call the same macro with different output files
macro(_generate_srv_fprime ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)

  set(GEN_OUTPUT_DIR ${GEN_OUTPUT_BASE_DIR}/${ARG_PKG})

  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)
  set(GEN_OUTPUT_FILES
    ${GEN_OUTPUT_DIR}/Types/${MSG_SHORT_NAME}InSerializableAi.xml
    ${GEN_OUTPUT_DIR}/Types/${MSG_SHORT_NAME}OutSerializableAi.xml
    ${GEN_OUTPUT_DIR}/Ports/${MSG_SHORT_NAME}PortAi.xml
  )

  _generate_fprime(${ARG_PKG} ${ARG_MSG} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" "${GEN_OUTPUT_DIR}" "${GEN_OUTPUT_FILES}")
endmacro()

#genfprime uses the same program to generate srv and msg files, so call the same macro with different output files
macro(_generate_msg_fprime ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)

  set(GEN_OUTPUT_DIR ${GEN_OUTPUT_BASE_DIR}/${ARG_PKG})

  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)
  set(GEN_OUTPUT_FILES
    ${GEN_OUTPUT_DIR}/Types/${MSG_SHORT_NAME}SerializableAi.xml
    ${GEN_OUTPUT_DIR}/Ports/${MSG_SHORT_NAME}PortAi.xml
  )

  _generate_fprime(${ARG_PKG} ${ARG_MSG} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" "${GEN_OUTPUT_DIR}" "${GEN_OUTPUT_FILES}")
endmacro()

macro(_generate_module_fprime ARG_PKG ARG_GEN_OUTPUT_DIR ARG_GENERATED_FILES)
  message(STATUS ARG_GENERATED_FILES: ${ARG_GENERATED_FILES})
  if(NOT EXISTS ${GEN_OUTPUT_BASE_DIR}/${ARG_PKG}/mod.mk)
    file(WRITE ${GEN_OUTPUT_BASE_DIR}/${ARG_PKG}/mod.mk "SUBDIRS = Ports Types\n")
  endif()

  #Append msg to output dir
  foreach(type "Ports" "Types")
    set(GEN_OUTPUT_DIR ${GEN_OUTPUT_BASE_DIR}/${ARG_PKG}/${type})
    set(GEN_OUTPUT_FILE ${GEN_OUTPUT_DIR}/mod.mk)

    if(NOT EXISTS ${GEN_OUTPUT_DIR}/Makefile)
      file(WRITE ${GEN_OUTPUT_DIR}/Makefile "MODULE_DIR = ROS/Gen/${ARG_PKG}/${type}\n")
      file(APPEND ${GEN_OUTPUT_DIR}/Makefile "MODULE = $(subst /,,$(MODULE_DIR))\n")
      file(APPEND ${GEN_OUTPUT_DIR}/Makefile "BUILD_ROOT ?= $(subst /$(MODULE_DIR),,$(CURDIR))\n")
      file(APPEND ${GEN_OUTPUT_DIR}/Makefile "export BUILD_ROOT\n")
      file(APPEND ${GEN_OUTPUT_DIR}/Makefile "include $(BUILD_ROOT)/mk/makefiles/module_targets.mk\n")
    endif()

    if(IS_DIRECTORY ${GEN_OUTPUT_DIR})
      add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
        DEPENDS ${GENFPRIME_BIN} ${GENFPRIME_SRC} ${ARG_GENERATED_FILES}
        COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENFPRIME_BIN}
        -o ${GEN_OUTPUT_DIR}
        --modmk
        COMMENT "Generating Fprime ${type} mod.mk for ${ARG_PKG}")
      list(APPEND ALL_GEN_OUTPUT_FILES_fprime ${GEN_OUTPUT_FILE} ${GEN_OUTPUT_DIR}/Makefile)
    endif()

  endforeach()
endmacro()

if(NOT EXISTS @(PROJECT_NAME)_SOURCE_DIR)
  set(genfprime_INSTALL_DIR share/genfprime/ros)
endif()
