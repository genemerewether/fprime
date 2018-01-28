@[if DEVELSPACE]@
# location of scripts in develspace
set(GENFPRIME_BIN_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[else]@
# location of scripts in installspace
set(GENFPRIME_BIN_DIR "${genfprime_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)")
@[end if]@

set(GENFPRIME_BIN ${GENFPRIME_BIN_DIR}/gen_fprime.py)

# Generate FPrime Serialiazable xml from .msg or .srv
# The generated xml files should be added ALL_GEN_OUTPUT_FILES_fprime
macro(_generate_fprime ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  file(MAKE_DIRECTORY ${ARG_GEN_OUTPUT_DIR})

  #Create input and output filenames
  get_filename_component(MSG_NAME ${ARG_MSG} NAME)
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  set(MSG_GENERATED_NAME ${MSG_SHORT_NAME}SerializableAi.xml)
  set(GEN_OUTPUT_FILE ${GEN_OUTPUT_DIR}/${MSG_GENERATED_NAME})

  assert(CATKIN_ENV)
  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    DEPENDS ${GENFPRIME_BIN} ${ARG_MSG} ${ARG_MSG_DEPS}
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENFPRIME_BIN} ${ARG_MSG}
    ${ARG_IFLAGS}
    -p ${ARG_PKG}
    -o ${ARG_GEN_OUTPUT_DIR}
    COMMENT "Generating FPrime XML from MSG ${ARG_PKG}/${MSG_SHORT_NAME}"
    )

  list(APPEND ALL_GEN_OUTPUT_FILES_fprime ${GEN_OUTPUT_FILE})

endmacro()

#genfprime uses the same program to generate srv and msg files, so call the same macro
macro(_generate_msg_fprime ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  _generate_fprime(${ARG_PKG} ${ARG_MSG} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" "${ARG_GEN_OUTPUT_DIR}/msg")
endmacro()

#genfprime uses the same program to generate srv and msg files, so call the same macro
macro(_generate_srv_fprime ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  _generate_fprime(${ARG_PKG} ${ARG_MSG} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" "${ARG_GEN_OUTPUT_DIR}/srv")
endmacro()

macro(_generate_module_fprime ARG_PKG ARG_GEN_OUTPUT_DIR ARG_GENERATED_FILES)
  message(STATUS ARG_GENERATED_FILES: ${ARG_GENERATED_FILES})
  if(NOT EXISTS ${ARG_GEN_OUTPUT_DIR}/mod.mk)
    file(WRITE ${ARG_GEN_OUTPUT_DIR}/mod.mk "SUBDIRS = msg srv\n")
  endif()

  #Append msg to output dir
  foreach(type "msg" "srv")
    set(GEN_OUTPUT_DIR "${ARG_GEN_OUTPUT_DIR}/${type}")
    set(GEN_OUTPUT_FILE ${GEN_OUTPUT_DIR}/mod.mk)

    if(IS_DIRECTORY ${GEN_OUTPUT_DIR})
      add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
        DEPENDS ${GENFPRIME_BIN} ${ARG_GENERATED_FILES}
        COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENFPRIME_BIN}
        -o ${GEN_OUTPUT_DIR}
        --modmk
        COMMENT "Generating Fprime ${type} mod.mk for ${ARG_PKG}")
      list(APPEND ALL_GEN_OUTPUT_FILES_fprime ${GEN_OUTPUT_FILE})
    endif()

  endforeach()
endmacro()

if(NOT EXISTS @(PROJECT_NAME)_SOURCE_DIR)
  set(genfprime_INSTALL_DIR share/genfprime/ros)
endif()
