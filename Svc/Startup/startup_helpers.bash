#!/bin/bash

. ${STARTUP_DIR}/logging.bash

# Substitute in boot counter variable, or 0 if blank
expand_fsw_args() {
  if ! (( $# == 2 )); then
    evr 0 "usage: expand_fsw_args dest_var src_var"
    return 1
  fi
  evr 6 "dest_var is $1, src_var is $2" # debug the variable names

  evr 5 "src_var ${!2}" # diagnostic before expansion

  eval "${1}=\"${!2} ${BOOT_COUNTER_RTN:-0}\""

  evr 5 "dest_var ${!1}" # diagnostic after expansion

  return 0
}
