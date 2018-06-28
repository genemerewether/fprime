#!/bin/bash

. ./settings.bash
. ${STARTUP_DIR}/logging.bash

increment_boot_counter() {
  if ! (( $# == 1 )); then
    evr 0 "usage: crc_vote dest_var"
    return 1
  fi
  evr 6 "dest_var is $1" # debug the destination variable name

  # return 0 for boot counter in case we return early
  eval ${1}=0

  # TODO(mereweth) - test if BOOT_COUNTER_FILE var is empty?
  if ! [ -f "$BOOT_COUNTER_FILE" ]; then
    evr 1 "$BOOT_COUNTER_FILE missing"
    return 1
  fi

  evr 4 "$BOOT_COUNTER_FILE present"

  local BOOT_COUNTER_LOCAL=$(cat $BOOT_COUNTER_FILE)

  re='^[0-9]+$'
  if ! [[ $BOOT_COUNTER_LOCAL =~ $re ]] ; then
    evr 1 "$BOOT_COUNTER_LOCAL not numeric"
    return 1
  fi

  (( BOOT_COUNTER_LOCAL++ ))
  $ECHO_BIN $BOOT_COUNTER_LOCAL > $BOOT_COUNTER_FILE
  # return good boot counter
  eval ${1}=$BOOT_COUNTER_LOCAL

  return 0
}
