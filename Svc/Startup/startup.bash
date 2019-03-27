#!/bin/bash

. ./settings.bash
. ${STARTUP_DIR}/deployment.bash
. ${STARTUP_DIR}/boot_counter.bash
. ${STARTUP_DIR}/hash_validation.bash
. ${STARTUP_DIR}/startup_helpers.bash
. ${STARTUP_DIR}/logging.bash

#TODO(mereweth) - set -e? if so, then make sure each command returns true

startup_signify_action &
pre_deployment &

#TODO(mereweth) - save core dumps?
rm ${CORE_DUMP_DIR}/${CORE_DUMP_PATTERN_BASE}* 2> /dev/null || true

increment_boot_counter BOOT_COUNTER_RTN
if ! (( $? == 0 )); then
  $ECHO_BIN "0" > $BOOT_COUNTER_FILE
  evr 1 "reset boot counter to 0"
fi
evr 3 "running boot $BOOT_COUNTER_RTN"

# source and destination variables can be the same.
# User can choose any name for the source variable, but dest variable must be FSW_ARGS
# This line sets the destination variable to: source variable followed by boot counter
expand_fsw_args FSW_ARGS FSW_ARGS

# left-pad boot counter with zeros
#BOOT_COUNTER_RTN=$(printf %05d $BOOT_COUNTER_RTN)

# take boot counter modulo 10
BOOT_COUNTER_RTN=$(($BOOT_COUNTER_RTN % 10))

#TODO(mereweth) - how long can we wait for user to put in magic cookie? vs. how long will Ryan wait?

$SLEEP_BIN $INHIBIT_COOKIE_TIMEOUT
if [ -f "$INHIBIT_COOKIE" ]; then
  rm $INHIBIT_COOKIE 2> /dev/null || evr 1 "failed to remove $INHIBIT_COOKIE"
  evr 1 "$INHIBIT_COOKIE present; not running FSW"
  fail_signify_action &
  wait $!
  while [ 1 ]; do
    $SLEEP_BIN 1 &
    wait $!
  done
  exit 0
fi
# Trying to make sure there is no way the inhibit cookie persists, even if there is a bug above
rm $INHIBIT_COOKIE 2> /dev/null || evr 4 "$INHIBIT_COOKIE not present"

up_signify_action &

############################# Try to run RUN_ONCE #############################
evr 3 "checking RUN_ONCE $DEPLOYMENT"
get_valid_bin BIN_NAME_DEST $RUN_ONCE_SYMLINK $DEPLOYMENT
if (( $? == 0 )); then
  # have to follow the symlink and store that path so we can delete the symlink pre-run
  BIN_DEREF=$($READLINK $BIN_NAME_DEST)
  evr 4 "dereferenced $BIN_NAME_DEST"
  evr 4 "    to $BIN_DEREF"
  rm $RUN_ONCE_SYMLINK 2> /dev/null || evr 1 "failed to remove $RUN_ONCE_SYMLINK"
  evr 3 "running RUN_ONCE $DEPLOYMENT"
  $SCREEN_BIN -D -m -S RUN_ONCE bash -c \
      "( ${DEP_ENV}; $BIN_DEREF $FSW_ARGS | $TEE_BIN ${STDOUT_LOG}$BOOT_COUNTER_RTN ) 3>&1 1>&2 2>&3 | $TEE_BIN ${STDERR_LOG}$BOOT_COUNTER_RTN"
  # NOTE(mereweth) - if we get here, then we crashed - FPGA powers us off
  evr 1 "RUN_ONCE failed $BIN_DEREF"
fi
# Trying to make sure there is no way the RUN_ONCE symlink persists, even if there is a bug above
rm $RUN_ONCE_SYMLINK 2> /dev/null || evr 5 "$RUN_ONCE_SYMLINK already removed"
evr 3 "after trying RUN_ONCE"

############################## Try to run CURRENT ##############################
evr 3 "checking CURRENT $DEPLOYMENT"
get_valid_bin BIN_NAME_DEST $CURRENT_SYMLINK $DEPLOYMENT
if (( $? == 0 )); then
  # follow the symlink for debugging
  BIN_DEREF=$($READLINK $BIN_NAME_DEST)
  evr 4 "dereferenced $BIN_NAME_DEST"
  evr 4 "    to $BIN_DEREF"
  evr 3 "running CURRENT $DEPLOYMENT"
  $SCREEN_BIN -D -m -S CURRENT bash -c \
      "( ${DEP_ENV}; $BIN_DEREF $FSW_ARGS | $TEE_BIN ${STDOUT_LOG}$BOOT_COUNTER_RTN ) 3>&1 1>&2 2>&3 | $TEE_BIN ${STDERR_LOG}$BOOT_COUNTER_RTN"
  # NOTE(mereweth) - if we get here, then we crashed - FPGA powers us off
  evr 1 "CURRENT failed $BIN_DEREF"
fi
evr 3 "after trying CURRENT"

############################## Try to run GOLDEN ##############################
evr 3 "checking GOLDEN $DEPLOYMENT"
get_valid_bin BIN_NAME_DEST $GOLDEN_DIR $DEPLOYMENT
if (( $? == 0 )); then
  evr 3 "running GOLDEN $DEPLOYMENT"
  $SCREEN_BIN -D -m -S GOLDEN bash -c \
      "( ${DEP_ENV}; $BIN_NAME_DEST $FSW_ARGS | $TEE_BIN ${STDOUT_LOG}$BOOT_COUNTER_RTN ) 3>&1 1>&2 2>&3 | $TEE_BIN ${STDERR_LOG}$BOOT_COUNTER_RTN"
  # NOTE(mereweth) - if we get here, then we crashed - FPGA powers us off
  evr 1 "GOLDEN failed $BIN_NAME_DEST"
fi
evr 1 "after trying GOLDEN"

# Hail mary - attempt to run any of the golden FSW versions; no FSW args
fail_signify_action &
while true; do
  for i in {0..2}; do
    ( ${GOLDEN_DIR}/${DEPLOYMENT}${i} | $TEE_BIN ${STDOUT_LOG}_${BOOT_COUNTER_RTN}_${i} ) 3>&1 1>&2 2>&3 | $TEE_BIN ${STDERR_LOG}_${BOOT_COUNTER_RTN}_${i}
    evr 1 "GOLDEN${i} failed ${GOLDEN_DIR}/${DEPLOYMENT}${i}"
  done
  if [[ "$__DEBUG" == "true" ]]; then exit 1; fi
done

# Ryan kills us
