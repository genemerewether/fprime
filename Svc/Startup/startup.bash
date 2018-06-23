#!/bin/bash

. ./settings.bash
. ${STARTUP_DIR}/deployment.bash
. ${STARTUP_DIR}/boot_counter.bash
. ${STARTUP_DIR}/hash_validation.bash
. ${STARTUP_DIR}/startup_helpers.bash
. ${STARTUP_DIR}/logging.bash

#TODO(mereweth) - set -e? if so, then make sure each command returns true

startup_signify_action &

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
get_valid_bin BIN_NAME_DEST $RUN_ONCE_SYMLINK $DEPLOYMENT
if (( $? == 0 )); then
  # have to follow the symlink and store that path so we can delete the symlink pre-run
  BIN_DEREF=$($READLINK $BIN_NAME_DEST)
  evr 4 "dereferenced $BIN_NAME_DEST to $BIN_DEREF"
  rm $RUN_ONCE_SYMLINK 2> /dev/null || evr 1 "failed to remove $RUN_ONCE_SYMLINK"
  evr 3 "running RUN_ONCE"
  $SCREEN_BIN -D -m -S RUN_ONCE bash -c "$BIN_DEREF $FSW_ARGS 2>&1 | $TEE_BIN ${RUN_ONCE_LOG}$BOOT_COUNTER_RTN"
  # NOTE(mereweth) - if we get here, then we crashed - FPGA powers us off
  evr 1 "RUN_ONCE failed $BIN_DEREF"
fi
# Trying to make sure there is no way the RUN_ONCE symlink persists, even if there is a bug above
rm $RUN_ONCE_SYMLINK 2> /dev/null || evr 4 "$RUN_ONCE_SYMLINK not present or already removed"

############################## Try to run CURRENT ##############################
get_valid_bin BIN_NAME_DEST $CURRENT_SYMLINK $DEPLOYMENT
if (( $? == 0 )); then
  # follow the symlink for debugging
  BIN_DEREF=$($READLINK $BIN_NAME_DEST)
  evr 4 "dereferenced $BIN_NAME_DEST to $BIN_DEREF"
  evr 3 "running CURRENT"
  $SCREEN_BIN -D -m -S CURRENT bash -c "$BIN_NAME_DEST $FSW_ARGS 2>&1 | $TEE_BIN ${CURRENT_LOG}$BOOT_COUNTER_RTN"
  # NOTE(mereweth) - if we get here, then we crashed - FPGA powers us off
  evr 1 "CURRENT failed $BIN_DEREF"
fi
evr 3 "after trying $CURRENT_SYMLINK"

############################## Try to run GOLDEN ##############################
get_valid_bin BIN_NAME_DEST $GOLDEN_DIR $DEPLOYMENT
if (( $? == 0 )); then
  evr 3 "running GOLDEN"
  $SCREEN_BIN -D -m -S GOLDEN bash -c "$BIN_NAME_DEST $FSW_ARGS 2>&1 | $TEE_BIN ${GOLDEN_LOG}$BOOT_COUNTER_RTN"
  # NOTE(mereweth) - if we get here, then we crashed - FPGA powers us off
  evr 1 "GOLDEN failed $BIN_NAME_DEST"
fi
evr 1 "after trying validated $GOLDEN_DIR"

# Hail mary - attempt to run any of the golden FSW versions; no FSW args
fail_signify_action &
while true; do
  for i in {0..2}; do
    ${GOLDEN_DIR}/${DEPLOYMENT}${i} 2>&1 | $TEE_BIN -a ${GOLDEN_LOG}_${i}_$BOOT_COUNTER_RTN
    evr 1 "GOLDEN${i} failed ${GOLDEN_DIR}/${DEPLOYMENT}${i}"
  done
  if [[ "$__DEBUG" == "true" ]]; then exit 1; fi
done

# Ryan kills us
