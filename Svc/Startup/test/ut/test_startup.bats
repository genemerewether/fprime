#!/usr/bin/env bats

. ./deployment.bash
. ./settings.bash

. ./startup_helpers.bash

# NOTE(mereweth) - not in settings.bash so we can run tests in test_logging.bats
VERBOSITY=${LOCAL_DIR}/seq/verbosity.bash

######################## startup test helpers ########################
setup_startup() {
  teardown_startup || true

  mkdir -p ${FSW_DIR}/myver
  ln -s ${FSW_DIR}/myver $CURRENT_SYMLINK
  cp ${GOLDEN_DIR}/${DEPLOYMENT}{0..2} ${FSW_DIR}/myver
  cp ${GOLDEN_DIR}/${DEPLOYMENT}.crc{0..2} ${FSW_DIR}/myver

  mkdir -p ${FSW_DIR}/myrun_once
  ln -s ${FSW_DIR}/myrun_once $RUN_ONCE_SYMLINK
  cp ${GOLDEN_DIR}/${DEPLOYMENT}{0..2} ${FSW_DIR}/myrun_once
  cp ${GOLDEN_DIR}/${DEPLOYMENT}.crc{0..2} ${FSW_DIR}/myrun_once

  mkdir -p ${FSW_DIR}/mycorrupt_dir
  cp ${GOLDEN_DIR}/${DEPLOYMENT}{0..2} ${FSW_DIR}/mycorrupt_dir
  cp ${GOLDEN_DIR}/${DEPLOYMENT}.crc{0..2} ${FSW_DIR}/mycorrupt_dir

  expand_fsw_args TEST_FSW_ARGS FSW_ARGS
}

startup_equivalent_time_sleep() {
  sleep 10
}

startup_check_runonce_log() {
  [ -f "${STDOUT_LOG}0" ]
  [[ "$(head -1 ${STDOUT_LOG}0)" = "I AM ${FSW_DIR}/myrun_once/${DEPLOYMENT}0, ARGS ${TEST_FSW_ARGS}" ]]
  # shift down one line in unified log
  tail -n +2 ${STDOUT_LOG}0 > ${STDOUT_LOG}0

  [ -f "${STDERR_LOG}0" ]
  [[ "$(head -1 ${STDERR_LOG}0)" = "STDERR: I AM ${FSW_DIR}/myrun_once/${DEPLOYMENT}0, ARGS ${TEST_FSW_ARGS}" ]]
  # shift down one line in unified log
  tail -n +2 ${STDERR_LOG}0 > ${STDERR_LOG}0
}

startup_check_current_log() {
  [ -f "${STDOUT_LOG}0" ]
  [[ "$(head -1 ${STDOUT_LOG}0)" = "I AM ${CURRENT_SYMLINK}/${DEPLOYMENT}0, ARGS ${TEST_FSW_ARGS}" ]]
  # shift down one line in unified log
  tail -n +2 ${STDOUT_LOG}0 > ${STDOUT_LOG}0

  [ -f "${STDERR_LOG}0" ]
  [[ "$(head -1 ${STDERR_LOG}0)" = "STDERR: I AM ${CURRENT_SYMLINK}/${DEPLOYMENT}0, ARGS ${TEST_FSW_ARGS}" ]]
  # shift down one line in unified log
  tail -n +2 ${STDERR_LOG}0 > ${STDERR_LOG}0
}

startup_check_golden_log() {
  [ -f "${STDOUT_LOG}0" ]
  [[ "$(head -1 ${STDOUT_LOG}0)" = "I AM ${GOLDEN_DIR}/${DEPLOYMENT}0, ARGS ${TEST_FSW_ARGS}" ]]
  # shift down one line in unified log
  tail -n +2 ${STDOUT_LOG}0 > ${STDOUT_LOG}0

  [ -f "${STDERR_LOG}0" ]
  [[ "$(head -1 ${STDERR_LOG}0)" = "STDERR: I AM ${GOLDEN_DIR}/${DEPLOYMENT}0, ARGS ${TEST_FSW_ARGS}" ]]
  # shift down one line in unified log
  tail -n +2 ${STDERR_LOG}0 > ${STDERR_LOG}0
}

startup_check_golden_fallthrough_log() {
  [ -f "${STDOUT_LOG}_0_0" ]
  [[ "$(head -1 ${STDOUT_LOG}_0_0)" = "I AM ${GOLDEN_DIR}/${DEPLOYMENT}0, ARGS " ]]
  [ -f "${STDOUT_LOG}_0_1" ]
  [[ "$(head -1 ${STDOUT_LOG}_0_1)" = "I AM ${GOLDEN_DIR}/${DEPLOYMENT}1, ARGS " ]]
  [ -f "${STDOUT_LOG}_0_2" ]
  [[ "$(head -1 ${STDOUT_LOG}_0_2)" = "I AM ${GOLDEN_DIR}/${DEPLOYMENT}2, ARGS " ]]

  [ -f "${STDERR_LOG}_0_0" ]
  [[ "$(head -1 ${STDERR_LOG}_0_0)" = "STDERR: I AM ${GOLDEN_DIR}/${DEPLOYMENT}0, ARGS " ]]
  [ -f "${STDERR_LOG}_0_1" ]
  [[ "$(head -1 ${STDERR_LOG}_0_1)" = "STDERR: I AM ${GOLDEN_DIR}/${DEPLOYMENT}1, ARGS " ]]
  [ -f "${STDERR_LOG}_0_2" ]
  [[ "$(head -1 ${STDERR_LOG}_0_2)" = "STDERR: I AM ${GOLDEN_DIR}/${DEPLOYMENT}2, ARGS " ]]
}

teardown_startup() {
  rm $BOOT_COUNTER_FILE || true
  rm $STARTUP_LOG || true

  rm ${STDOUT_LOG}[0-9]* || true
  rm ${STDOUT_LOG}_[0-9]_{0..2}* || true
  rm ${STDERR_LOG}[0-9]* || true
  rm ${STDERR_LOG}_[0-9]_{0..2}* || true

  # NOTE(mereweth) - these are not used
  rm ${RUN_ONCE_LOG}[0-9]* || true
  rm ${CURRENT_LOG}[0-9]* || true
  rm ${GOLDEN_LOG}[0-9]* || true
  rm ${GOLDEN_LOG}_[0-9]_{0..2}* || true

  rm $RUN_ONCE_SYMLINK 2> /dev/null || true
  rm $CURRENT_SYMLINK 2> /dev/null || true
  rm -r ${FSW_DIR}/myver 2> /dev/null || true
  rm -r ${FSW_DIR}/myrun_once 2> /dev/null || true
  rm -r ${FSW_DIR}/mycorrupt_dir 2> /dev/null || true
}

############################### run_once symlink ###############################
@test "run-once symlink present; no current symlink" {
  setup_startup
  rm $CURRENT_SYMLINK 2> /dev/null || true

  ./startup.bash &
  startup_pid=$!
  $SLEEP_BIN $INHIBIT_COOKIE_TIMEOUT
  # extra few seconds to get through all test binaries (all of which exit)
  startup_equivalent_time_sleep

  ! [ -f "$RUN_ONCE_SYMLINK" ]
  startup_check_runonce_log

  startup_check_golden_log

  startup_check_golden_fallthrough_log

  wait $startup_pid || true # because our test binaries exit, this returns an error, so catch it

  #teardown_startup
}

@test "run-once symlink present; current symlink present" {
  setup_startup

  ./startup.bash &
  startup_pid=$!
  $SLEEP_BIN $INHIBIT_COOKIE_TIMEOUT
  # extra few seconds to get through all test binaries (all of which exit)
  startup_equivalent_time_sleep

  ! [ -f "$RUN_ONCE_SYMLINK" ]
  startup_check_runonce_log

  startup_check_current_log

  startup_check_golden_log

  startup_check_golden_fallthrough_log

  teardown_startup
}

@test "no run-once symlink; current symlink present" {
  setup_startup
  rm $RUN_ONCE_SYMLINK 2> /dev/null || true

  ./startup.bash &
  startup_pid=$!
  $SLEEP_BIN $INHIBIT_COOKIE_TIMEOUT
  # extra few seconds to get through all test binaries (all of which exit)
  startup_equivalent_time_sleep

  ! [ -f "$RUN_ONCE_SYMLINK" ]

  startup_check_current_log

  startup_check_golden_log

  startup_check_golden_fallthrough_log

  teardown_startup
}

@test "no run-once symlink; no current symlink" {
  setup_startup
  rm $RUN_ONCE_SYMLINK 2> /dev/null || true
  rm $CURRENT_SYMLINK 2> /dev/null || true

  ./startup.bash &
  startup_pid=$!
  $SLEEP_BIN $INHIBIT_COOKIE_TIMEOUT
  # extra few seconds to get through all test binaries (all of which exit)
  startup_equivalent_time_sleep

  ! [ -f "$RUN_ONCE_SYMLINK" ]

  startup_check_golden_log

  startup_check_golden_fallthrough_log

  teardown_startup
}

################################# magic cookie #################################

################################# boot counter #################################
