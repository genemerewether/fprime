#!/usr/bin/env bats

. ./boot_counter.bash

# NOTE(mereweth) - not in settings.bash so we can run tests in test_logging.bats
VERBOSITY=${LOCAL_DIR}/seq/verbosity.bash

################################### increment ###################################
@test "missing boot counter file" {
  rm $BOOT_COUNTER_FILE 2> /dev/null || true
  ! increment_boot_counter dest_var
  (( $dest_var == 0 ))
  rm $BOOT_COUNTER_FILE 2> /dev/null || true
}

@test "boot counter file present but empty" {
  rm $BOOT_COUNTER_FILE 2> /dev/null || true
  touch $BOOT_COUNTER_FILE
  ! increment_boot_counter dest_var
  (( $dest_var == 0 ))
  rm $BOOT_COUNTER_FILE 2> /dev/null || true
}

@test "boot counter file present but non-numeric" {
  rm $BOOT_COUNTER_FILE 2> /dev/null || true
  echo "hi" > $BOOT_COUNTER_FILE
  ! increment_boot_counter dest_var
  (( $dest_var == 0 ))
  rm $BOOT_COUNTER_FILE 2> /dev/null || true
}

@test "boot counter file present and numeric" {
  rm $BOOT_COUNTER_FILE 2> /dev/null || true
  echo "0" > $BOOT_COUNTER_FILE
  increment_boot_counter dest_var
  (( $dest_var == 1 ))
  (( $(cat $BOOT_COUNTER_FILE) == 1 ))
  rm $BOOT_COUNTER_FILE 2> /dev/null || true
}
