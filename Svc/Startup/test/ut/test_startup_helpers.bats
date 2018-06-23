#!/usr/bin/env bats

. ./settings.bash

. ./startup_helpers.bash

############################# test startup helpers #############################

@test "expand fsw args no arguments" {
  ! expand_fsw_args
}

@test "expand fsw args no BOOT_COUNTER_RTN" {
  expand_fsw_args dest_var FSW_ARGS
  [[ "$dest_var" == "${FSW_ARGS} 0" ]]
}

@test "expand fsw args with BOOT_COUNTER_RTN" {
  BOOT_COUNTER_RTN=1
  expand_fsw_args dest_var FSW_ARGS
  [[ "$dest_var" == "${FSW_ARGS} 1" ]]
}
