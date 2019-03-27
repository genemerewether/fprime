#!/usr/bin/env bats

. ./settings.bash

. ./logging.bash

############## test missing or malforformed VERBOSITY file / var ##############

@test "missing VERBOSITY var" {
  OLD_VERBOSITY=$VERBOSITY
  unset VERBOSITY
  unset VERBOSE_LOG
  unset VERBOSE_PRINT

  . ./logging.bash
  [ $VERBOSE_LOG -eq 10 ]
  [ $VERBOSE_PRINT -eq 10 ]
  evr 1 "test EVR"
  VERBOSITY=$OLD_VERBOSITY
}

@test "missing VERBOSITY file" {
  OLD_VERBOSITY=$VERBOSITY
  VERBOSITY="DOES_NOT_EXIST"
  unset VERBOSE_LOG
  unset VERBOSE_PRINT

  . ./logging.bash
  [ $VERBOSE_LOG -eq 10 ]
  [ $VERBOSE_PRINT -eq 10 ]
  evr 1 "test EVR"
  VERBOSITY=$OLD_VERBOSITY
}

@test "VERBOSITY file present with 4 for both levels" {
  OLD_VERBOSITY=$VERBOSITY
  VERBOSITY="seq/verbosity.bash"
  rm $VERBOSITY || true
  echo "VERBOSE_LOG=4" >> $VERBOSITY
  echo "VERBOSE_PRINT=4" >> $VERBOSITY
  . ./logging.bash
  [ $VERBOSE_LOG -eq 4 ]
  [ $VERBOSE_PRINT -eq 4 ]
  evr 5 "test EVR"
  VERBOSITY=$OLD_VERBOSITY
}

################################### test evr ###################################
