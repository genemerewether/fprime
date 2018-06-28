#!/bin/bash

. ./settings.bash

# only source the verbosity control script if it exists
[[ -f "$VERBOSITY" ]] && . $VERBOSITY 2> /dev/null
# set to always print and log unless overridden in verbosity.bash
: ${VERBOSE_LOG:=10}
: ${VERBOSE_PRINT:=10}

function backtrace() {
  local frame=0
  while caller $frame; do
    ((frame++));
  done
  $ECHO_BIN "$*" >&2
  $ECHO_BIN "$*" >> $STARTUP_LOG
}

declare -a EVR_LEVELS
# NavPatch does not turn FATAL EVRs in STARTUP_LOG into FSW fatal EVRs
EVR_LEVELS=([0]="FATAL" [1]="WARNING_HI" [2]="WARNING_LO" [3]="ACTIVITY_HI" [4]="ACTIVITY_LO" [5]="DIAGNOSTIC" [6]="DEBUG")
function evr () {
  # Use this function like so: evr 1 INFORMATIVE MESSAGE HERE
  CALLER_INFO=$(caller 0)
  local LEVEL=${1}

  re='^[0-9]+$'
  # if first argument, which should be logging level, is not numeric
  if ! [[ "$LEVEL" =~ $re ]]; then
    local __MSG="$($DATE_BIN) $LINENO $FUNCNAME $BASH_SOURCE LEVEL $LEVEL not numeric"
    # can't use the evr function - we are inside it!
    $ECHO_BIN "${EVR_LEVELS[$LEVEL]} $__MSG" >&2
    $ECHO_BIN "1 $__MSG" >> $STARTUP_LOG
    LEVEL=1
  else
    # Remove the first argument from the full argument list
    shift
  fi

  re='^[0-9]+$'
  if (! [[ "$VERBOSE_LOG" =~ $re ]]) || (! [[ "$VERBOSE_PRINT" =~ $re ]]); then
    local __MSG="$($DATE_BIN) $LINENO $FUNCNAME $BASH_SOURCE VERBOSE_LOG $VERBOSE_LOG or VERBOSE_PRINT $VERBOSE_PRINT not numeric"
    # can't use the evr function - we are inside it!
    $ECHO_BIN "${EVR_LEVELS[$LEVEL]} $__MSG" >&2
    $ECHO_BIN "1 $__MSG" >> $STARTUP_LOG
    VERBOSE_LOG=10
    VERBOSE_PRINT=10
  fi

  # FATAL - generate backtrace
  if (( $LEVEL == 0 )); then
    backtrace
  fi

  if [ ${VERBOSE_PRINT} -ge ${LEVEL} ]; then
    # use the array above to look up the appropriate print level
    $ECHO_BIN "${EVR_LEVELS[$LEVEL]}" "$($DATE_BIN)" "$CALLER_INFO" "$@" >&2
  fi

  if [ ${VERBOSE_LOG} -ge ${LEVEL} ]; then
    # NavPatch will look up the appropriate FSW EVR level
    $ECHO_BIN "$LEVEL" "$($DATE_BIN)" "$CALLER_INFO" "$@" >> $STARTUP_LOG
  fi
}
