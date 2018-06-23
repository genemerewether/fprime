#!/bin/bash

# NOTE(mereweth) - WE DO NOT SUPPORT SPACE-SEPARATED DIRECTORIES OR NAMES

# critical settings
INHIBIT_COOKIE_TIMEOUT=0
# boot counter is filled in by expand_fsw_args in startup.bash
FSW_ARGS="-p 50000 -a 192.168.2.1 -b"

# binaries
CKSUM_BIN=/usr/bin/cksum
RM_BIN=/bin/rm
CUT_BIN=/usr/bin/cut
ECHO_BIN=/bin/echo
TEE_BIN=/usr/bin/tee
SCREEN_BIN=/usr/bin/screen
SLEEP_BIN=/bin/sleep
READLINK="/bin/readlink -f"
DATE_BIN="/bin/date +%s.%3N"

# locations of files
VERBOSITY=/fsw/verbosity.bash
CURRENT_SYMLINK=/fsw/CURRENT
RUN_ONCE_SYMLINK=/seq/RUN_ONCE

STARTUP_LOG=/eng/STARTUP_LOG
RUN_ONCE_LOG=/eng/RUN_ONCE_LOG
CURRENT_LOG=/eng/CURRENT_LOG
GOLDEN_LOG=/eng/GOLDEN_LOG

CORE_DUMP_DIR=/img
CORE_DUMP_PATTERN_BASE=core

FSW_DIR=/fsw
STARTUP_DIR=/startup
GOLDEN_DIR=/golden

INHIBIT_COOKIE=/tmp/MAGIC_COOKIE

BOOT_COUNTER_FILE=/seq/BOOT_COUNTER

startup_signify_action() {
    ${ECHO_BIN} 1 > /sys/class/leds/green/blink || true
    ${ECHO_BIN} 1 > /sys/class/leds/blue/blink || true
    ${ECHO_BIN} 1 > /sys/class/leds/red/blink || true
}

up_signify_action() {
    ${ECHO_BIN} 1 > /sys/class/leds/green/blink || true
    ${ECHO_BIN} 0 > /sys/class/leds/blue/blink || true
    ${ECHO_BIN} 0 > /sys/class/leds/red/blink || true

    # configure core dump location and contents
    # TODO(mereweth) - how do we lose the heap but keep globals?
    ${ECHO_BIN} 0 > /proc/$$/coredump_filter || true
    ${ECHO_BIN} "/video/${CORE_DUMP_PATTERN_BASE}" > /proc/sys/kernel/core_pattern || true
}

fail_signify_action() {
    ${ECHO_BIN} 0 > /sys/class/leds/green/blink || true
    ${ECHO_BIN} 0 > /sys/class/leds/blue/blink || true
    ${ECHO_BIN} 1 > /sys/class/leds/red/blink || true
}
