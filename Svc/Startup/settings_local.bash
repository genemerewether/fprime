#!/bin/bash

################## WARNING - settings for local testing only ##################

# boot counter is filled in by expand_fsw_args in startup.bash
FSW_ARGS="-p 50000 -a localhost -l -b"

LOCAL_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# leave the Hail Mary FSW run loop early
__DEBUG=true

# critical settings
MAGIC_COOKIE_TIMEOUT=5

# binaries
CKSUM_BIN=/usr/bin/cksum
RM_BIN=/bin/rm
CUT_BIN=/usr/bin/cut
ECHO_BIN=/bin/echo
TEE_BIN=/usr/bin/tee
SCREEN_BIN=/usr/bin/screen
DATE_BIN="/bin/date +%s"

# locations of files
CURRENT_SYMLINK=${LOCAL_DIR}/fsw/CURRENT
RUN_ONCE_SYMLINK=${LOCAL_DIR}/seq/RUN_ONCE

STARTUP_LOG=${LOCAL_DIR}/eng/STARTUP_LOG
RUN_ONCE_LOG=${LOCAL_DIR}/eng/RUN_ONCE_LOG
CURRENT_LOG=${LOCAL_DIR}/eng/CURRENT_LOG
GOLDEN_LOG=${LOCAL_DIR}/eng/GOLDEN_LOG

CORE_DUMP_DIR=${LOCAL_DIR}/img
CORE_DUMP_PATTERN_BASE=core

FSW_DIR=${LOCAL_DIR}/fsw
STARTUP_DIR=${LOCAL_DIR}
GOLDEN_DIR=${LOCAL_DIR}/golden

INHIBIT_COOKIE=${LOCAL_DIR}/tmp/MAGIC_COOKIE

BOOT_COUNTER_FILE=${LOCAL_DIR}/seq/BOOT_COUNTER

readlink_semiportable() {
    local d="$(dirname ${1})"
    local f="$(basename ${1})"
    if ! [ -f ${1} ]; then return 1; fi
    (
        cd ${d} >/dev/null 2>&1
        while [ -h "${f}" ] ; do
            cd $(dirname $(readlink ${f})) >/dev/null 2>&1
        done
        ${ECHO_BIN} $(pwd -P)/${f}
    )
}

READLINK=readlink_semiportable

startup_signify_action() {
    rm ${LOCAL_DIR}/STARTUP_BIN_UP || true
    rm ${LOCAL_DIR}/STARTUP_FAIL || true
    rm ${LOCAL_DIR}/STARTUP_UP || true

    touch ${LOCAL_DIR}/STARTUP_UP
}

up_signify_action() {
    rm ${LOCAL_DIR}/STARTUP_BIN_UP || true
    rm ${LOCAL_DIR}/STARTUP_FAIL || true
    rm ${LOCAL_DIR}/STARTUP_UP || true

    touch ${LOCAL_DIR}/STARTUP_BIN_UP
}

fail_signify_action() {
    rm ${LOCAL_DIR}/STARTUP_BIN_UP || true
    rm ${LOCAL_DIR}/STARTUP_FAIL || true
    rm ${LOCAL_DIR}/STARTUP_UP || true

    touch ${LOCAL_DIR}/STARTUP_FAIL
}
