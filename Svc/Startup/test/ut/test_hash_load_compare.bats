#!/usr/bin/env bats

. ./deployment.bash
. ./settings.bash

. ./hash_validation.bash

DEBUG=true

############################ get_valid_bin helpers ############################
setup_get_valid_bin() {
  teardown_get_valid_bin || true

  mkdir -p ${FSW_DIR}/myver
  ln -s ${FSW_DIR}/myver $RUN_ONCE_SYMLINK
  cp ${GOLDEN_DIR}/${DEPLOYMENT}{0..2} ${FSW_DIR}/myver
  cp ${GOLDEN_DIR}/${DEPLOYMENT}.crc{0..2} ${FSW_DIR}/myver
}

teardown_get_valid_bin() {
  rm $RUN_ONCE_SYMLINK 2> /dev/null || true
  rm -r ${FSW_DIR}/myver 2> /dev/null || true
}

############################ get_valid_bin failure ############################
@test "symlink missing" {
  setup_get_valid_bin
  rm $RUN_ONCE_SYMLINK 2> /dev/null || true
  ! get_valid_bin BIN_NAME_DEST $RUN_ONCE_SYMLINK DOES_NOT_EXIST
  teardown_get_valid_bin
}

@test "symlink present; destination of symlink (fsw version folder) missing" {
  setup_get_valid_bin
  rm -r ${FSW_DIR}/myver 2> /dev/null || true
  ! get_valid_bin BIN_NAME_DEST $RUN_ONCE_SYMLINK DOES_NOT_EXIST
  teardown_get_valid_bin
}

@test "fsw folder present but missing 2 out of 3 checksums" {
  setup_get_valid_bin
  rm ${FSW_DIR}/myver/${DEPLOYMENT}.crc{1,2}
  ! get_valid_bin BIN_NAME_DEST $RUN_ONCE_SYMLINK DOES_NOT_EXIST
  teardown_get_valid_bin
}

@test "fsw folder three checksums present and matching; binaries missing" {
  setup_get_valid_bin
  rm ${FSW_DIR}/myver/${DEPLOYMENT}{0..2}
  ! get_valid_bin BIN_NAME_DEST $RUN_ONCE_SYMLINK DOES_NOT_EXIST
  teardown_get_valid_bin
}

@test "fsw folder no binaries match" {
  setup_get_valid_bin
  rm ${FSW_DIR}/myver/${DEPLOYMENT}.crc1
  rm ${FSW_DIR}/myver/${DEPLOYMENT}{0..2}
  touch ${FSW_DIR}/myver/${DEPLOYMENT}2
  ! get_valid_bin BIN_NAME_DEST $RUN_ONCE_SYMLINK ${DEPLOYMENT}
  teardown_get_valid_bin
}

############################ get_valid_bin success ############################
@test "fsw folder two checksums present and matching" {
  setup_get_valid_bin
  rm ${FSW_DIR}/myver/${DEPLOYMENT}.crc1
  get_valid_bin BIN_NAME_DEST $RUN_ONCE_SYMLINK ${DEPLOYMENT}
  [[ "$BIN_NAME_DEST" = "${DEPLOYMENT}0" ]]
  teardown_get_valid_bin
}

@test "fsw folder nominal" {
  setup_get_valid_bin
  get_valid_bin BIN_NAME_DEST $RUN_ONCE_SYMLINK ${DEPLOYMENT}
  [[ "$BIN_NAME_DEST" = "${DEPLOYMENT}0" ]]
  teardown_get_valid_bin
}

@test "fsw folder two checksums present and matching; only one binary present" {
  setup_get_valid_bin
  rm ${FSW_DIR}/myver/${DEPLOYMENT}.crc1
  rm ${FSW_DIR}/myver/${DEPLOYMENT}{0,1}
  get_valid_bin BIN_NAME_DEST $RUN_ONCE_SYMLINK ${DEPLOYMENT}
  [[ "$BIN_NAME_DEST" = "${DEPLOYMENT}2" ]]
  teardown_get_valid_bin
}

@test "version_folder is NOT a symlink - e.g. golden dir" {
  setup_get_valid_bin
  get_valid_bin BIN_NAME_DEST $GOLDEN_DIR ${DEPLOYMENT}
  [[ "$BIN_NAME_DEST" = "${DEPLOYMENT}0" ]]
  teardown_get_valid_bin
}
