#!/bin/bash

. ./settings.bash
. ${STARTUP_DIR}/logging.bash

crc_vote() {
  if ! (( $# == 4 )); then
    evr 0 "usage: crc_vote dest_var crc1 crc2 crc3"
    return 1
  fi
  evr 6 "dest_var is $1" # debug the destination variable name

  # majority voting on correct CRC value
  if (( $2 == $3 )); then
    # This is the nominal condition, so is only level 4, ACTIVITY_LO
    evr 4 "1st and 2nd equal $2"
    eval ${1}=$2
    # this check is so we can tell if the 3rd binary has gone bad
    if (( $2 == $4 )); then
      # This is the nominal condition, so is only level 4, ACTIVITY_LO
      evr 4 "1st and 3rd equal $2"
    else
      evr 1 "1st and 3rd not equal"
    fi
    return 0
  elif (( $2 == $4 )); then
    # This is off-nominal, so it is level 1, WARNING_HI
    evr 1 "1st and 2nd not equal; 1st and 3rd equal $2"
    eval ${1}=$2
    return 0
  elif (( $3 == $4 )); then
    evr 1 "1st and 2nd / 1st and 3rd not equal; 2nd and 3rd equal $3"
    eval ${1}=$3
    return 0
  else
    evr 1 "none equal; $2 $3 $4"
    eval ${1}=-1
    return 1
  fi
}

get_crc32_from_cksum() {
  if (( $# < 2 )); then
    evr 0 "usage: get_crc32_from_cksum dest_var cksum_output ..."
    return 1
  fi
  evr 6 "dest_var is $1" # debug the destination variable name

  local temp_array=($2)

  evr 5 "${temp_array[0]}"

  eval ${1}=${temp_array[0]}

  return 0
}

get_valid_bin() {
  if ! (( $# == 3 )); then
    evr 0 "usage: get_valid_bin dest_var version_folder bin_name"
    return 1
  fi
  evr 6 "dest_var is $1" # debug the destination variable name

  evr 5 "version_folder is $2"

  # Test if version_folder is a symlink to a folder, or is a folder
  if ! [ -d "$2" ]; then
    if [ -e "$2" ]; then
      evr 1 "$2 exists"
    fi
    if [ -L "$2" ]; then
      evr 1 "$2 dangling symlink"
    fi
    evr 2 "$3 is missing";
    return 1
  fi

  # Load all three checksums
  local CHK=(-1 -2 -3)
  for i in {0..2}; do
    local BIN_FULL=${2}/${3}.crc${i}
    if [ -f "$BIN_FULL" ]; then
      CHK[i]=$(cat "$BIN_FULL")
    else
      CHK[i]=-${i}
    fi
  done

  crc_vote consensus ${CHK[*]} || return 1

  # If a binary that matches the crc exists, return it
  for i in {0..2}; do
    local BIN_FULL=${2}/${3}${i}
    if [ -f "$BIN_FULL" ]; then
      local TMP_CMP
      get_crc32_from_cksum TMP_CMP $($CKSUM_BIN $BIN_FULL)
      if (( ${CHK[i]} == $TMP_CMP )); then
        eval ${1}=$BIN_FULL
        evr 4 "${i}th try; $BIN_FULL is valid";
        return 0 # we are done - found a valid copy
      fi
    fi
  done

  evr 1 "no binary found matching $consensus";
  # Fall through if we didn't find a binary that matches the crc
  return 1
}
