#!/bin/bash

SVC_STARTUP_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/../"

pushd . > /dev/null
cd ${SVC_STARTUP_DIR}
. ./settings.bash

VERBOSE_LOG=0
VERBOSE_PRINT=5
. ./logging.bash
popd > /dev/null

DEP_WAS_SET=0
VER_WAS_SET=0

# first colon is silent parsing; colon after option name is for argument
while getopts ":d:v:" opt; do
  case $opt in
    d)
      if [[ $DEP_WAS_SET = 0 ]]; then
        DEP_WAS_SET=1
        DEPLOYMENT=$OPTARG
      else
        evr 1 "Option -d already used."
        exit 1
      fi
      ;;
    v)
      if [[ $VER_WAS_SET = 0 ]]; then
        VER_WAS_SET=1
        VERNAME=$OPTARG
      else
        evr 1 "Option -v already used."
        exit 1
      fi
      ;;
    \?)
      evr 1 "Invalid option: -$OPTARG"
      exit 1
      ;;
    :)
      evr 1 "Option -$OPTARG requires an argument."
      exit 1
      ;;
  esac
done

if [ -z "$DEPLOYMENT" ]; then
  evr 1 "DEPLOYMENT not set"
  exit 1
fi

if [ -z "$VERNAME" ]; then
  evr 6 "FSW version folder unset; using date/time stamped folder"
  echo ${FSW_DIR}/${DEPLOYMENT}_$(date +%F_%H-%M-%S)
else
  echo ${FSW_DIR}/${VERNAME}
fi
