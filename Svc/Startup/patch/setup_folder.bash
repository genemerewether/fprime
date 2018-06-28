#!/bin/bash

SVC_STARTUP_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/../"

pushd . > /dev/null
cd ${SVC_STARTUP_DIR}
. ./settings.bash

. ./hash_validation.bash 2> /dev/null

VERBOSE_LOG=0
VERBOSE_PRINT=5
. ./logging.bash
popd > /dev/null

DEP_WAS_SET=0
FULLVER_WAS_SET=0
TYPE_WAS_SET=0
INFOLD_WAS_SET=0
RUNSCRIPT_WAS_SET=0

# first colon is silent parsing; colon after option name is for argument
while getopts ":d:f:t:i:r:n" opt; do
  case $opt in
    n)
      NOPUSH=1
      ;;
    d)
      if [[ $DEP_WAS_SET = 0 ]]; then
        DEP_WAS_SET=1
        DEPLOYMENT=$OPTARG
      else
        evr 1 "Option -d already used."
        exit 1
      fi
      ;;
    r)
      if [[ $RUNSCRIPT_WAS_SET = 0 ]]; then
        RUNSCRIPT_WAS_SET=1
        RUNSCRIPT=$OPTARG
      else
        evr 1 "Option -r already used."
        exit 1
      fi
      ;;
    i)
      if [[ $INFOLD_WAS_SET = 0 ]]; then
        INFOLD_WAS_SET=1
        INFOLD=$OPTARG
      else
        evr 1 "Option -i already used."
        exit 1
      fi
      ;;
    f)
      if [[ $FULLVER_WAS_SET = 0 ]]; then
        FULLVER_WAS_SET=1
        FULLVER=$OPTARG
      else
        evr 1 "Option -f already used."
        exit 1
      fi
      ;;
    t)
      if [[ $TYPE_WAS_SET = 0 ]]; then
        TYPE_WAS_SET=1
        TYPE=$OPTARG
      else
        evr 1 "Option -t already used."
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

if [ -z "$TYPE" ]; then
  evr 1 "FSW operation type not set"
  exit 1
fi

if [ "$TYPE" == "gold" ]; then
  FULLVER=$GOLDEN_DIR
  TYPE="load"
fi

if [ -z "$FULLVER" ]; then
  evr 1 "FSW version folder unset"
  exit 1
fi

if [ $RUNSCRIPT_WAS_SET -ne 0 ]; then
  SRCFILE=$(echo "${RUNSCRIPT}" | sed s#//*\#/\#g)
  DSTFILE=$(echo "${FULLVER}/${RUNSCRIPT}" | sed s#//*\#/\#g)
  # adb push doesn't like double slashes
  adb push $SRCFILE ${DSTFILE}
fi

case "$TYPE" in
  'load')
    SRCFILE=$(echo "${INFOLD}/${DEPLOYMENT}" | sed s#//*\#/\#g)
    DSTFILE=$(echo "${FULLVER}/${DEPLOYMENT}" | sed s#//*\#/\#g)
    # push the binary
    if [ -z "$NOPUSH" ]; then
      # adb push doesn't like double slashes
      adb push $SRCFILE ${DSTFILE}0
    fi
    get_crc32_from_cksum CRC_OUT $(cksum $SRCFILE)
    echo $CRC_OUT > ${SRCFILE}.crc
    adb push ${SRCFILE}.crc ${DSTFILE}.crc0
    rm ${SRCFILE}.crc

    adb shell "cp -p ${DSTFILE}0 ${DSTFILE}1; cp -p ${DSTFILE}0 ${DSTFILE}2; cp -p ${DSTFILE}.crc0 ${DSTFILE}.crc1; cp -p ${DSTFILE}.crc0 ${DSTFILE}.crc2"
  ;;
  'setcur')
    adb shell "rm $CURRENT_SYMLINK; ln -s $FULLVER $CURRENT_SYMLINK"
  ;;
  'setonce')
    adb shell "rm $RUN_ONCE_SYMLINK; ln -s $FULLVER $RUN_ONCE_SYMLINK"
  ;;
  *)
    evr 1 "Invalid FSW operation type $TYPE"
    exit 1
  ;;
esac
