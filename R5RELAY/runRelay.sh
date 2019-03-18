#!/bin/csh

if !($?BUILD_ROOT) then
    set curdir = "${PWD}"
    setenv BUILD_ROOT `dirname $0`/../..
    cd $BUILD_ROOT
    setenv BUILD_ROOT ${PWD}
    cd ${curdir}
endif

./linux-linux-x86-debug-gnu-bin/R5RELAY -p 50000 -a localhost -l -s /dev/ttyUSB0
