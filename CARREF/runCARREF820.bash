#!/bin/bash

# check that time is sufficiently recent
if [ $(date +%s) -ge 1509000000 ]; then
    echo "time is after October 25, 2017 - manual sanity check"
else
    echo "time is too old - refusing to start"
    exit
fi

rostopic list || exit 1
$(pwd)/CARREF0 -p 50100 -u 50020 -z localhost:50030 -a localhost -s
