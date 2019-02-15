#!/bin/sh

# check that time is sufficiently recent
if [ $(date +%s) -ge 1509000000 ]; then
    echo "time is after October 25, 2017 - manual sanity check"
else
    echo "time is too old - refusing to start"
    exit
fi

# start SW in screen so we can detach and reattach
# start SW in script so we can record interactive session
screen -S "SDREF" -d -m bash --norc
screen -S "SDREF" -X logfile $(date +"/eng/SDREF_%F_%H-%M.log")
screen -S "SDREF" -X log
screen -r "SDREF" -X stuff ". /opt/ros/indigo/setup.bash || true\n"
screen -r "SDREF" -X stuff "pgrep roscore || roscore &\n"
screen -r "SDREF" -X stuff "$(pwd)/SDREF0 -p 50000 -x 50010 -u 50020 -a 192.168.2.1\n"
screen -r "SDREF"
