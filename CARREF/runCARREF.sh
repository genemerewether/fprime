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
screen -S "CARREF" -d -m bash --norc
screen -S "CARREF" -X logfile $(date +"/eng/CARREF_%F_%H-%M.log")
screen -S "CARREF" -X log
screen -r "CARREF" -X stuff "export ROS_HOME=/eng/ros\n"
screen -r "CARREF" -X stuff ". /eng/load/set_ip_snap.bash || true\n"
screen -r "CARREF" -X stuff ". /opt/ros/indigo/setup.bash || true\n"
screen -r "CARREF" -X stuff "pgrep roscore || roscore &\n"
screen -r "CARREF" -X stuff "$(pwd)/CARREF0 -p 50100 -u 50020 -z localhost:50030 -a localhost -s\n"
screen -r "CARREF"
