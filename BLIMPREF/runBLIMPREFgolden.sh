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
screen -S "BLIMPREF" -d -m bash --norc
screen -S "BLIMPREF" -X logfile $(date +"/eng/BLIMPREF_%F_%H-%M.log")
screen -S "BLIMPREF" -X log
screen -r "BLIMPREF" -X stuff "export ROS_HOME=/eng/ros\n"
screen -r "BLIMPREF" -X stuff ". /eng/load/set_ip_snap.bash || true\n"
screen -r "BLIMPREF" -X stuff ". /opt/ros/indigo/setup.bash || true\n"
screen -r "BLIMPREF" -X stuff "rostopic list || roscore &\n"
screen -r "BLIMPREF" -X stuff "/golden/BLIMPREF0 -p 50000 -a localhost -s\n"
screen -r "BLIMPREF"
