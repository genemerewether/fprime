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
screen -r "SDREF" -X stuff "insmod /golden/dsp-offset_taskset_cpustat_mqueue.ko || true\n"
screen -r "SDREF" -X stuff "export ROS_HOME=/eng/ros\n"
screen -r "SDREF" -X stuff ". /eng/load/set_ip_snap.bash || true\n"
screen -r "SDREF" -X stuff ". /opt/ros/indigo/setup.bash || true\n"
screen -r "SDREF" -X stuff "rostopic list || roscore &\n"
screen -r "SDREF" -X stuff "gdb /golden/SDREF0\n"
screen -r "SDREF" -X stuff "handle SIGILL pass noprint nostop\n"
screen -r "SDREF" -X stuff "r -p 50000 -x 50010 -u 50020 -z localhost:50030 -a localhost -s"
screen -r "SDREF"
