#!/bin/bash

# check that time is sufficiently recent
if [ $(date +%s) -ge 1509000000 ]; then
    echo "time is after October 25, 2017 - manual sanity check"
else
    echo "time is too old - refusing to start"
    exit
fi

insmod /golden/dsp-offset_taskset_cpustat_mqueue.ko
export ROS_HOME=/eng/ros
export ROS_PACKAGE_PATH=/home/root/quest_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
export PYTHONPATH=/home/root/quest_ws/devel/lib/python2.7/site-packages:/opt/ros/indigo/lib/python2.7/site-packages
export ROS_ETC_DIR=/opt/ros/indigo/etc/ros
export ROS_ROOT=/opt/ros
export ROS_DISTRO=indigo
export PATH=$PATH:/opt/ros/indigo/bin
export LD_LIBRARY_PATH=/opt/ros/indigo/lib
export ROS_MASTER_URI=http://localhost:11311
export CMAKE_PREFIX_PATH=/opt/ros/indigo
#touch /opt/ros/indigo/.catkin

sysctl -w kernel.sched_rt_runtime_us=-1
#pgrep roscore || roscore &
$(pwd)/CARREF0 -p 50100 -u 50020 -z localhost:50030 -a localhost -s
