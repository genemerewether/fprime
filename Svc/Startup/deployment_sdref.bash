#!/bin/bash

DEP_ENV="export ROS_HOME=/eng/ros; . /opt/ros/indigo/setup.bash; . /eng/load/set_ip_snap.bash"
DEPLOYMENT=SDREF

# boot counter is filled in by expand_fsw_args in startup.bash
FSW_ARGS="-p 50000 -x 50010 -a localhost -u 50020 -d -b"

pre_deployment() {
    insmod /golden/dsp-offset_taskset_cpustat_mqueue.ko
    bash -c "${DEP_ENV}; pgrep roscore || roscore"
}
