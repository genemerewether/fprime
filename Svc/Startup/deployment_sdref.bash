#!/bin/bash

DEP_ENV=". /opt/ros/indigo/setup.bash || true"
DEPLOYMENT=SDREF

# boot counter is filled in by expand_fsw_args in startup.bash
FSW_ARGS="-p 50000 -x 50010 -a 192.168.1.2 -u 50020 -b"

pre_deployment() {
    bash -c "${DEP_ENV}; pgrep roscore || roscore"
}
