#!/bin/bash

rostopic pub -r 100 /flight_actuators_command mav_msgs/Actuators "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
angles:
- 0
angular_velocities: [120,120,120,120,120,120,120,120]
normalized:
- 0"
