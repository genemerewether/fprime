#!/bin/bash

rostopic pub -r 100 -s /ackermann_cmd ackermann_msgs/AckermannDriveStamped "header:
  seq: auto
  stamp: now
  frame_id: ''
drive: {steering_angle: -10.0, steering_angle_velocity: 0.0, speed: 0.0, acceleration: 0.0,
  jerk: 0.0}" 
