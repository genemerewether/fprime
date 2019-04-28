#!/bin/bash

rostopic pub /flysafe mav_msgs/BoolStamped -r 10 -s "header:
  seq: auto
  stamp: now
  frame_id: ''
data:
  data: true"
