#!/bin/bash

taskset -c 3 chrt -r 50 rosbag record -a
