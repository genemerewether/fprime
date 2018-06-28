#!/bin/sh

# DON'T USE THIS ON ECMs - they have a different partition map

# This script does the following:
# remounts the fsw partition on the NAV read-write

set -e
adb shell 'mount -o remount,rw /dev/mmcblk0p23 /fsw -t ext4'
