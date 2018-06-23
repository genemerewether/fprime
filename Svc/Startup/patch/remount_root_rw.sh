#!/bin/sh

# DON'T USE THIS ON ECMs - they have a different partition map

# This script does the following:
# remounts the root partition on the NAV read-write

set -e
adb shell 'mount -o remount,rw /dev/mmcblk0p13 / -t ext4'
