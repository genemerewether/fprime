#!/bin/sh

# prevent mpdecision from starting by default
adb push mpdecision.conf /etc/init/mpdecision.conf

# set scaling_governor to performance; disable power saving on L2 cache
adb push post-boot.conf /etc/init/post-boot.conf
