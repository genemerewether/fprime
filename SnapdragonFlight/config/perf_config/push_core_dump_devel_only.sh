#!/bin/sh

# enable core dumps for login shell
adb shell 'grep "ulimit -c unlimited" /root/.bashrc || echo "ulimit -c unlimited" >> /root/.bashrc'
adb shell 'grep "core_pattern" /root/.bashrc || echo "echo /video/core_%E_%t > /proc/sys/kernel/core_pattern" >> /root/.bashrc'
