#!/bin/sh

set -e

mkpart() {
    adb shell "mkdir -p /data/$MOUNT_POINT"
    #adb shell "ln -s /data/$MOUNT_POINT /$MOUNT_POINT"
}

MOUNT_POINT="seq"
mkpart

MOUNT_POINT="img"
mkpart

MOUNT_POINT="eng"
mkpart

MOUNT_POINT="log"
mkpart

MOUNT_POINT="fsw"
mkpart

MOUNT_POINT="video"
mkpart
