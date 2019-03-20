#!/bin/sh

set -e

mkpart() {
    if adb shell mount | grep $DATA_PART
    then
        echo "$DATA_PART is mounted - unmount manually and acknowledge that all data will be erased"
        exit 1
    fi

    adb shell "screen mkfs.ext4 /dev/${DATA_PART}"
    adb shell "mkdir -p $MOUNT_POINT"
    adb shell "mount -t ext4 /dev/${DATA_PART} $MOUNT_POINT"

    if adb shell mount | grep $DATA_PART
    then
        echo "$DATA_PART filesystem created and mounted successfully"
    else
        echo "$DATA_PART filesystem create or mount step failed"
        exit 1
    fi

    if adb shell "cat /etc/fstab" | grep $DATA_PART
    then
        echo "fstab already has an entry for $DATA_PART"
    else
        adb shell "echo \"/dev/${DATA_PART} $MOUNT_POINT ext4 defaults 0 0\" >> /etc/fstab"
    fi
}

adb shell 'mount -o remount,rw /dev/mmcblk0p13 / -t ext4'

DATA_PART="mmcblk0p19"
MOUNT_POINT="/seq"
mkpart

DATA_PART="mmcblk0p20"
MOUNT_POINT="/img"
mkpart

DATA_PART="mmcblk0p21"
MOUNT_POINT="/eng"
mkpart

DATA_PART="mmcblk0p22"
MOUNT_POINT="/log"
mkpart

DATA_PART="mmcblk0p23"
MOUNT_POINT="/fsw"
mkpart

DATA_PART="mmcblk0p24"
MOUNT_POINT="/video"
mkpart
