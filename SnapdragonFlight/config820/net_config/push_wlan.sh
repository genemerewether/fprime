#!/bin/sh
adb push wlan /etc/init.d/wlan
adb push wpa_supplicant.conf /data/misc/wifi/wpa_supplicant.conf
adb shell 'echo softap > /data/misc/wifi/wlan_mode'
adb push wlan_daemon.service /etc/systemd/system/wlan_daemon.service
