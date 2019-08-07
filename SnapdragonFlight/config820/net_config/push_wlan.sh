#!/bin/sh
adb push wlan /etc/initscripts/wlan
adb push wpa_supplicant.conf /data/misc/wifi/wpa_supplicant.conf
adb shell "echo 'sta,ap' > /data/misc/wifi/wlan_mode"
adb push wlan_daemon.service /etc/systemd/system/wlan_daemon.service
adb shell 'chmod 644 /etc/systemd/system/wlan_daemon.service'
adb shell 'chmod 755 /etc/initscripts/wlan' 
