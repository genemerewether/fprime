#!/bin/bash

# This script does the following:
# inhibits Svc::Startup from launching FSW by creating /tmp/MAGIC_COOKIE

#adb wait-for-device; adb shell service startup stop

COMMAND="touch /tmp/MAGIC_COOKIE"
adb wait-for-device
adb shell $COMMAND
