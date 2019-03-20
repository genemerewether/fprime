#!/bin/sh

# This script does the following:
# sets the SD time based on host time using date -s

set -e
current_date=`date +%s`
if adb shell "date -s \"@${current_date}\" +%s" | grep -E "^${current_date}"
then
    echo "Set time on SD successfully"
else
    echo "Failed to set time on SD"
    exit
fi
