#!/bin/bash

# This script does the following:
# polls whether board is up to configure before Svc::Startup launches FSW

UP_CHECK_SECS=1 # time to sleep between ping attempts
POST_PING_SLEEP_SECS=2 # time between ping up and ssh
NUM_PINGS=3
PING_TIMEOUT=1

# redirect to file to capture output if desired
REDIR_DEV="/dev/null"
DISCARD_DEV="/dev/null"
#DISCARD_DEV="verbose.txt"

ETH_IP="192.168.2.20"

DIV_TEXT="################################################################################"
SUBDIV_TEXT="--------------------------------------------------------------------------------"

cleanup ()
{
    kill -9 $!
    exit 0
}

trap cleanup SIGINT SIGTERM

adb disconnect

while [ 1 ]; do
    ping -i 0.2 -W $PING_TIMEOUT -c $NUM_PINGS $ETH_IP | tee -a $REDIR_DEV | grep "64 bytes from $ETH_IP" >> $DISCARD_DEV &
    if wait $!; then
        sleep $POST_PING_SLEEP_SECS &
        wait $!
        adb connect ${ETH_IP} &
        wait $!
        if wait $!; then break; fi
        adb disconnect
    fi

    sleep $UP_CHECK_SECS 2>&1 > /dev/null &
    wait $!

    echo "$ETH_IP not connected yet; sleeping $UP_CHECK_SECS seconds" | tee -a $REDIR_DEV
done

current_date=`date +%s`
if adb shell "date -s \"@${current_date}\" +%s" | grep -E "^${current_date}"
then
    echo "Set time on SD successfully"
else
    echo "Failed to set time on SD"
    exit
fi
