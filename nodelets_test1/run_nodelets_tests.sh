#!/bin/bash
roscore_runs=`ps -ef|grep roscore|grep -v grep`

typeset -i maxsize
typeset -i stepsize

if [[ -z "$1" || -z "$2" || -z "$3" ]]; then
	echo "Usage: $0 <maxsize> <stepsize> <use_message_pointer>"
	echo " where <use_message_pointer> can be true or false" 
	exit 1
fi

maxsize=$1
stepsize=$2
use_message_pointer=$3

typeset -i message_payload_size
typeset -i shutdown_after_sending
typeset -i publish_rate
typeset -i report_count_interval

message_payload_size=0
shutdown_after_sending=1800
publish_rate=60
report_count_interval=60

runstamp=`date +%F_%H%M%S`
while (($message_payload_size <= $maxsize)); do
	roscore_runs=`ps -ef|grep roscore|grep -v grep`
	if [ -z "$roscore_runs" ]; then
		echo "ERROR: start roscore first"
		exit 1
	fi
	roslaunch nodelets_test1 nodelets.launch shutdown_after_sending:=$shutdown_after_sending message_payload_size:=$message_payload_size publish_rate:=$publish_rate report_count_interval:=$report_count_interval use_message_pointer:=$use_message_pointer launch_prefix:=nohup
	mv ~/.ros/nohup.out ~/.ros/${runstamp}_nodelets-ms${message_payload_size}pr${publish_rate}rc${report_count_interval}sa${shutdown_after_sending}PT${use_message_pointer}
	message_payload_size+=$stepsize
done
