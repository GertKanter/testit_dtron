#!/bin/bash
source /catkin_ws/devel/setup.bash

TOPNAV="$1"

echo "Start Spread server"

cd /catkin_ws/spread/sbin
./spread -c /catkin_ws/src/testit/dtron/config/spread.conf &

sleep 5

echo "Start adapter"

roslaunch dtron adapter.launch config_file:=$TOPNAV &

echo "Wait 10 sec for adapter initialization"

sleep 20

echo "Start DTRON"
cd /catkin_ws/dtronpack
source run_dtron_test.sh
