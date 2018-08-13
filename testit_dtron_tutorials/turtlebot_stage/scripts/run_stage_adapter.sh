#!/bin/bash
source /catkin_ws/devel/setup.bash

echo "Start Spread server"

cd /catkin_ws/spread/sbin
./spread -c /catkin_ws/src/testit/dtron/config/spread.conf &

sleep 5

echo "Start adapter"

roslaunch dtron stage_adapter.launch  &

echo "Wait 10 sec for adapter initialization"

sleep 20

echo "Start DTRON"
cd /catkin_ws/dtronpack/
source run_dtron_stage_test.sh
