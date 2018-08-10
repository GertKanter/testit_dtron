#!/bin/bash
source /catkin_ws/devel/setup.bash

echo "Start MongoDb"

roslaunch mongodb_store mongodb_store.launch db_path:=/opt/ros/mongodb_store &
sleep 10
echo "Start topological navigation"
roslaunch dtron topological_navigation.launch map:=map &

sleep 30
echo "Start adapter"
cd /catkin_ws/src/testit/dtron/turtlebot/scripts
source run_adapter.sh config_top_nav.yaml
