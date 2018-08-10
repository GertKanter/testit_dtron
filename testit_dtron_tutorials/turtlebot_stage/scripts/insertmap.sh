#!/bin/bash

source /catkin_ws/devel/setup.bash

roslaunch mongodb_store mongodb_store.launch db_path:=/opt/ros/mongodb_store & \
sleep 10
rosrun topological_utils insert_map.py /catkin_ws/dtronpack/generateModelFromTmap/examples/map.tmap map map