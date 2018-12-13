#!/bin/bash

echo "Building testit_dtron_tb_sut container..."
cd $(rospack find testit_dtron_tutorials)/turtlebot_stage/docker/sut
docker build --no-cache -t testit_dtron_tb_sut .
echo "Building testit_tb_testit container..."
cd $(rospack find testit_dtron_tutorials)/turtlebot_stage/docker/testit
docker build --no-cache -t testit_dtron_tb_testit .
