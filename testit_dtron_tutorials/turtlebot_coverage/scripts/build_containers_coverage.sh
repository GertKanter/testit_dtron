#!/bin/bash -e

echo "Building testit_dtron_tb_coverage_sut container..."
cd $(rospack find testit_dtron_tutorials)/turtlebot_coverage/docker/sut
docker build --no-cache -t testit_dtron_tb_coverage_sut .
echo "Building testit_dtron_tb_coverage_testit container..."
cd $(rospack find testit_dtron_tutorials)/turtlebot_coverage/docker/testit
docker build --no-cache -t testit_dtron_tb_coverage_testit .
