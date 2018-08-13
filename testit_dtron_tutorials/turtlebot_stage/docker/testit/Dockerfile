FROM testit:latest
MAINTAINER Artur Gummel <artur.gummel@ttu.ee>
LABEL Description="TestIt! ROS Testing toolkit docker image"
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils && \
	echo 'export PATH=$PATH:/catkin_ws/spread/sbin' >> ~/.bashrc && \
	echo 'source /opt/ros/lunar/setup.bash' >> ~/.bashrc && \
#insert map to database
	/bin/bash -c "/catkin_ws/src/testit/dtron/scripts/insertmap.sh"
CMD bash