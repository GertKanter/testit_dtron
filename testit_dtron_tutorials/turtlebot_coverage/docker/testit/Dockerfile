FROM testit_dtron:latest
LABEL Description="TestIt! ROS Testing toolkit docker image for turtlebot_coverage"

# clone and catkin_make your use case specific repositories etc
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils && \
	echo 'export PATH=$PATH:/catkin_ws/spread/sbin' >> ~/.bashrc && \
	echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
CMD bash
