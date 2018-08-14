FROM ros:kinetic-robot-xenial
MAINTAINER Gert Kanter <gert.kanter@ttu.ee>
LABEL Description="TestIt! ROS Testing toolkit tutorial SUT image"
RUN apt-get update && \
	apt-get install -y ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-image-geometry ros-kinetic-depth-image-proc ros-kinetic-joy ros-kinetic-kobuki-description ros-kinetic-navigation ros-kinetic-stage-ros && \
	mkdir -p /catkin_ws/src && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash && cd /catkin_ws/src && catkin_init_workspace" && \
	cd /catkin_ws/src && \
	git clone https://github.com/turtlebot/turtlebot_msgs && \
	git clone https://github.com/turtlebot/turtlebot_apps && \
	git clone https://github.com/turtlebot/turtlebot && \
	git clone https://github.com/GertKanter/robots_stage.git && \
	cd .. && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash && cd /catkin_ws && catkin_make"
CMD ['/bin/bash', ' -c "source /catkin_ws/devel/setup.bash && rosrun mission_control start_move_base_in_docker.sh && tail -f /dev/null"']
