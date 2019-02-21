FROM ros:kinetic-robot-xenial
MAINTAINER Gert Kanter <gert.kanter@taltech.ee>
LABEL Description="TestIt! ROS Testing toolkit tutorial SUT image"
# copy patches to docker context
COPY patches/*.patch /
# get dependencies and packages
RUN apt-get update && \
	apt-get install -y scons python-pip ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-image-geometry ros-kinetic-depth-image-proc ros-kinetic-joy ros-kinetic-kobuki-description ros-kinetic-stage-ros libbullet-dev libsdl-image1.2-dev libpcl-dev ros-kinetic-move-base-msgs ros-kinetic-tf2-geometry-msgs ros-kinetic-bfl ros-kinetic-pcl-ros && \
        pip install coverage && \
	mkdir -p /catkin_ws/src && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash && cd /catkin_ws/src && catkin_init_workspace" && \
	cd /catkin_ws/src && \
	git clone https://github.com/turtlebot/turtlebot_msgs && \
	git clone https://github.com/turtlebot/turtlebot_apps && \
	git clone https://github.com/turtlebot/turtlebot && \
	git clone https://github.com/GertKanter/robots_stage.git && \
        git clone https://github.com/GertKanter/testit_msgs.git && \
        git clone https://github.com/GertKanter/testit_sut.git -b develop && \
        git clone https://github.com/GertKanter/autosys-object-detector && \
        git clone https://github.com/ros-planning/navigation -b kinetic-devel && \
# clone STRANDS packages
        apt-get install -y python-pymongo mongodb && \
        mkdir /opt/ros/mongodb_store && \
        apt-get install -y ros-kinetic-rviz && \
        git clone https://github.com/strands-project/strands_navigation.git && \
        git clone https://github.com/strands-project/mongodb_store.git && \
        cd /catkin_ws/src/mongodb_store && \
        git checkout 09d587f53dc2c5ebe5865d9ec98af447fd91fbe4 && \
        cd /catkin_ws/src && \
        git clone https://github.com/strands-project/strands_apps.git && \
        git clone https://github.com/strands-project/strands_movebase && \
        git clone https://github.com/strands-project/fremen.git && \
        mv /catkin_ws/src/fremen/FremenServer /catkin_ws/src/strands_navigation/ && \
        rm -r /catkin_ws/src/fremen && \
        git clone https://github.com/GT-RAIL/robot_pose_publisher.git && \
# patch packages that we want to track coverage of
        cd / && \
        for i in /*.patch; do patch -p0 < $i; done && \
# build the packages
	cd /catkin_ws && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash && cd /catkin_ws && catkin_make -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage'"
        #insert map to database
	#/bin/bash -c "source /catkin_ws/devel/setup.bash && rosrun robots_stage insertmap.sh"
CMD bash
