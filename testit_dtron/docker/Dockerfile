# VERSION 0.0.1
FROM testitros/testit:latest
LABEL Description="TestIt! DTRON extension Docker image"
# copy patches to docker context
COPY patches/*.patch /
RUN apt-get update && \
        # Install Java 8
        apt-get install -y scons libfftw3-dev openjdk-8-jdk openjdk-8-jre && \
        # Install TestIt DTRON extension
        cd /catkin_ws/src && \
	git clone https://github.com/GertKanter/testit_dtron -b develop && \
        # Install Spread and Google Protobuf
	/bin/bash -c "/catkin_ws/src/testit_dtron/testit_dtron/scripts/build_inside_docker.sh" && \
        # clone STRANDS packages to support topological navigation
	git clone https://github.com/strands-project/strands_navigation.git && \
        apt-get install -y ros-melodic-costmap-2d ros-melodic-navfn ros-melodic-move-base-msgs ros-melodic-octomap-server && \
	git clone https://github.com/strands-project/mongodb_store.git && \
	git clone https://github.com/strands-project/strands_apps.git && \
	git clone https://github.com/strands-project/strands_movebase && \
	git clone https://github.com/strands-project/fremen.git && \
        # Build packages
	/bin/bash -c "source /opt/ros/melodic/setup.bash && cd /catkin_ws && catkin_make -DBUILD_TESTIT_DTRON_PACKAGE=ON -DCMAKE_CXX_FLAGS=-std=c++11 || true" && \
        # patch packages
        cd / && \
        for i in /*.patch; do patch -p0 < $i; done && \
        # Build packages
	/bin/bash -c "source /opt/ros/melodic/setup.bash && cd /catkin_ws && catkin_make -DBUILD_TESTIT_DTRON_PACKAGE=ON -DCMAKE_CXX_FLAGS=-std=c++11" && \
# configure Spread
	useradd spread && usermod -a -G spread spread && \
	mkdir -m 777 /var/run/spread && \
	echo 'export PATH=$PATH:/catkin_ws/spread/sbin' >> ~/.bashrc && \
	echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc && \
# install required architecture for running TRON
	dpkg --add-architecture i386 && \
	apt-get update && \
	apt-get install -y libc6:i386 libncurses5:i386 libstdc++6:i386 && \
	echo '************************************************************************************' && \
	echo 'Download UPPAAL TRON manually from http://people.cs.aau.dk/~marius/tron/download.html' && \
	echo 'You will need to accept the license!'&& \
	echo 'unzip uppaal-tron-1.5-linux.zip in testit_dtron/tron directory:'&& \
	echo 'cd /catkin_ws/src/testit_dtron/testit_dtron/tron && unzip uppaal-tron-1.5-linux.zip' && \
	echo '************************************************************************************'
CMD bash
