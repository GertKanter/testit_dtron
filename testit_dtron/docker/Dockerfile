# VERSION 0.0.1
FROM testitros/testit:latest
LABEL Description="TestIt! DTRON extension Docker image"
RUN apt-get update && \
#install autoconf and scons
	apt-get install -y autoconf scons && \
#install Java 8, Lunar packages, mongodb
	apt-get install software-properties-common -y && \
	add-apt-repository ppa:webupd8team/java -y && \
	apt-get update && \
	echo debconf shared/accepted-oracle-license-v1-1 select true | debconf-set-selections && \
	echo debconf shared/accepted-oracle-license-v1-1 seen true | debconf-set-selections && \
	apt-get install oracle-java8-installer -y && \
	apt-get install -y ros-lunar-rviz && \
	apt-get install -y ros-lunar-map-server && \
	apt-get install -y ros-lunar-move-base-msgs && \
	apt-get install -y python-pymongo mongodb && \
	mkdir /opt/ros/mongodb_store && \
	apt-get install -y ros-lunar-navfn && \
	apt-get install -y ros-lunar-costmap-2d && \
#clone example for TestIt and STRANDS packages
	git clone https://github.com/arturgummel/dtrontestit.git /catkin_ws/src/testit/dtron && \
	git clone https://github.com/arturgummel/dtronpack.git /catkin_ws/dtronpack && \
	/bin/bash -c "/catkin_ws/src/testit/dtron/scripts/build_stuff.sh" && \
	cd /catkin_ws/src && \
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
	/bin/bash -c "source /opt/ros/lunar/setup.bash && cd /catkin_ws && catkin_make" && \
#configure Spread
	useradd spread && usermod -a -G spread spread && \
	mkdir -m 777 /var/run/spread && \
	echo 'export PATH=$PATH:/catkin_ws/spread/sbin' >> ~/.bashrc && \
	echo 'source /opt/ros/lunar/setup.bash' >> ~/.bashrc && \
#install docker
	apt-get update && \
	apt-get -y install apt-transport-https \
	ca-certificates \
	curl \
	gnupg2 \
	software-properties-common && \
	curl -fsSL https://download.docker.com/linux/$(. /etc/os-release; echo "$ID")/gpg > /tmp/dkey; apt-key add /tmp/dkey && \
	add-apt-repository \
	"deb [arch=amd64] https://download.docker.com/linux/$(. /etc/os-release; echo "$ID") \
	$(lsb_release -cs) \
	stable" && \
	apt-get update && \
	apt-get -y install docker-ce && \
#install required architecture for running TRON
	dpkg --add-architecture i386 && \
	apt-get update && \
	apt-get install -y libc6:i386 libncurses5:i386 libstdc++6:i386 && \
	echo '************************************************************************************' && \
	echo 'Download UPPAAL TRON manually from http://people.cs.aau.dk/~marius/tron/download.html' && \
	echo 'You will need to accept the license!'&& \
	echo 'unzip uppaal-tron-1.5-linux.zip in dtronpack directory:'&& \
	echo 'cd /catkin_ws/dtronpack && unzip uppaal-tron-1.5-linux.zip' && \
	echo '************************************************************************************'
CMD bash

#download UPPAAL TRON manually from http://people.cs.aau.dk/~marius/tron/download.html
#You will need to accept the license!
#unzip uppaal-tron-1.5-linux.zip in dtronpack directory:
#cd /catkin_ws/dtronpack && unzip uppaal-tron-1.5-linux.zip