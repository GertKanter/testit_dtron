Running the docker container
-----------
After a successful installation run TestIt base image with Docker:<br/>
`$ docker run --name testit -v /var/run/docker.sock:/var/run/docker.sock -it testit bash`<br/>
It is necessary to mount Docker socket to run Docker commands inside the container.

After you have started the container you can execute the example.

Executing the Single Robot example
-----------
Navigate to the “catkin_ws” directory and source files:<br/>
`$ cd /catkin_ws`<br/>
`$ source devel/setup.bash`

To start TestIt daemon, type next command:<br/>
`$ roslaunch dtron turtlebot.launch`

The file for navigation with simple *move_base* action server is specified by default. But it can be changed by passing the path to the configuration file in the parameter.

For example, if we want to test topological navigation:<br/>
`$ roslaunch dtron turtlebot.launch config:=/catkin_ws/src/testit/dtron/turtlebot/cfg/config_top_nav.yaml`

To set the state and to bring up open new terminal and type:<br/>
`$ rosrun testit testit_command.py bringup`

And finally, to start the test type:<br/>
`$ rosrun testit testit_command.py test`

It is possible to monitor what happens in containers by passing the container identifier to the parameter:

`$ docker attach CONTAINER-ID`

Executing the Multi-robot example
-----------
Navigate to the “catkin_ws” directory and source files:<br/>
`$ cd /catkin_ws`<br/>
`$ source devel/setup.bash`

To start TestIt daemon, type next command:<br/>
`$ roslaunch dtron turtlebot.launch config:=/catkin_ws/src/testit/dtron/turtlebot/cfg/config_stage.yaml`<br/>

To set the state and to bring up open new terminal and type:<br/>
`$ rosrun testit testit_command.py bringup`

And finally, to start the test type:<br/>
`$ rosrun testit testit_command.py test`
