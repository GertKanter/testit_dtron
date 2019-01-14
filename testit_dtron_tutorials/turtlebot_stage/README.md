TestIt DTRON: a DTRON extension for the Scalable Long-Term Autonomy Testing Toolkit for ROS tutorials
=====================================================================================================

## Turtlebot navigation
This tutorial demonstrates how we can test Turtlebot navigation in the Stage simulator using model-based testing.
### Prerequisites
#### Dependencies
You need to have [Docker](https://www.docker.com/) installed for this tutorial.
#### Packages
For this tutorial, you also need the `testit` package. You can clone the repository using this command:
```
cd ~/catkin_ws/src
git clone https://github.com/GertKanter/testit
```
#### Docker containers
This tutorial is based on two containers: SUT (System Under Test) and TestIt containers. The SUT container represents the full robot software stack. TestIt itself can be configured to test against any specific configuration, but for the purposes of this tutorial we have placed the full software stack in the SUT container.

The TestIt container is based on `testitros/testit:latest`, make sure to build that before with the command:
```
rosrun testit build_local_testit_docker.sh
```
In order to run the tutorial, we need to build the containers so that TestIt can start both the SUT and the SUT specific TestIt container. We can do it with the following command:
```
rosrun testit_dtron_tutorials build_containers.sh
```
### Running the daemon
To run this tutorial we can launch the `turtlebot.launch` file like this:
```
roslaunch testit_dtron_tutorials turtlebot.launch
```
This command will launch the TestIt daemon with the specific configuration for this tutorial.

### Executing the tests
To execute the tests, the infrastructure must be initialized (e.g., start the cloud test servers) with the following command:
```
rosrun testit testit_command.py bringup
```
After bringup, we can execute the tests with this command:
```
rosrun testit testit_command.py test
```
