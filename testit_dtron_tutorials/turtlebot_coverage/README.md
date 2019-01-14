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
rosrun testit_dtron_tutorials build_containers_coverage.sh
```
#### Coverage support requirements
The main difference between the two tutorials (with and without coverage analysis) is that we need to install an additional package to the SUT stack. This package will allow TestIt to send a signal to flush coverage data and report the data from SUT back to TestIt. This allows us to gather information about the lines of code that are executed when transitioning from one state to another.
##### C++
Note that the SUT C++ stack that is being tested needs to be compiled with coverage support (`-fprofile-arcs -ftest-coverage`).

The packages that are being tested need to handle `SIGUSR1` signal and flush the coverage data upon receiving that signal.
To achieve this, it is possible to inject the following code into the `main()` function of the node.
```cpp
#include <signal.h>
extern "C" void __gcov_flush();
void flush(int sig) { __gcov_flush(); } // FLUSH ON SIGUSR1

int main(int argc, char **argv)
{
  signal(SIGUSR1, &flush); // INJECT HANDLER
  ...
}
```
##### Python
The Python stack requires `coverage` package to be installed.
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
### Coverage analysis
TBD
