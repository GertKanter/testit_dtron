This package is designed to be built inside the TestIt DTRON Docker container (not as a stand-alone package) with prerequisites installed to specific paths.
# TestIt DTRON Adapter
## UPPAAL sync channels
The sync channels have prefixes `i_` for input and `o_` for output channels. The "direction" for input/output is w.r.t. SUT. This means that input is an input to the SUT and output is output from the SUT.

As the default adapter supports multi-robot systems, we have to also prefix the sync channel with the robot's name.

Currently, the response from SUT is always from the `response` channel. That is, for example when there is a robot named `robot_0`, the sync would be `o_robot_0_response`.

The adapter supports the following commands
### "moveto"
This command gives the robot a simple move_base goal with parameters `x`, `y` and `a`.
The parameter values are encoded as integers but they are divided by 10 before passing to ROS (e.g., 123 becomes 12.3, 1 becomes 0.1).
Parameters `x` and `y` are in meters and `a` is in radians. The rotation is counterclockwise (e.g., 1.6 is about 90 degrees counterclockwise).
For example, if the robot is named `robot_0`, we need to first assign values to the parameters:
```
i_robot_0_moveto_x = 10
i_robot_0_moveto_y = 15
i_robot_0_moveto_a = 16
```
This will give the robot the goal `x = 1.0 (meters)`, `y = 1.5 (meters)` and `a = 1.6 (radians)`.

Then we can call the sync channel:
```
i_robot_0_moveto!
```
The adapter will always return the value `1` for this sync even if the goal is not reached (error or unreachable pose).

### "object_detect"
This command retrieves robot's object detector current value.
If the robot's object detector node is currently detecting an object, the sync returns value `2` and if the node is not detecting an object, it returns value `1`.
The value is returned as a variable called `value` for the `response` output sync.
```
o_robot_0_response_value
```