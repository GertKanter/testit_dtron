This package is designed to be built inside the TestIt DTRON Docker container (not as a stand-alone package) with prerequisites installed to specific paths.
# TestIt DTRON Adapter
## UPPAAL sync channels
The sync channels have prefixes `i_` for input and `o_` for output channels. The "direction" for input/output is w.r.t. SUT. This means that input is an input to the SUT and output is output from the SUT.

As the default adapter supports multi-robot systems, we have to also prefix the sync channel with the robot's name.

Currently, the response from SUT is always from the `response` channel. That is, for example when there is a robot named `robot_0`, the sync would be `o_robot_0_response`.

The adapter supports the following commands
### "goto"
This command gives the robot a goal to navigate to. It is possible to utilize the command using `topological_navigation` or traditional `move_base`. The waypoints must be previously loaded into ROS Param server in the following format for the `move_base` version to work.
```
test_adapter:
 node_map: {'ID_1': 'NODE_ID_1', 'ID_2': 'NODE_ID_2', ...}
 nodes:
  'NODE_ID_1': {x: COORD_X_1, COORD_Y_1}
  'NODE_ID_2': {x: COORD_X_2, COORD_Y_2}
  ...
```
Where `ID_*` is a positive integer (>=1), `NODE_ID_*` is a string identifier, `COORD_X_* and COORD_Y_*` are real values.

For example
```
test_adapter:
 node_map: {'1': ChargingPoint, '3': WayPoint1, '5': WayPoint2, '7': WayPoint3, '9': WayPoint4,
  '11': WayPoint5, '13': WayPoint6, '15': WayPoint7, '17': WayPoint8, '19': WayPoint9}
 nodes:
  ChargingPoint: {x: 28.6, y: 4.0}
  WayPoint1: {x: 28.3, y: 6.37}
  WayPoint2: {x: 26.7, y: 6.47}
  WayPoint3: {x: 24.1, y: 6.52}
  WayPoint4: {x: 25.8, y: 8.96}
  WayPoint5: {x: 24.2, y: 10.6}
  WayPoint6: {x: 24.4, y: 14.2}
  WayPoint7: {x: 26.0, y: 14.0}
  WayPoint8: {x: 26.0, y: 14.8}
  WayPoint9: {x: 29.3, y: 12.0}
```
To specify which version you wish to use (default is `topological_navigation`) you need to supply the parameter `mode`.
```
mode = 1 (or not defined) => topological_navigation goal
mode = 2 => move_base goal
```
This means that, using the previous waypoints configuration, for example, if the robot is named `robot_0`, we need to assign the parameter `mode`:
```
i_robot_0_goto_waypoint = 5
i_robot_0_goto_mode = 2
```
And then the robot named `robot_0` will move to coordinate `x = 26.7, y = 6.47` (`Waypoint2`) using `move_base` navigation.

If we only specify in the sync
```
i_robot_0_goto_waypoint = 17
```
Then the robot named `robot_0` will move to `x = 26.0, y = 14.8` (`Waypoint8`) using `topological_navigation` action server.

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
