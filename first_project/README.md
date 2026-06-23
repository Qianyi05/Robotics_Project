# first_project

`first_project` is a ROS 2 C++ package for estimating tracked-vehicle odometry from Bunker status messages and evaluating the estimated TF against a ground-truth TF.

## Overview

- Reads the initial pose from `/odom`.
- Reads left and right motor RPMs from `/bunker_status`.
- Converts RPM values into linear and angular velocity, then integrates the robot pose.
- Publishes custom odometry on `/project_odom`.
- Publishes the estimated TF: `odom -> base_link2`.
- Compares the ground-truth TF `odom -> base_link` with the estimated TF `odom -> base_link2`.
- Publishes error information on `/tf_error_msg`.
- Provides a service for resetting the estimated odometry.

## Package Structure

```text
first_project
├── CMakeLists.txt
├── package.xml
├── launch
│   └── first_project.launch.py
├── msg
│   └── TfErrorMsg.msg
├── rviz
│   └── first_project.rviz
├── screenshots
└── src
    ├── odometer.cpp
    └── tf_error.cpp
```

## Nodes

### odometer

Source: `src/odometer.cpp`

The `odometer` node estimates robot motion by integrating the motor RPM data from `/bunker_status`.

Subscriptions:

- `/odom` (`nav_msgs/msg/Odometry`): used to initialize the robot position and orientation.
- `/bunker_status` (`bunker_msgs/msg/BunkerStatus`): used to read left and right motor RPMs.

Publications:

- `/project_odom` (`nav_msgs/msg/Odometry`): estimated odometry.
- TF `odom -> base_link2`: estimated robot pose.

Service:

- `/odometer/reset` (`std_srvs/srv/Empty`): resets the estimated pose to `(0, 0, 0)` and restarts integration timing.

Main parameters:

| Parameter | Default | Description |
| --- | ---: | --- |
| `kv_left` | `0.00117` | Scale factor from left RPM to linear velocity |
| `kv_right` | `0.00117` | Scale factor from right RPM to linear velocity |
| `kw` | `0.00167` | Scale factor from RPM difference to angular velocity |
| `angular_bias` | `0.0` | Angular velocity bias |
| `rpm_diff_deadband` | `5.0` | RPM difference deadband; angular velocity is set to zero below this value |
| `invert_left` | `false` | Invert the sign of the left RPM |
| `invert_right` | `false` | Invert the sign of the right RPM |
| `swap_left_right` | `false` | Swap left and right motor readings |

### tf_error

Source: `src/tf_error.cpp`

The `tf_error` node periodically reads two TF transforms and computes the 2D position error:

- Ground truth: `odom -> base_link`
- Project estimate: `odom -> base_link2`

Publication:

- `/tf_error_msg` (`first_project/msg/TfErrorMsg`)

Message format:

```text
std_msgs/Header header
float32 tf_error
int32 time_from_start
float32 travelled_distance
```

Field descriptions:

- `tf_error`: Euclidean distance between ground truth and estimated position, in meters.
- `time_from_start`: elapsed time since the first successful TF lookup, in seconds.
- `travelled_distance`: accumulated distance traveled according to the estimated trajectory, in meters.

## Build

From the root of the ROS 2 workspace:

```bash
colcon build --packages-select first_project
source install/setup.bash
```

## Run

Start the nodes and RViz:

```bash
ros2 launch first_project first_project.launch.py
```

This launch file starts:

- `odometer`
- `tf_error`
- `rviz2`, loading `rviz/first_project.rviz`

## Debugging Commands

Inspect the estimated odometry:

```bash
ros2 topic echo /project_odom
```

Inspect the TF error:

```bash
ros2 topic echo /tf_error_msg
```

Generate the TF tree:

```bash
ros2 run tf2_tools view_frames
```

Reset odometry:

```bash
ros2 service call /odometer/reset std_srvs/srv/Empty {}
```

## Notes

- This package uses `use_sim_time:=true` by default, which is suitable for rosbag playback or simulation.
- `tf_error` requires both `odom -> base_link` and `odom -> base_link2` to exist; otherwise it will wait and print warnings.
- If the estimated trajectory moves in the wrong direction, first check the `invert_left`, `invert_right`, and `swap_left_right` parameters.
