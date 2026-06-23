# second_project

`second_project` is a ROS 2 C++ package for robot mapping and navigation. It includes SLAM configuration, Stage simulation worlds, Nav2 parameters, map files, RViz configurations, and a node that automatically sends navigation goals from a CSV file.

## Overview

- Converts a 3D point cloud into a 2D laser scan using `pointcloud_to_laserscan`.
- Runs online mapping with `slam_toolbox`.
- Loads simulation worlds with Stage.
- Runs Nav2 for AMCL localization, global planning, local control, and obstacle avoidance.
- Reads goals from `csv/goals.csv` and sends them sequentially to `/navigate_to_pose`.
- Publishes `/waypoints` markers in RViz to show the current goal.
- When navigation fails, clears costmaps and performs a short velocity-based reverse escape before retrying.

## Package Structure

```text
second_project
├── CMakeLists.txt
├── package.xml
├── config
│   ├── nav2_params.yaml
│   └── slam_toolbox_mapping.yaml
├── csv
│   └── goals.csv
├── launch
│   ├── mapping.launch.py
│   ├── navigation.launch.py
│   ├── stage_test.launch.py
│   ├── empty_stage_test.launch.py
│   ├── floorplan_test.launch.py
│   └── floorplan_no_include.launch.py
├── map
│   ├── task1_map.yaml
│   ├── task1_map.pgm
│   └── tiny_stage_map.pgm
├── rviz
│   ├── mapping.rviz
│   └── nav2_config.rviz
├── src
│   └── goal_publisher_node.cpp
└── world
    ├── scout_mini.world
    ├── empty_test.world
    ├── floorplan_test.world
    └── floorplan_no_include.world
```

## Build

From the root of the ROS 2 workspace:

```bash
colcon build --packages-select second_project
source install/setup.bash
```

## Task 1: SLAM Mapping

Start mapping:

```bash
ros2 launch second_project mapping.launch.py
```

This launch file starts:

- `pointcloud_to_laserscan_node`
- `async_slam_toolbox_node`
- `rviz2`

Main data flow:

```text
/ugv/rslidar_points -> pointcloud_to_laserscan -> /base_scan -> slam_toolbox
```

Key configuration files:

- `config/slam_toolbox_mapping.yaml`
- `rviz/mapping.rviz`

SLAM frames:

- `map_frame`: `map`
- `odom_frame`: `UGV_odom`
- `base_frame`: `UGV_base_link`
- `scan_topic`: `/base_scan`

To save a map:

```bash
ros2 run nav2_map_server map_saver_cli -f task1_map
```

## Task 2: Stage + Nav2 Navigation

Start the complete navigation stack:

```bash
ros2 launch second_project navigation.launch.py
```

This launch file starts:

- Stage simulation, loading `world/scout_mini.world`
- Nav2 bringup, loading `map/task1_map.yaml` and `config/nav2_params.yaml`
- RViz, loading `rviz/nav2_config.rviz`
- `goal_publisher_node`, reading `csv/goals.csv` and sending goals automatically

Startup timing:

- Stage starts immediately.
- Nav2 starts after 3 seconds.
- RViz starts after 6 seconds.
- The goal publisher starts after 10 seconds.

## Goal CSV

The goal file is located at:

```text
csv/goals.csv
```

Format:

```csv
x,y,theta
1.6,-4.4,-0.013
12.3,-4.0,1.513
```

Field descriptions:

- `x`: target x coordinate in the map frame.
- `y`: target y coordinate in the map frame.
- `theta`: target heading in radians.

After editing the goals, rebuild the package or make sure the installed resource file is updated:

```bash
colcon build --packages-select second_project
source install/setup.bash
```

## goal_publisher_node

Source: `src/goal_publisher_node.cpp`

This node reads `csv/goals.csv` from the installed package share directory and sends each goal to the Nav2 action server `/navigate_to_pose`.

Interfaces:

- Action client: `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
- Service client: `/global_costmap/clear_entirely_global_costmap`
- Service client: `/local_costmap/clear_entirely_local_costmap`
- Publishes: `/waypoints` (`visualization_msgs/msg/MarkerArray`)
- Publishes: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Subscribes to: `/cmd_vel` (`geometry_msgs/msg/Twist`)

Navigation result handling:

- Success: clears costmaps and sends the next goal.
- Canceled: clears costmaps and retries the current goal.
- Aborted: retries up to 2 times; before retrying, the node briefly moves in the reverse direction of the recent average `/cmd_vel` to escape local blockage.
- Too many failures: skips the current goal and continues with the next one.

## Stage Test Launch Files

Start the main Stage world only:

```bash
ros2 launch second_project stage_test.launch.py
```

Start the empty test world:

```bash
ros2 launch second_project empty_stage_test.launch.py
```

Start the floorplan test world:

```bash
ros2 launch second_project floorplan_test.launch.py
```

Start the floorplan world without includes:

```bash
ros2 launch second_project floorplan_no_include.launch.py
```

## Debugging Commands

Check the Nav2 action:

```bash
ros2 action list
ros2 action info /navigate_to_pose
```

Inspect goal markers:

```bash
ros2 topic echo /waypoints
```

Inspect velocity commands:

```bash
ros2 topic echo /cmd_vel
```

Inspect lifecycle nodes:

```bash
ros2 lifecycle nodes
```

Generate the TF tree:

```bash
ros2 run tf2_tools view_frames
```

## Notes

- This package uses `use_sim_time:=true` by default, so `/clock` must be published.
- In `navigation.launch.py`, AMCL provides `map -> odom`; no additional static `map -> odom` transform is required.
- The local planner in `nav2_params.yaml` is configured for holonomic motion, which helps the Stage robot move sideways in narrow passages.
- If localization or planning behaves unexpectedly, first check that the map origin, Stage initial pose, and AMCL initial pose are consistent.
