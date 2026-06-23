# Robotics Project

This repository contains two ROS 2 packages:

- `first_project`: odometry estimation from tracked-vehicle motor RPMs, with TF-based error evaluation against ground truth.
- `second_project`: SLAM mapping, Stage simulation, Nav2 navigation, and automatic CSV-based navigation goal publishing.

Both packages use `ament_cmake` and are intended to be built inside the `src/` folder of a ROS 2 workspace.

## Dependencies

The project uses ROS 2, C++, and standard robotics mapping/navigation components. Make sure the following tools and packages are available:

- ROS 2 Humble or a compatible ROS 2 distribution
- `colcon`
- `rviz2`
- `tf2_ros`
- `slam_toolbox`
- `pointcloud_to_laserscan`
- `nav2_bringup` and related Nav2 packages
- `stage_ros2`
- `bunker_msgs`

On Ubuntu with ROS 2 Humble, common dependencies can be installed with:

```bash
sudo apt update
sudo apt install \
  ros-humble-rviz2 \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup
```

If `stage_ros2` or `bunker_msgs` are not available through apt, install them using the method provided by the course or lab environment.

## Build

From the root of the ROS 2 workspace:

```bash
colcon build --packages-select first_project second_project
source install/setup.bash
```

To build only one package:

```bash
colcon build --packages-select first_project
colcon build --packages-select second_project
```

## first_project: Odometry and TF Error Evaluation

`first_project` reads left and right motor RPMs from `/bunker_status`, integrates a custom odometry estimate, publishes `/project_odom`, and broadcasts the estimated TF `odom -> base_link2`. A second node compares `odom -> base_link` with `odom -> base_link2` and publishes the position error on `/tf_error_msg`.

Run:

```bash
ros2 launch first_project first_project.launch.py
```

Main nodes:

- `odometer`: subscribes to `/odom` for initial pose, subscribes to `/bunker_status` for RPM-based integration, and publishes `/project_odom` plus the `base_link2` TF.
- `tf_error`: reads TF transforms and computes the Euclidean position error between the estimated pose and ground truth.

Main interfaces:

- Subscribes to: `/odom`, `/bunker_status`
- Publishes: `/project_odom`, `/tf_error_msg`
- Service: `/odometer/reset`
- TF: `odom -> base_link2`

See [first_project/README.md](first_project/README.md) for details.

## second_project: Mapping and Navigation

`second_project` contains two main tasks:

- Mapping: converts the 3D point cloud `/ugv/rslidar_points` into the 2D LaserScan topic `/base_scan`, then runs SLAM with `slam_toolbox`.
- Navigation: starts Stage simulation, loads an existing map, runs Nav2, and uses `goal_publisher_node` to send goals from `csv/goals.csv` in sequence.

Mapping:

```bash
ros2 launch second_project mapping.launch.py
```

Navigation:

```bash
ros2 launch second_project navigation.launch.py
```

Stage-only test:

```bash
ros2 launch second_project stage_test.launch.py
```

Goal CSV format:

```csv
x,y,theta
1.6,-4.4,-0.013
12.3,-4.0,1.513
```

See [second_project/README.md](second_project/README.md) for details.

## Repository Structure

```text
.
├── first_project
│   ├── launch
│   ├── msg
│   ├── rviz
│   ├── screenshots
│   └── src
└── second_project
    ├── config
    ├── csv
    ├── launch
    ├── map
    ├── rviz
    ├── src
    └── world
```

## Troubleshooting

If RViz does not show TF or the map, first make sure the workspace has been sourced and that all nodes are using simulation time:

```bash
ros2 param get /rviz2 use_sim_time
ros2 topic echo /clock
```

If Nav2 keeps waiting for the action server, check that Stage, Nav2 bringup, and `/navigate_to_pose` are running correctly:

```bash
ros2 action list
ros2 lifecycle nodes
```
