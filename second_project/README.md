# second_project

ROS 2 package for the second robotics project:

- Task 1: build a map from the provided bag using `pointcloud_to_laserscan` and `slam_toolbox`.
- Task 2: run Stage simulation, Nav2, RViz, and the CSV goal publisher.

## External Source Dependencies

This project uses `stage_ros2` from the TU Wien robotics repository:

```bash
cd ~/colcon_ws/src
git clone https://github.com/tuw-robotics/stage_ros2.git
```

The TUW `stage_ros2` package also requires `marker_msgs`. Make sure `marker_msgs`
is available in the same ROS 2 workspace or installed in the environment before
building.

## Build

From the workspace root:

```bash
cd ~/colcon_ws
colcon build --packages-select second_project
source install/setup.bash
```

## Task 1 Mapping

```bash
ros2 launch second_project mapping.launch.py
```

The launch file starts:

- `pointcloud_to_laserscan`, converting `/ugv/rslidar_points` to `/base_scan`
- `slam_toolbox`
- RViz with map, laser scan, and TF displays

The generated map files are stored in `map/`.

## Task 2 Navigation

```bash
ros2 launch second_project navigation.launch.py
```

The launch file starts:

- Stage simulation using `world/scout_mini.world`
- Nav2 with `map/task1_map.yaml`
- RViz
- `goal_publisher_node`, which reads goals from `csv/goals.csv` and sends them
  to Nav2 one by one through the `/navigate_to_pose` action
