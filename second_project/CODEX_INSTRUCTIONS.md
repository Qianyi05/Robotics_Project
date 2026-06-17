# CODEX_INSTRUCTIONS.md

## Project Context

This is a ROS 2 Humble robotics project using:

* ROS 2 Humble
* Stage simulator / stage_ros2
* Nav2
* AMCL
* RViz
* A custom waypoint or goal-sending node

The project is mainly developed and edited in VS Code, but the actual build, launch, and runtime testing are performed manually by the user in an Ubuntu terminal.

Codex should help edit code only. Codex should not assume it can run the robot successfully inside the IDE terminal.

---

## Very Important Rule

Do not run long-running robotics commands unless the user explicitly asks.

Do not run commands such as:

```bash
ros2 launch ...
ros2 run ...
rviz2
stage
ros2 topic echo ...
ros2 topic pub ...
ros2 action send_goal ...
```

The user will run these manually in the Ubuntu terminal.

Codex may suggest commands for the user to run, but should not execute them automatically.

---

## Allowed Codex Tasks

Codex may:

1. Read and inspect source files.
2. Modify Python, C++, YAML, launch, setup.py, package.xml, or config files when needed.
3. Add comments when they clarify logic.
4. Fix bugs in the waypoint sender, goal sender, launch files, or package installation.
5. Provide exact build and test commands for the user.
6. Explain every changed file and the reason for each change.

---

## Commands Codex May Use Carefully

Codex may use safe inspection commands, for example:

```bash
find .
ls
tree
cat
sed -n
grep -R
```

Codex may also suggest, but should not automatically run unless asked:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ...
ros2 topic list
ros2 action list
ros2 action info /navigate_to_pose
```

---

## Do Not Rewrite the Whole Project

The current project is already partially working.

Navigation mostly works. The main issue may be specific parts such as:

* waypoint transition logic
* goal completion detection
* Nav2 action result handling
* launch order
* frame_id or timestamp in PoseStamped
* package installation or executable entry point

Codex should always prefer the minimal robust fix.

Do not perform a large refactor unless the user explicitly asks.

---

## Files That Should Not Be Changed Unless Clearly Necessary

Do not modify these unless there is a clear reason:

* Stage world file
* map image
* map YAML origin or resolution
* AMCL frame names
* TF frame names
* robot footprint or robot_radius
* local costmap plugins
* global costmap plugins
* controller plugin
* planner plugin

If Codex thinks one of these must be changed, it must first explain:

1. What exact problem the change solves.
2. Why the current setting is wrong.
3. What risk the change may introduce.
4. How the user can verify the effect.

---

## ROS 2 Version

Use ROS 2 Humble by default.

When giving commands, use:

```bash
source /opt/ros/humble/setup.bash
```

Do not use ROS 2 Jazzy commands or APIs unless the user explicitly says so.

---

## Goal Sending Rules

If editing the waypoint or goal sender, prefer using the Nav2 action:

```text
/nav2_msgs/action/NavigateToPose
```

The node should use:

```text
/navigate_to_pose
```

Preferred behavior:

1. Wait for the action server.
2. Send only one goal at a time.
3. Use `PoseStamped`.
4. Use `header.frame_id = "map"`.
5. Use a valid timestamp from the node clock.
6. Convert yaw to quaternion correctly.
7. Use the action result callback to detect completion.
8. When the result status is `SUCCEEDED`, send the next waypoint.
9. If the goal is `ABORTED` or `CANCELED`, log the status clearly.
10. Avoid repeatedly publishing the same goal in a loop.

Do not rely only on manually checking Euclidean distance unless this is explicitly required.

---

## Frame Rules

Default frames:

```text
map
odom
base_link
```

Laser topic is usually:

```text
/base_scan
```

Do not rename frames or topics unless the existing project files prove that different names are required.

---

## Launch File Rules

When editing launch files:

1. Do not duplicate Nav2 lifecycle managers.
2. Do not launch duplicate map servers.
3. Do not launch duplicate AMCL nodes.
4. Do not create duplicate goal sender nodes.
5. Make sure the waypoint node starts after the required Nav2 components are available.
6. Prefer clear comments for launch order if needed.

---

## Nav2 Parameter Rules

Do not randomly tune Nav2 parameters.

Only change Nav2 parameters if the bug is clearly related to that parameter.

For example:

* If the robot reaches near the goal but Nav2 never returns success, goal checker tolerance may be relevant.
* If the robot oscillates near obstacles, controller or inflation settings may be relevant.
* If the robot jumps in RViz, AMCL, map origin, Stage pose, or TF may be relevant.

But Codex must explain the reason before changing these.

---

## Required Output After Every Edit

After modifying files, Codex must report:

1. Exact files changed.
2. What was changed in each file.
3. Why the change was necessary.
4. Whether any behavior changed.
5. Whether any parameter was changed.
6. Build command for the user.
7. Run command for the user.
8. Debug commands for the user.
9. Expected successful logs or behavior.

Example format:

```text
Changed files:
- second_project/waypoint_navigator.py
- second_project/launch/navigation.launch.py
- setup.py

Reason:
- Fixed waypoint transition by using NavigateToPose action result callback.
- Ensured next goal is sent only after SUCCEEDED.

User should run:
colcon build --symlink-install
source install/setup.bash
ros2 launch ...

Expected:
- Log: Sending goal 1
- Log: Goal 1 accepted
- Log: Goal 1 succeeded
- Log: Sending goal 2
```

---

## Debug Commands to Suggest to the User

Codex may suggest these commands when relevant:

```bash
ros2 action list
ros2 action info /navigate_to_pose
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /goal_pose --once
ros2 topic echo /base_scan --once
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 lifecycle nodes
```

But Codex should not run them automatically unless the user explicitly asks.

---

## Communication Style

When explaining changes, be direct and specific.

Avoid vague explanations such as:

```text
I improved navigation.
```

Prefer precise explanations such as:

```text
The old node published a goal but did not listen to the NavigateToPose action result. Therefore, it had no reliable way to know when Nav2 completed the goal. I changed it to wait for result.status == SUCCEEDED before sending the next waypoint.
```

---

## Main Principle

Make the smallest safe change that fixes the current problem.

Do not break currently working navigation.
