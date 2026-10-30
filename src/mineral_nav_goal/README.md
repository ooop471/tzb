# Mineral navigation goal

This ROS 2 node combines `/yolo/detections` with aligned `/camera/depth/color/points`.
After three detections of the configured class, it samples the depth at the box
center, transforms the target through `base_link` into `map`, and publishes a
candidate Nav2 parking pose at `/mineral_nav_goal/goal_pose` (`PoseStamped`).
The target itself is published at `/mineral_nav_goal/target_point` (`PointStamped`).

The parking pose faces the target and puts its center 0.30 m ahead of `base_link`.
The allowed standoff interval is 0.20–0.40 m. A 0.30 m front wheel inner gap
with 0.03 m clearance on each side accepts detected objects up to 0.24 m wide.
These values are parameters, not measured calibration. Check the actual wheel
clearance and the `base_link` origin on the rover before navigation.

The node refuses to publish when the detection, depth, or required TF is missing
or stale. In particular, the active navigation stack must provide
`map -> base_link`; one nav_ws configuration uses `base_frame` instead. Those
frame names are not interchangeable without a verified transform.

`send_nav_goal` defaults to `false`: the node publishes a candidate goal but
does not move the rover. After visual inspection of candidate poses and Nav2
costmap/path checks, set `send_nav_goal: true` to send one
`nav2_msgs/action/NavigateToPose` goal. Restart the node to request another goal.
The existing `navigation_goal_sequence` uses fixed coordinates and does not
subscribe to this candidate topic.

Start the node separately, without rebuilding or changing other workspace folders:

```bash
cd ~/tzb_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=$PWD/src/mineral_nav_goal:$PYTHONPATH \
  /usr/bin/python3 src/mineral_nav_goal/mineral_nav_goal/node.py \
  --ros-args --params-file $PWD/src/mineral_nav_goal/config/params.yaml
```

For normal ROS package use later, build only this package with
`colcon build --packages-select mineral_nav_goal`, source `install/setup.bash`,
then run `ros2 launch mineral_nav_goal mineral_nav_goal.launch.py`.

Inspect output:

```bash
ros2 topic echo /mineral_nav_goal/goal_pose
ros2 topic echo /mineral_nav_goal/target_point
ros2 run tf2_ros tf2_echo map base_link
```

Change `target_class` in `config/params.yaml` when the YOLO model changes.
Transparent objects can produce background depth; inspect the target point
against the physical object before enabling Nav2 action submission.
