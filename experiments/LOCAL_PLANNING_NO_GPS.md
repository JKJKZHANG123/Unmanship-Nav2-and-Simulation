# No-GPS Local Planning Experiment

## Purpose

Verify the real Unitree L1/IMU → Point-LIO → obstacle costmap → Nav2 planner chain while the vessel remains stationary. This experiment starts no controller, BT navigator, velocity smoother, GPS/RTK node, Gazebo node, or actuator bridge.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select usv_navigation usv_cloud_filter
source install/setup.bash
```

## Run

Terminal 1 starts the real L1 and Point-LIO:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_mapping_launcher mapping_no_gps.launch.py rviz:=false
```

Keep the vessel stationary for 20–30 seconds while the IMU/LIO initializes.

Terminal 2 starts only obstacle filtering, the planner server, its rolling `camera_init` costmap, and RViz:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch usv_navigation local_planning_experiment.launch.py rviz:=true
```

Check readiness:

```bash
ros2 topic hz /cloud_registered
ros2 topic hz /usv/raw_obstacle_cloud
ros2 topic hz /aft_mapped_to_init
ros2 lifecycle get /planner_server
ros2 action info /compute_path_to_pose
```

Choose a collision-free point in RViz, in the `camera_init` frame, preferably 10–20 m away. The launch enables the RViz point-click planner by default: select **Publish Point**, then click the target in the 3D view. The node sends only `ComputePathToPose`; it never sends velocity or thrust.

The clicked point is published on `/clicked_point`; the resulting path is published on `/plan`, and the target marker is shown on `/nav_waypoints_markers`.

For a repeatable numeric test, Terminal 3 can request one path directly without commanding motion:

```bash
ros2 run usv_navigation local_plan_once --ros-args \
  -p goal_x:=15.0 -p goal_y:=5.0 -p goal_yaw:=0.0 \
  -p hold_seconds:=30.0
```

## Verify Output

A successful click or numeric run prints the frame, pose count, path length, start/end coordinates, goal error, planning time, and error code. It publishes `nav_msgs/msg/Path` on `/plan` and writes `/tmp/usv_local_plan.json`.

```bash
ros2 topic echo /plan --once
python3 -m json.tool /tmp/usv_local_plan.json | head -80
ros2 topic info /cmd_vel -v
```

Pass criteria: error code is `NONE(0)`, `poses` is non-empty, `frame_id` is `camera_init`, the endpoint is near the selected goal, the RViz green path avoids occupied cells, and this launch creates no `/cmd_vel` publisher. Because GPS is intentionally absent, the path contains local meters (`x`, `y`, yaw), not latitude/longitude.
