#!/bin/bash
# RViz, with the robot turned to match how it stands in the room (robot.room_yaw_deg in config/cell.yaml).
# Display only: the "room" frame exists for RViz alone, so the driver, MoveIt and the API do not see it.
YAW=$(python3 -c 'import math, os, yaml; print(math.radians(yaml.safe_load(open(os.environ.get("UR_CELL_CONFIG") or "/app/config/cell.yaml"))["robot"].get("room_yaw_deg", 0)))')
ros2 run tf2_ros static_transform_publisher --yaw "$YAW" --frame-id room --child-frame-id world &
exec ros2 run rviz2 rviz2 -d /app/ros/rviz/cell.rviz
