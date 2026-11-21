#!/bin/bash
# RViz, drawn along the axes chosen on the web page (room | robot, state/surfaces_axes.json): view_frame_publisher.py
# turns the "room" frame by robot.room_yaw_deg (config/cell.yaml) for "room", not at all for "robot", and follows a change.
# Display only: the "room" frame exists for RViz alone, so the driver, MoveIt and the API do not see it.
python3 /app/ros/scripts/view_frame_publisher.py &
exec ros2 run rviz2 rviz2 -d /app/ros/rviz/cell.rviz
