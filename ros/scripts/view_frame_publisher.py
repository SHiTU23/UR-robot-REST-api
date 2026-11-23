#!/usr/bin/env python3
"""(2026-11-21) RViz's fixed frame "room" follows the axes chosen on the web page (jog "axes": room | robot, saved by
PUT /cell/surfaces/axes in state/surfaces_axes.json): room -> world turned by robot.room_yaw_deg for "room", not turned
for "robot". Read once a second, so a change shows without restarting RViz. Display only: the driver, MoveIt and the
API never use the "room" frame."""
import json
import math
import os

import rclpy
import rclpy.executors
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

APP = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AXES_FILE = os.path.join(os.environ.get("UR_STATE_DIR") or os.path.join(APP, "state"), "surfaces_axes.json")
CELL = os.environ.get("UR_CELL_CONFIG") or os.path.join(APP, "config", "cell.yaml")


def axes() -> str:
    """as ur_api/routes_surfaces.axes(): room unless the file says robot"""
    try:
        with open(AXES_FILE) as f:
            a = json.load(f).get("axes")
    except (OSError, ValueError, AttributeError):
        a = None
    return a if a in ("room", "robot") else "room"


class ViewFrame(Node):
    def __init__(self):
        super().__init__("view_frame_publisher")
        with open(CELL) as f:
            self.room_yaw = math.radians(float((yaml.safe_load(f).get("robot") or {}).get("room_yaw_deg", 0.0)))
        self.br = StaticTransformBroadcaster(self)
        self.sent = None
        self.create_timer(1.0, self.tick)
        self.tick()

    def tick(self):
        a = axes()
        if a == self.sent:
            return
        yaw = self.room_yaw if a == "room" else 0.0
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "room", "world"
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.br.sendTransform(t)
        self.sent = a
        self.get_logger().info(f"RViz axes: {a} (room -> world yaw {math.degrees(yaw):.1f} deg)")


def main():
    rclpy.init()
    try:
        rclpy.spin(ViewFrame())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
