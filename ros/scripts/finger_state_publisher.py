#!/usr/bin/env python3
"""Publishes the gripper's driving finger joint on /joint_states, so robot_state_publisher and MoveIt see a
complete robot state: the gripper is not a ros2_control device. The other finger joints mimic it.

The width comes from /gripper/width (std_msgs/Float64, metres, transient local), which the REST API publishes
after every gripper action, and becomes a joint value through the model's table (config/grippers/<model>.yaml:
rows [joint, width, tip depth], computed from the model). Until the first message the fingers are shown fully
open (initial_width_m): the widest envelope, i.e. the cautious choice for collision checks near walls."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class FingerStatePublisher(Node):
    def __init__(self):
        super().__init__("finger_state_publisher")
        self.joint = self.declare_parameter("joint", "rg6_finger_joint").value
        tj = list(self.declare_parameter("table_joint", [0.0]).value)
        tw = list(self.declare_parameter("table_width", [0.0]).value)
        order = np.argsort(tw)                       # np.interp needs increasing widths
        self.tw, self.tj = np.asarray(tw)[order], np.asarray(tj)[order]
        self.max_width = float(self.declare_parameter("max_width_m", 0.160).value)
        self.width = float(self.declare_parameter("initial_width_m", self.max_width).value)
        rate = float(self.declare_parameter("rate_hz", 30.0).value)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(Float64, "/gripper/width", self.on_width,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0 / rate, self.tick)

    def on_width(self, msg):
        self.width = min(max(float(msg.data), 0.0), self.max_width)

    def tick(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = [self.joint]
        m.position = [float(np.interp(self.width, self.tw, self.tj))]
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(FingerStatePublisher())


if __name__ == "__main__":
    main()
