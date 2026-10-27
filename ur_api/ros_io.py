"""The ROS 2 side of the API: one rclpy node spun by a background executor.

Flask request threads never spin. They call services and actions through
``call()`` / ``send_goal()``, which hand the request to the executor and wait
on the future with a timeout. Subscriptions keep the latest joint state,
wrench, robot/safety mode and program state in small thread-safe caches, so
the read endpoints (/joints, /state, /force, /health) answer without any
ROS round trip.
"""
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from errors import err


def _latched(depth=1):
    return QoSProfile(depth=depth, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class RosIO:
    """Owns the node, the executor thread and the state caches."""

    def __init__(self, cell, node_name="ur_rest_api"):
        from geometry_msgs.msg import WrenchStamped
        from moveit_msgs.msg import DisplayTrajectory
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64, String
        from ur_dashboard_msgs.msg import RobotMode, SafetyMode
        from visualization_msgs.msg import MarkerArray

        self.cell = cell
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self.joints = None              # {"name": [..], "position": [..], "velocity": .., "effort": .., "stamp", "rx"}
        self._merged = {}               # name -> (pos, vel, eff, rx)
        self.wrench = None              # (fx, fy, fz, tx, ty, tz, frame_id, stamp_s, rx)
        self.robot_mode = None          # ur_dashboard_msgs/RobotMode value
        self.safety_mode = None         # ur_dashboard_msgs/SafetyMode value
        self.program_running = None
        self.urdf = None
        self.srdf = None
        self.rosout_errors = []         # last ERROR/FATAL lines of /rosout (for last_error fields)
        self._clients = {}
        self._actions = {}

        n, cb = self.node, self.cb
        n.create_subscription(JointState, "/joint_states", self._on_joints, 50, callback_group=cb)
        n.create_subscription(WrenchStamped, cell.wrench_topic, self._on_wrench, 10, callback_group=cb)
        # latched (TRANSIENT_LOCAL) and published only on change: a volatile subscription never sees a value
        n.create_subscription(RobotMode, "/io_and_status_controller/robot_mode", self._on_robot_mode, _latched(),
                              callback_group=cb)
        n.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode", self._on_safety_mode, _latched(),
                              callback_group=cb)
        n.create_subscription(Bool, "/io_and_status_controller/robot_program_running", self._on_program, _latched(),
                              callback_group=cb)
        n.create_subscription(String, "/robot_description", self._on_urdf, _latched(), callback_group=cb)
        n.create_subscription(String, "/robot_description_semantic", self._on_srdf, _latched(), callback_group=cb)
        from rcl_interfaces.msg import Log
        n.create_subscription(Log, "/rosout", self._on_rosout, 100, callback_group=cb)
        self.width_pub = n.create_publisher(Float64, "/gripper/width", _latched())
        self.display_pub = n.create_publisher(DisplayTrajectory, "/display_planned_path", 5)
        self.marker_pub = n.create_publisher(MarkerArray, "/ur_ee_force", 2)
        # MoveGroupInterface::stop() publishes "stop" here; the trajectory execution manager then cancels the
        # controller goal. (Cancelling the ExecuteTrajectory action goal alone does NOT stop the arm in MoveIt 2.)
        self.exec_event_pub = n.create_publisher(String, "/trajectory_execution_event", 10)

        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, name="rclpy-executor", daemon=True).start()

    # ---- subscriptions ----

    def _on_joints(self, msg):
        rx = time.time()
        with self._lock:
            for i, name in enumerate(msg.name):
                self._merged[name] = (msg.position[i] if i < len(msg.position) else None,
                                      msg.velocity[i] if i < len(msg.velocity) else None,
                                      msg.effort[i] if i < len(msg.effort) else None, rx,
                                      msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _on_wrench(self, msg):
        w = msg.wrench
        with self._lock:
            self.wrench = (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z,
                           msg.header.frame_id or self.cell.wrench_frame,
                           msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, time.time())

    def _on_robot_mode(self, msg):
        self.robot_mode = msg.mode

    def _on_safety_mode(self, msg):
        self.safety_mode = msg.mode

    def _on_program(self, msg):
        self.program_running = bool(msg.data)

    def _on_urdf(self, msg):
        self.urdf = msg.data

    def _on_srdf(self, msg):
        self.srdf = msg.data

    # harmless start-up messages that would otherwise show as the "last error"
    BENIGN = ("No 3D sensor plugin(s) defined for octomap updates",)

    def _on_rosout(self, msg):
        if msg.level >= 40 and msg.name != self.node.get_name() and not any(b in msg.msg for b in self.BENIGN):
            line = f"[{msg.stamp.sec}] {msg.name}: {msg.msg}"
            with self._lock:
                self.rosout_errors.append(line)
                del self.rosout_errors[:-50]

    # ---- reads ----

    def joint_state(self, names):
        """{name: (pos, vel, eff, rx, stamp)} for ``names``; None if any is missing."""
        with self._lock:
            if not all(n in self._merged for n in names):
                return None
            return {n: self._merged[n] for n in names}

    def arm_q(self):
        js = self.joint_state(self.cell.arm_joints)
        return None if js is None else [js[n][0] for n in self.cell.arm_joints]

    def all_positions(self):
        with self._lock:
            return {n: v[0] for n, v in self._merged.items()}

    def last_error(self, since=None):
        with self._lock:
            if since is None:
                return self.rosout_errors[-1] if self.rosout_errors else None
            return self.rosout_errors[-1] if len(self.rosout_errors) > since else None

    def error_count(self):
        with self._lock:
            return len(self.rosout_errors)

    def wait_for_description(self, timeout_s=30.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.urdf and self.srdf:
                return True
            time.sleep(0.1)
        return bool(self.urdf)

    # ---- services / actions ----

    def client(self, srv_type, name):
        key = (srv_type, name)
        if key not in self._clients:
            self._clients[key] = self.node.create_client(srv_type, name, callback_group=self.cb)
        return self._clients[key]

    def call(self, srv_type, name, request, timeout_s=10.0, wait_s=3.0):
        """Synchronous service call from a non-executor thread."""
        c = self.client(srv_type, name)
        if not c.wait_for_service(timeout_sec=wait_s):
            raise err(503, "service_unavailable", f"{name} is not available (is move_group / the driver running?)")
        fut = c.call_async(request)
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_s):
            fut.cancel()
            raise err(504, "service_timeout", f"{name} did not answer within {timeout_s} s")
        if fut.exception() is not None:
            raise err(500, "service_failed", f"{name}: {fut.exception()}")
        return fut.result()

    def service_ready(self, srv_type, name, wait_s=0.1):
        return self.client(srv_type, name).wait_for_service(timeout_sec=wait_s)

    def action(self, action_type, name):
        key = (action_type, name)
        if key not in self._actions:
            self._actions[key] = ActionClient(self.node, action_type, name, callback_group=self.cb)
        return self._actions[key]

    def run_action(self, action_type, name, goal, timeout_s, on_handle=None, wait_s=3.0):
        """Send a goal and wait for its result. ``on_handle(handle)`` receives the
        goal handle as soon as it is accepted (so another thread can cancel it).
        -> (accepted, result_or_None, status)."""
        ac = self.action(action_type, name)
        if not ac.wait_for_server(timeout_sec=wait_s):
            raise err(503, "action_unavailable", f"{name} action server is not available")
        ev, box = threading.Event(), {}
        send = ac.send_goal_async(goal)
        send.add_done_callback(lambda f: (box.__setitem__("h", f.result()), ev.set()))
        if not ev.wait(10.0):
            raise err(504, "action_timeout", f"{name}: goal not acknowledged within 10 s")
        h = box["h"]
        if not h.accepted:
            return False, None, None
        if on_handle:
            on_handle(h)
        ev2 = threading.Event()
        res = h.get_result_async()
        res.add_done_callback(lambda _f: ev2.set())
        if not ev2.wait(timeout_s):
            h.cancel_goal_async()
            ev2.wait(5.0)
            if not res.done():
                return True, None, None
        r = res.result()
        return True, r.result, r.status

    def stop_execution(self):
        from std_msgs.msg import String
        self.exec_event_pub.publish(String(data="stop"))

    def publish_width(self, width_m):
        from std_msgs.msg import Float64
        self.width_pub.publish(Float64(data=float(width_m)))

    def shutdown(self):
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
