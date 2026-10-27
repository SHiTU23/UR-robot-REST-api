"""Planning and execution through move_group (the UR counterpart of the Franka API's franka.py).

MoveIt 2 has no moveit_commander, so this talks to move_group's own interfaces: /plan_kinematic_path
(Pilz PTP / LIN, OMPL), /compute_cartesian_path, /check_state_validity, and the /execute_trajectory action.
Safety rules kept from the Franka API:
  * targets and every waypoint of a plan must stay `joint_limit_margin_rad` inside the joint limits;
  * the big-swing guard rejects plans in which any joint but the tool roll travels more than
    max_joint_travel_deg (cumulative, so out-and-back counts);
  * pose goals for the sampling planner are turned into the NEAREST collision-free IK configuration first;
  * /control/stop cancels the running trajectory and suppresses the automatic retry.
UR stops map onto the Franka outcomes: a protective stop (collision detection, someone pushing the arm)
-> stopped_external_force, an emergency / safeguard stop -> stopped_user_button (only the hardware can clear it).
"""
import copy
import math
import threading
import time

import numpy as np

import geom
from errors import err

SUCCESS = 1
MOVEIT_ERRORS = {
    1: "SUCCESS", 99999: "FAILURE", -1: "PLANNING_FAILED: planner found no valid plan",
    -2: "INVALID_MOTION_PLAN: plan has issues (self-collision, joint limits)",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: scene changed during planning",
    -4: "CONTROL_FAILED: trajectory execution failed (protective stop? controller? program not running?)",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA", -6: "TIMED_OUT", -7: "PREEMPTED: another goal replaced this one",
    -10: "START_STATE_IN_COLLISION: robot already in collision",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS: current orientation outside constraint tolerance",
    -26: "START_STATE_INVALID",
    -12: "GOAL_IN_COLLISION: target pose collides with scene", -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED", -27: "GOAL_STATE_INVALID", -28: "UNRECOGNIZED_GOAL_TYPE",
    -15: "INVALID_GROUP_NAME", -16: "INVALID_GOAL_CONSTRAINTS", -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME", -19: "INVALID_OBJECT_NAME", -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE", -23: "ROBOT_STATE_STALE", -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE", -29: "CRASH", -30: "ABORT",
    -31: "NO_IK_SOLUTION: target is reachable in position but orientation can't be achieved",
}
PLANNERS = {"RRTConnect": "ompl", "PTP": "pilz_industrial_motion_planner", "LIN": "pilz_industrial_motion_planner"}

# ur_dashboard_msgs SafetyMode
PROTECTIVE_STOP = 3
USER_STOPS = {5: "SAFEGUARD_STOP", 6: "SYSTEM_EMERGENCY_STOP", 7: "ROBOT_EMERGENCY_STOP",
              12: "AUTOMATIC_MODE_SAFEGUARD_STOP", 13: "SYSTEM_THREE_POSITION_ENABLING_STOP"}
SAFETY_NAMES = {1: "NORMAL", 2: "REDUCED", 3: "PROTECTIVE_STOP", 4: "RECOVERY", 5: "SAFEGUARD_STOP",
                6: "SYSTEM_EMERGENCY_STOP", 7: "ROBOT_EMERGENCY_STOP", 8: "VIOLATION", 9: "FAULT",
                10: "VALIDATE_JOINT_ID", 11: "UNDEFINED_SAFETY_MODE", 12: "AUTOMATIC_MODE_SAFEGUARD_STOP",
                13: "SYSTEM_THREE_POSITION_ENABLING_STOP"}
ROBOT_MODE_NAMES = {-1: "NO_CONTROLLER", 0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING", 3: "POWER_OFF",
                    4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE", 7: "RUNNING", 8: "UPDATING_FIRMWARE"}
SAFETY_CODES = {v: k for k, v in SAFETY_NAMES.items()}
ROBOT_MODE_CODES = {v: k for k, v in ROBOT_MODE_NAMES.items()}
PROTECTIVE_UNLOCK_WAIT_S = 5.0          # the controller refuses an unlock in the first 5 s of a protective stop


def decode_error(code):
    return MOVEIT_ERRORS.get(getattr(code, "val", code), f"UNKNOWN_ERROR_{getattr(code, 'val', code)}")


def _dur(d):
    return d.sec + d.nanosec * 1e-9


def traj_points(traj, names):
    """[(t, [q in ``names`` order])] of a moveit_msgs/RobotTrajectory."""
    jt = traj.joint_trajectory
    idx = [list(jt.joint_names).index(n) for n in names]
    return [(_dur(p.time_from_start), [p.positions[i] for i in idx]) for p in jt.points]


def joint_travel(traj):
    """{joint: cumulative |dq|} over a RobotTrajectory (out-and-back counts)."""
    jt = traj.joint_trajectory
    out, prev = {}, None
    for p in jt.points:
        if prev is not None:
            for n, a, b in zip(jt.joint_names, prev, p.positions):
                out[n] = out.get(n, 0.0) + abs(b - a)
        prev = p.positions
    return out


class Motion:
    def __init__(self, ros, cell, kin, dashboard=None, mock=False):
        self.ros, self.cell, self.kin, self.dash, self.mock = ros, cell, kin, dashboard, mock
        self._stop = threading.Event()
        self._goal_lock = threading.Lock()
        self._goals = []                         # accepted ExecuteTrajectory goal handles
        self.protective_since = None
        self.abort_reason = None

    # ---- states ----

    def robot_state(self, q=None, width=None, extra_attached=None):
        """A diff RobotState (keeps the attached bodies: a full state would drop the tool add-ons and a held
        part from every check) with optional arm joints and gripper width."""
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        st = RobotState(is_diff=True)
        names, pos = [], []                      # (JointState.position is an array.array: build lists first)
        if q is not None:
            names += list(self.cell.arm_joints)
            pos += [float(v) for v in q]
        if width is not None:
            for n, v in self.cell.finger_q(float(width)).items():
                names.append(n)
                pos.append(v)
        st.joint_state = JointState(name=names, position=pos)
        if extra_attached:
            st.attached_collision_objects = list(extra_attached)
        return st

    def current_q(self):
        q = self.ros.arm_q()
        if q is None:
            raise err(503, "no_joint_state", "no /joint_states for the arm received yet")
        return q

    def check_state(self, q=None, width=None, state=None):
        """(valid, [{a, b, depth_m, point}]) of a configuration against the current planning scene."""
        from moveit_msgs.srv import GetStateValidity
        req = GetStateValidity.Request(robot_state=state or self.robot_state(q, width), group_name=self.cell.group)
        res = self.ros.call(GetStateValidity, "/check_state_validity", req, timeout_s=5.0)
        contacts = [{"a": c.contact_body_1, "b": c.contact_body_2, "depth_m": c.depth,
                     "point": [c.position.x, c.position.y, c.position.z]} for c in res.contacts]
        return bool(res.valid), contacts

    def contact_pairs(self, contacts):
        return sorted({" <-> ".join(sorted((c["a"], c["b"]))) for c in contacts})

    # ---- goals ----

    def joint_goal(self, q):
        from moveit_msgs.msg import Constraints, JointConstraint
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=n, position=float(v), tolerance_above=1e-4,
                                               tolerance_below=1e-4, weight=1.0) for n, v in zip(self.cell.arm_joints, q)]
        return c

    def pose_goal(self, pose, link=None):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import BoundingVolume, Constraints, OrientationConstraint, PositionConstraint
        from shape_msgs.msg import SolidPrimitive
        link = link or self.cell.tcp_link
        c = Constraints()
        pc = PositionConstraint(link_name=link, weight=1.0)
        pc.header.frame_id = self.cell.planning_frame
        bv = BoundingVolume()
        bv.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-4])]
        p = Pose()
        p.position.x, p.position.y, p.position.z = (float(v) for v in pose["position"])
        p.orientation.w = 1.0
        bv.primitive_poses = [p]
        pc.constraint_region = bv
        oc = OrientationConstraint(link_name=link, weight=1.0, absolute_x_axis_tolerance=1e-3,
                                   absolute_y_axis_tolerance=1e-3, absolute_z_axis_tolerance=1e-3)
        oc.header.frame_id = self.cell.planning_frame
        oc.orientation = pose_msg(pose).orientation
        c.position_constraints, c.orientation_constraints = [pc], [oc]
        return c

    def orientation_path_constraint(self, quat, tol=0.8):
        """Keep the TCP within ``tol`` rad of ``quat`` on every axis along the path (no wrist flips)."""
        from moveit_msgs.msg import Constraints, OrientationConstraint
        oc = OrientationConstraint(link_name=self.cell.tcp_link, weight=1.0, absolute_x_axis_tolerance=tol,
                                   absolute_y_axis_tolerance=tol, absolute_z_axis_tolerance=tol)
        oc.header.frame_id = self.cell.planning_frame
        oc.orientation = pose_msg({"position": [0, 0, 0], "orientation": list(quat)}).orientation
        c = Constraints()
        c.orientation_constraints = [oc]
        return c

    # ---- planning ----

    def plan(self, goal, pipeline, planner, vel, acc, start_state=None, attempts=1, time_s=5.0, path=None):
        """One /plan_kinematic_path call. -> (ok, RobotTrajectory, planning_time_s, error_code)."""
        from moveit_msgs.msg import MotionPlanRequest
        from moveit_msgs.srv import GetMotionPlan
        r = MotionPlanRequest()
        r.group_name = self.cell.group
        r.pipeline_id, r.planner_id = pipeline, planner
        r.start_state = start_state or self.robot_state()
        r.goal_constraints = [goal]
        if path is not None:
            r.path_constraints = path
        r.num_planning_attempts = int(attempts)
        r.allowed_planning_time = float(time_s)
        r.max_velocity_scaling_factor = float(vel)
        r.max_acceleration_scaling_factor = float(acc)
        res = self.ros.call(GetMotionPlan, "/plan_kinematic_path", GetMotionPlan.Request(motion_plan_request=r),
                            timeout_s=time_s + 15.0).motion_plan_response
        return res.error_code.val == SUCCESS, res.trajectory, res.planning_time, res.error_code.val

    def plan_joint_goal(self, target, vel=0.2, acc=0.1, start_state=None):
        """Pilz PTP first (straight in joint space, every joint monotonic: no detours), OMPL RRTConnect as the
        fallback (typically when the straight joint path collides).
        -> (ok, traj, planning_time, error_code, planner_used)."""
        goal = self.joint_goal(target)
        last = (False, None, 0.0, None, None)
        for pipeline, planner, attempts in (("pilz_industrial_motion_planner", "PTP", 1), ("ompl", "RRTConnect", 10)):
            ok, traj, t, code = self.plan(goal, pipeline, planner, vel, acc, start_state, attempts)
            if ok:
                return True, traj, t, code, planner
            self.ros.node.get_logger().warning(f"joint goal: {pipeline}/{planner} failed ({decode_error(code)})")
            last = (False, traj, t, code, None)
        return last

    def ik_near(self, pose, start_q, start_state=None, max_checks=25):
        """The collision-free IK configuration nearest to ``start_q`` (weighted: base/elbow expensive, tool
        roll cheap). -> (q or None, number of analytic solutions)."""
        sols = self.kin.solutions(geom.pose_to_T(pose), start_q)
        for _cost, q in sols[:max_checks]:
            st = copy.deepcopy(start_state) if start_state is not None else self.robot_state()
            st.joint_state.name = list(st.joint_state.name)
            st.joint_state.position = list(st.joint_state.position)
            for n, v in zip(self.cell.arm_joints, q):
                if n in st.joint_state.name:
                    st.joint_state.position[st.joint_state.name.index(n)] = v
                else:
                    st.joint_state.name.append(n)
                    st.joint_state.position.append(v)
            if self.check_state(state=st)[0]:
                return q, len(sols)
        return None, len(sols)

    def plan_pose(self, pose, planner="LIN", preserve_orientation=True, vel=0.3, acc=0.05):
        """Plan the TCP to ``pose`` (base_link). LIN: Pilz straight line. RRTConnect: nearest IK -> joint goal,
        3 plans, the one with the least joint travel. -> (ok, traj, planning_time, error_code, meta)."""
        path = self.orientation_path_constraint(pose["orientation"]) if preserve_orientation else None
        if planner == "LIN":
            ok, traj, t, code = self.plan(self.pose_goal(pose), PLANNERS["LIN"], "LIN", vel, acc)
            return ok, traj, t, code, {"ik_seeded": False}
        q0 = self.current_q()
        q, n = self.ik_near(pose, q0)
        goal = self.joint_goal(q) if q is not None else self.pose_goal(pose)
        best = None
        for _ in range(3):
            ok, traj, t, code = self.plan(goal, "ompl", planner, vel, acc, attempts=10, path=path)
            cost = max(joint_travel(traj).values() or [0.0]) if ok else float("inf")
            if best is None or (ok and cost < best[4]):
                best = (ok, traj, t, code, cost)
        ok, traj, t, code, _ = best
        return ok, traj, t, code, {"ik_seeded": q is not None, "ik_solutions": n}

    def plan_cartesian(self, waypoints, vel=0.05, acc=0.05, start_state=None, eef_step=0.01, link=None):
        """Straight TCP line(s) through ``waypoints`` (pose dicts). -> (traj, fraction, error_code)."""
        from moveit_msgs.srv import GetCartesianPath
        r = GetCartesianPath.Request()
        r.header.frame_id = self.cell.planning_frame
        r.start_state = start_state or self.robot_state()
        r.group_name = self.cell.group
        r.link_name = link or self.cell.tcp_link
        r.waypoints = [pose_msg(w) for w in waypoints]
        r.max_step = float(eef_step)
        r.jump_threshold = 0.0
        r.avoid_collisions = True
        r.max_velocity_scaling_factor = float(vel)
        r.max_acceleration_scaling_factor = float(acc)
        res = self.ros.call(GetCartesianPath, "/compute_cartesian_path", r, timeout_s=20.0)
        return res.solution, res.fraction, res.error_code.val

    # ---- checks ----

    def traj_limit_violations(self, traj):
        out = []
        for i, (_, q) in enumerate(traj_points(traj, self.cell.arm_joints)):
            out += [{"waypoint_index": i, **b} for b in self.cell.near_limits(q)]
        return out

    def too_far(self, traj, limit_rad):
        travel = joint_travel(traj)
        return travel, {j: round(t, 3) for j, t in travel.items() if limit_rad and t > limit_rad
                        and j not in self.cell.travel_exempt}

    def display(self, traj):
        from moveit_msgs.msg import DisplayTrajectory
        try:
            d = DisplayTrajectory()
            d.trajectory = [traj]
            self.ros.display_pub.publish(d)
        except Exception:      # noqa: BLE001
            pass

    # ---- execution ----

    def execute(self, traj, timeout_s=None):
        """Run a trajectory through move_group. -> (ok, error_code).

        On a real controller a watcher polls the Dashboard (5 Hz) while the arm moves: on a protective /
        emergency / safeguard stop, or when the driver's program stops, it ends the execution at once. Without
        it the request would hang: the scaled trajectory controller does not abort on a protective stop, it
        scales time to zero and waits (seen on URSim: 34 s until the timeout, and move_group then still held the
        frozen goal and ignored the next one)."""
        from moveit_msgs.action import ExecuteTrajectory
        pts = traj.joint_trajectory.points
        dur = _dur(pts[-1].time_from_start) if pts else 0.0
        timeout_s = timeout_s or max(30.0, 3.0 * dur + 10.0)
        holder, done = {}, threading.Event()
        self.abort_reason = None

        def keep(h):
            holder["h"] = h
            with self._goal_lock:
                self._goals.append(h)
            if self._stop.is_set():
                self.ros.stop_execution()

        def watch():
            while not done.wait(0.2):
                try:
                    s, running = self.safety_mode(), self.dash.running()
                except Exception:      # noqa: BLE001
                    continue
                if s not in (1, 2) or not running:          # NORMAL, REDUCED
                    self.abort_reason = (f"safety mode {SAFETY_NAMES.get(s, s)}" if s not in (1, 2)
                                         else "the driver's program stopped running")
                    self.ros.stop_execution()
                    return

        if self.dash is not None:
            threading.Thread(target=watch, name="exec-watch", daemon=True).start()
        try:
            accepted, result, _status = self.ros.run_action(ExecuteTrajectory, "/execute_trajectory",
                                                            ExecuteTrajectory.Goal(trajectory=traj), timeout_s, keep)
        except Exception:
            self.ros.stop_execution()                 # never leave move_group holding a goal
            raise
        finally:
            done.set()
            with self._goal_lock:
                if "h" in holder and holder["h"] in self._goals:
                    self._goals.remove(holder["h"])
        if not accepted or result is None:
            self.ros.stop_execution()
            return False, None
        return result.error_code.val == SUCCESS, result.error_code.val

    def stop(self):
        """Cancel every running trajectory; the in-flight request answers 'stopped' and is not retried."""
        self._stop.set()
        self.ros.stop_execution()                # what actually halts the controller (see ros_io)
        with self._goal_lock:
            goals = list(self._goals)
        for h in goals:
            try:
                h.cancel_goal_async()
            except Exception:      # noqa: BLE001
                pass
        return len(goals)

    def clear_stop(self):
        self._stop.clear()

    def stop_requested(self):
        return self._stop.is_set()

    def safety_mode(self):
        """ur_dashboard_msgs SafetyMode value. The controller's Dashboard is authoritative: the driver's latched
        /io_and_status_controller/safety_mode topic is published only on change, through a publisher that may skip
        a message, and was seen stuck at PROTECTIVE_STOP while the controller said NORMAL (URSim, 2026-10-26).
        The topic is only the fallback (mock hardware has no Dashboard)."""
        if self.dash is not None:
            try:
                name = self.dash.safety_status().upper()          # "NORMAL", "PROTECTIVE_STOP", ...
                return SAFETY_CODES.get(name, 11)
            except Exception:      # noqa: BLE001
                pass
        return self.ros.safety_mode

    def robot_mode(self):
        if self.dash is not None:
            try:
                return ROBOT_MODE_CODES.get(self.dash.robot_mode().upper(), -1)
            except Exception:      # noqa: BLE001
                pass
        return self.ros.robot_mode

    def program_running(self):
        """Is the driver's control script running? The Dashboard again is authoritative: the latched topic was
        seen saying True after a protective stop had stopped the script."""
        if self.dash is not None:
            try:
                return self.dash.running()
            except Exception:      # noqa: BLE001
                pass
        return bool(self.ros.program_running)

    def classify_stop(self):
        """('stopped_external_force' | 'stopped_user_button' | None, reason)."""
        s = self.safety_mode()
        if s == PROTECTIVE_STOP:
            return "stopped_external_force", "UR protective stop (collision detection: a push or contact)"
        if s in USER_STOPS:
            return "stopped_user_button", f"UR {USER_STOPS[s]} active"
        return None, None

    def recover(self, timeout_s=15.0):
        """Clear what software may clear and give control back to the driver: a protective stop is unlocked
        (the controller refuses that within its first 5 s), safety popups are closed, then the driver's
        program is resent when it does not run. Emergency / safeguard stops and a powered-off arm are left to
        a person (power-on + brake release move the joints). -> (ok, reason)."""
        if self.mock:
            return True, None
        t0 = time.time()
        self.ros.stop_execution()                     # a frozen goal must not stay in move_group
        try:
            if self.safety_mode() == PROTECTIVE_STOP and self.dash is not None:
                while time.time() - t0 < timeout_s:
                    ans = self.dash.unlock_protective_stop()       # "Protective stop releasing" on success,
                    if "releasing" in ans.lower():                 # "Cannot unlock ... until 5s ..." before that
                        break
                    time.sleep(1.0)
                self.dash.close_safety_popup()
                while self.safety_mode() == PROTECTIVE_STOP and time.time() - t0 < timeout_s:
                    time.sleep(0.2)
            s = self.safety_mode()
            if s in USER_STOPS:
                return False, f"{USER_STOPS[s]} is active: release it on the hardware (and confirm on the pendant)"
            m = self.robot_mode()
            if m is not None and m != 7:
                return False, (f"the arm is {ROBOT_MODE_NAMES.get(m, m)}, not RUNNING: power it on and release the "
                               "brakes on the pendant (or the dashboard: power on, brake release) -- not done remotely")
            if not self.program_running():
                from std_srvs.srv import Trigger
                r = self.ros.call(Trigger, "/io_and_status_controller/resend_robot_program", Trigger.Request(),
                                  timeout_s=10)
                while not self.program_running() and time.time() - t0 < timeout_s:
                    time.sleep(0.2)
                if not self.program_running():
                    return False, f"the driver's program does not run after resend_robot_program ({r.message})"
            return True, None
        except Exception as e:      # noqa: BLE001
            self.ros.node.get_logger().error(f"recover failed: {e}")
            return False, f"{type(e).__name__}: {getattr(e, 'msg', e)}"

    def plan_and_execute(self, plan_fn, auto_recover=True, max_retries=1, max_travel_rad=None, err_baseline=None):
        """Plan, check, execute, and on an execution failure recover + replan + retry. Same outcomes as the
        Franka API: success | plan_failed | plan_would_violate_limits | plan_too_large | execute_failed |
        recovery_failed | stopped | stopped_external_force | stopped_user_button."""
        limit = self.cell.max_travel_rad if max_travel_rad is None else max_travel_rad
        rec_trig = rec_ok = False
        attempts, meta = 0, {}
        self._stop.clear()

        def out(outcome, **kw):
            return {"outcome": outcome, "recovery_triggered": rec_trig, "recovery_succeeded": rec_ok,
                    "attempts": attempts, "plan_metadata": meta, **kw}

        while attempts <= max_retries:
            attempts += 1
            kind, why = self.classify_stop()
            if kind == "stopped_user_button":
                return out(kind, stop_reason=why)
            traj, ok, meta = plan_fn()
            if not ok:
                kind, why = self.classify_stop()
                return out(kind, stop_reason=why) if kind else out("plan_failed")
            self.display(traj)
            bad = self.traj_limit_violations(traj)
            if bad:
                return out("plan_would_violate_limits", trajectory_violations=bad)
            travel, far = self.too_far(traj, limit)
            if far:
                return out("plan_too_large", joint_travel=travel, max_joint_travel_rad=limit)
            if self._stop.is_set():
                return out("stopped", stop_reason="user /control/stop")
            ok, code = self.execute(traj)
            if self._stop.is_set():
                return out("stopped", stop_reason="user /control/stop")
            if ok:
                return out("success")
            meta["execute_error"] = decode_error(code) if code is not None else "no result"
            if self.abort_reason:
                meta["aborted_because"] = self.abort_reason
            time.sleep(0.3)                   # let the safety-mode topic catch up
            kind, why = self.classify_stop()
            if kind:
                return out(kind, stop_reason=why)
            if not auto_recover or attempts > max_retries:
                return out("execute_failed")
            rec_trig = True
            rec_ok, why = self.recover()
            if not rec_ok:
                return out("recovery_failed", recovery_reason=why)
        return out("execute_failed")

    def move_joints(self, q, vel=0.1, acc=0.05):
        """Plan (PTP, then RRTConnect) and execute to joints q; for multi-pose routines and home.
        -> {executed, planner_used, planning_time, plan_error_codes, stopped?}."""
        if self._stop.is_set():
            return {"executed": False, "stopped": True, "joint_goal": list(q), "plan_error_codes": []}
        ok, traj, t, code, planner = self.plan_joint_goal(q, vel, acc)
        res = {"executed": False, "joint_goal": [round(v, 5) for v in q], "planner_used": planner,
               "planning_time": t, "plan_error_codes": [decode_error(code)] if code is not None else []}
        if ok:
            self.display(traj)
            res["executed"] = self.execute(traj)[0]
        return res


def pose_msg(p):
    from geometry_msgs.msg import Pose
    m = Pose()
    m.position.x, m.position.y, m.position.z = (float(v) for v in p["position"])
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = (float(v) for v in p["orientation"])
    return m


def pose_dict(m):
    return {"position": [m.position.x, m.position.y, m.position.z],
            "orientation": [m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w]}
