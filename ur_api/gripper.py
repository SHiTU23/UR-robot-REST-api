"""The gripper. OnRobot RG6 on the UR, driven like ~/sota/ui/robot.py drives it on the real arm:

  1. write the target width (mm) and force (N) into RTDE input registers (rtde_regs: a session that claims
     only those registers -- ur_rtde's RTDEIOInterface also claims fields the ROS driver holds and is refused);
  2. stop whatever runs on the controller -- with the ROS driver that is its External Control script --
     and wait until the dashboard really reports nothing running (a play while the old script is still
     winding down is refused, and a "running" poll would then mistake the dying script for the program);
  3. load + play gripper_open.urp / gripper_close.urp (pendant programs that read those registers in a
     Script node); play is retried while PolyScope is still loading the program;
  4. wait for the program to start and finish;
  5. ALWAYS give control back to the driver (io_and_status_controller/resend_robot_program) -- without
     its script the arm ignores every following trajectory -- and wait until it runs again.

The gripper's real width is known only if the programs write it into an output register
(gripper.width_feedback_register in cell.yaml); otherwise the commanded width is reported, marked
width_measured=false. SimGripper is the same interface without hardware (mock hardware / tests); NoGripper
answers 501 for a gripper model with no control yet (Robotiq 2F-85 on the real arm).
"""
import threading
import time

from dashboard import DashboardError
from rtde_regs import RTDEError

OUTCOME_STATUS = {"success": 200, "action_timeout": 504, "server_unreachable": 503, "action_failed": 500,
                  "recovery_failed": 500, "stopped": 200, "not_supported": 501}


class GripperResult(dict):
    pass


class _Base:
    def __init__(self, cell, ros):
        self.cell, self.ros = cell, ros
        g = cell.gripper
        self.max_width = float(g["max_width_m"])
        self.min_force, self.max_force = float(g["min_force_n"]), float(g["max_force_n"])
        self.default_force = float(g["default_force_n"])
        self.name = g.get("name", "gripper")
        self.width = None              # last known width, m
        self.width_measured = False
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

    def clamp(self, width, force):
        notes = []
        w = min(max(float(width), 0.0), self.max_width)
        if abs(w - width) > 1e-9:
            notes.append(f"width {width:.4f} m clamped to the {self.name} range 0..{self.max_width:.3f} m")
        f = self.default_force if force is None else float(force)
        fc = min(max(f, self.min_force), self.max_force)
        if abs(fc - f) > 1e-9:
            notes.append(f"force {f:.1f} N clamped to the {self.name} range {self.min_force:.0f}..{self.max_force:.0f} N")
        return w, fc, notes

    def _set_width(self, w, measured):
        self.width, self.width_measured = w, measured
        if self.ros is not None and w is not None:
            self.ros.publish_width(w)

    def state(self):
        return {"width_m": self.width, "width_measured": self.width_measured, "type": self.cell.gripper["type"],
                "model": self.cell.gripper_model, "backend": self.backend}

    def stop(self):
        self.stop_event.set()


class SimGripper(_Base):
    """No hardware: the fingers 'arrive' at once. A grasp closes to the requested width (the object)."""
    backend = "sim"

    def __init__(self, cell, ros):
        super().__init__(cell, ros)
        self._set_width(self.max_width, False)

    def move(self, width, force=None, closing=False, timeout_s=10.0):
        w, f, notes = self.clamp(width, force)
        self.stop_event.clear()
        time.sleep(0.2)
        self._set_width(w, False)
        return GripperResult(outcome="success", width_m=w, force_n=f, notes=notes, program=None, elapsed_s=0.2)

    def available(self):
        return True


class NoGripper(_Base):
    """A gripper model without control on the real arm yet (Robotiq 2F-85): every action answers 501, so a
    client never takes a grasp for done. The model still shows the fingers open."""
    backend = "none"

    def __init__(self, cell, ros):
        super().__init__(cell, ros)
        self._set_width(self.max_width, False)

    def move(self, width, force=None, closing=False, timeout_s=None):
        return GripperResult(outcome="not_supported", width_m=self.width, force_n=force, notes=[], program=None,
                             elapsed_s=0.0, error=f"no control for the {self.name} on the real arm yet "
                                                  "(gripper.model in config/cell.yaml / env UR_GRIPPER)")

    def available(self):
        return False


class RG6Gripper(_Base):
    backend = "rg6"

    def __init__(self, cell, ros, dashboard):
        super().__init__(cell, ros)
        self.dash = dashboard
        g = cell.gripper
        self.open_program, self.close_program = g["open_program"], g["close_program"]
        self.timeout_s = float(g.get("program_timeout_s", 10.0))
        self.width_reg, self.force_reg = int(g["width_register"]), int(g["force_register"])
        self.feedback_reg = int(g.get("width_feedback_register", -1))
        from rtde_regs import RegisterWriter
        self._regs = RegisterWriter(cell.robot_ip)

    # ---- RTDE registers (rtde_regs: only these registers, so it works next to the driver) ----

    def _write_registers(self, width_m, force_n):
        self._regs.write_doubles({self.width_reg: width_m * 1000.0, self.force_reg: force_n})

    def _read_feedback(self):
        if self.feedback_reg < 0:
            return None
        try:
            return self._regs.read_doubles([self.feedback_reg])[self.feedback_reg] / 1000.0
        except Exception:      # noqa: BLE001
            return None

    # ---- the program run ----

    def available(self):
        try:
            self.dash.running()
            return True
        except DashboardError:
            return False

    def _play_when_loaded(self, name, timeout_s=5.0):
        t0 = time.time()
        while True:
            try:
                self.dash.play()
                return
            except DashboardError as e:
                try:
                    if self.dash.running():       # an error answer does not prove it did not start
                        return
                except DashboardError:
                    pass
                if time.time() - t0 > timeout_s:
                    raise RuntimeError(
                        f"the dashboard refused to play {name} for {timeout_s:.0f} s ({e}); most likely the arm is "
                        "not at the program's first waypoint (a remote play cannot do 'Move Robot into Position'): "
                        "make its first move a Script movej(...), or check the program has no unset nodes") from e
                time.sleep(0.2)

    def _hand_back(self):
        """Give control back to the driver: resend its script and wait until it runs."""
        from std_srvs.srv import Trigger
        r = self.ros.call(Trigger, "/io_and_status_controller/resend_robot_program", Trigger.Request(), timeout_s=10.0)
        if not r.success:
            return False, r.message
        t0 = time.time()
        while time.time() - t0 < 5.0:
            try:
                if self.dash.running():               # the Dashboard, not the latched topic (see motion.py)
                    return True, r.message
            except DashboardError:
                pass
            time.sleep(0.1)
        return False, "the driver's program did not start within 5 s after resend_robot_program"

    def move(self, width, force=None, closing=False, timeout_s=None):
        """Run the open (or close) program to ``width`` with ``force``. Holds the gripper lock."""
        w, f, notes = self.clamp(width, force)
        program = self.close_program if closing else self.open_program
        timeout_s = self.timeout_s if timeout_s is None else timeout_s
        self.stop_event.clear()
        t0 = time.time()
        out = GripperResult(width_m=None, force_n=f, notes=notes, program=program)
        failed, outcome = None, "success"
        try:
            self._write_registers(w, f)
            if not self.dash.wait_stopped(5.0):
                raise RuntimeError(f"the controller is still busy before {program} (could not stop it)")
            self.dash.load(program)
            self._play_when_loaded(program)
            started, deadline = False, time.time() + timeout_s
            while True:
                if self.stop_event.is_set():
                    self.dash.stop()
                    outcome = "stopped"
                    break
                running = self.dash.running()
                started = started or running
                if started and not running:
                    break
                if time.time() > deadline:
                    self.dash.stop()
                    outcome = "action_timeout"
                    failed = RuntimeError(f"{program} did not finish within {timeout_s:.0f} s (started={started}); "
                                          "does it exist on the pendant?")
                    break
                time.sleep(0.05)
        except (DashboardError, RTDEError, RuntimeError, OSError) as e:
            failed, outcome = e, ("server_unreachable" if isinstance(e, (DashboardError, RTDEError, OSError))
                                  else "action_failed")
        # always: nothing may run when the driver's script is re-sent (resending while a .urp plays blocks)
        stopped = self.dash.wait_stopped(5.0) if self.available() else False
        ok, msg = (self._hand_back() if stopped else
                   (False, f"{program} is still running on the controller; press Stop on the pendant"))
        out["control_restored"] = ok
        if not ok:
            out["control_restore_msg"] = msg
            if failed is None:
                failed, outcome = RuntimeError(f"could not give control back to the driver: {msg}"), "action_failed"
        measured = self._read_feedback()
        if outcome == "success":
            self._set_width(measured if measured is not None else w, measured is not None)
        out.update(outcome=outcome, width_m=self.width, width_measured=self.width_measured,
                   elapsed_s=round(time.time() - t0, 2))
        if failed is not None:
            out["error"] = str(failed)
        return out


def make_gripper(cell, ros, dashboard, mock):
    if mock or cell.gripper.get("type") == "sim":
        return SimGripper(cell, ros)
    if cell.gripper_model == "rg6":
        return RG6Gripper(cell, ros, dashboard)
    return NoGripper(cell, ros)
