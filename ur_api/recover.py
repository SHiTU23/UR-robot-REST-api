"""Getting motions going again after a Stop left MoveIt waiting (2026-11-20).

Seen on the real arm: Stop (the "stop" execution event + the goal cancel) left the scaled trajectory controller with its goal
in CANCELING; MoveIt kept its own execute goal EXECUTING, waiting for it, and acknowledged no new goal: every later motion
answered 504 "goal not acknowledged within 10 s" while the arm stood still. The mock controller never does this.

check()        the goals still active although the API runs no motion (the caller holds C.lock)
unstick()      restart the trajectory controller (deactivate + activate: aborts its goal, the arm holds still), then wait
               for MoveIt's goal to end
probe()        an EMPTY trajectory to /execute_trajectory: MoveIt answers at once without commanding anything
after_stop()   Motion.stop runs it: STOP_SETTLE_S after a Stop, unstick if goals are still active
run()          POST /control/recover: unstick if needed, probe, and if MoveIt still does not answer restart the ROS side
"""
import logging
import os
import threading
import time

from ctx import C

log = logging.getLogger("ur_rest_api")
STOP_SETTLE_S = 3.0           # a cancelled goal normally ends well within this
END_WAIT_S = 5.0
PROBE_ACK_S = 5.0


def controller():
    return "joint_trajectory_controller" if C.mock else "scaled_joint_trajectory_controller"


def check():
    return C.ros.active_goals()


def unstick(why):
    stale = check()
    log.warning("recover (%s): goals still active with no motion running: %s -> restarting %s", why, stale, controller())
    ok = C.ros.restart_controller(controller())
    t0 = time.time()
    while check() and time.time() - t0 < END_WAIT_S:
        time.sleep(0.1)
    left = check()
    log.warning("recover: controller restart %s, goals still active after it: %s", "ok" if ok else "NOT ok", left or "none")
    return {"stale_goals": stale, "controller": controller(), "controller_restarted": ok, "still_active": left}


def probe():
    """True if MoveIt acknowledges (and finishes) an empty trajectory, else why not."""
    from moveit_msgs.action import ExecuteTrajectory
    try:
        accepted, result, _ = C.ros.run_action(ExecuteTrajectory, "/execute_trajectory", ExecuteTrajectory.Goal(),
                                               timeout_s=PROBE_ACK_S, ack_s=PROBE_ACK_S)
    except Exception as e:  # noqa: BLE001
        return getattr(e, "msg", None) or str(e)
    if not accepted:
        return "MoveIt rejected the probe goal"
    return True if result is not None else "MoveIt accepted the probe but did not finish it"


def after_stop():
    """Motion.stop: once the stopped request has let go of the lock, unstick what the Stop left behind."""
    def go():
        time.sleep(STOP_SETTLE_S)
        t0 = time.time()
        while not C.lock.acquire(blocking=False):        # the stopped request still answering, or a new motion
            if time.time() - t0 > 10.0:
                return
            time.sleep(0.2)
        try:
            if C.motion._goals:                          # a new motion runs: its goals are not stale
                return
            if check():
                unstick("after a Stop")
        except Exception as e:  # noqa: BLE001
            log.error("recover after a Stop failed: %s", e)
        finally:
            C.lock.release()
    threading.Thread(target=go, name="recover-after-stop", daemon=True).start()


def run(state_dir):
    """The caller holds C.lock. -> (HTTP status, body)."""
    out = {"stale_goals": check()}
    if out["stale_goals"]:
        out.update(unstick("POST /control/recover"))
    p = probe()
    out["moveit_answers"] = p is True
    if p is True:
        out["outcome"] = "recovered" if out["stale_goals"] else "ok"
        out["msg"] = ("MoveIt was waiting for a stopped goal; the trajectory controller was restarted and motions work again"
                      if out["stale_goals"] else "MoveIt answers: motions work")
        return 200, out
    out["probe_error"] = p
    if C.mock:
        out.update(outcome="failed", msg="MoveIt does not answer (mock hardware: restart it with ./run.sh mock)")
        return 503, out
    log.error("recover: MoveIt still does not answer (%s): restarting the ROS side", p)
    import tcp_follow

    def _go():
        time.sleep(1.0)
        tcp_follow.restart_stack(state_dir, "recover: MoveIt did not answer")
    threading.Thread(target=_go, daemon=True).start()
    out.update(outcome="restarting", msg="MoveIt still does not answer: the API and the ROS side restart (about 30-60 s)")
    return 202, out
