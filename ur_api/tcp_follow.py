"""tcp_mode: follow -- keep the API's TCP equal to the controller's active TCP (the pendant's, e.g. TCP_1).

The tcp link is part of the robot description, so a new TCP needs the stack restarted. Every POLL_S this thread reads
the controller's active TCP (READ-ONLY RTDE, ur_tcp.read_tcp). When it differs from the one in use, it is the same on
STABLE_READS reads in a row (a program switching the TCP for a moment does not count), it is plausible, the motion lock
is free and nothing used the arm for IDLE_S, the thread keeps the lock (no motion can start), saves the TCP to the
gripper's tcp file, asks the ROS side to restart (state/restart_ros, watched by ros/scripts/cell.sh) and exits the API
(Docker starts it again). Both are back in about 30-60 s with the new TCP. Never moves the robot. At most one restart
per MIN_RESTART_GAP_S (state/tcp_follow.json): if the TCP still differs after one, it only logs."""
import json
import logging
import os
import threading
import time

import ur_tcp
from ctx import C

log = logging.getLogger("ur_rest_api")
POLL_S = 5.0
STABLE_READS = 2
IDLE_S = 15.0
MIN_RESTART_GAP_S = 120.0
ROS_ACK_WAIT_S = 20.0


def start(state_dir):
    if C.mock or C.cell.tcp_mode != "follow" or not C.cell.tcp_file:
        return
    threading.Thread(target=_loop, args=(state_dir,), name="tcp_follow", daemon=True).start()
    log.info("tcp_mode follow: the TCP follows the controller's active TCP (checked every %.0f s)", POLL_S)


def _in_use():
    c = C.cell
    return c.tcp_xy + [c.tcp_z], c.tcp_rpy


def _loop(state_dir):
    last, count, warned = None, 0, None
    stamp = os.path.join(state_dir, "tcp_follow.json")
    while True:
        time.sleep(POLL_S)
        if ur_tcp.mode_for(C.cell.cfg, C.cell.gripper_model) != "follow":    # switched off through the API meanwhile
            continue
        try:
            live = ur_tcp.read_tcp(C.cell.robot_ip)
        except Exception as e:  # noqa: BLE001  (controller rebooting, cable out: try again)
            if warned != "read":
                log.warning("tcp follow: cannot read the controller's TCP (%s); retrying", e)
                warned = "read"
            continue
        if ur_tcp.same(live, *_in_use()):
            last, count, warned = None, 0, None
            continue
        count = count + 1 if last is not None and ur_tcp.same(live, last[:3], ur_tcp.rotvec_to_rpy(last[3:])) else 1
        last = live
        if count < STABLE_READS:
            continue
        why = ur_tcp.check(live[:3])
        if why:
            if warned != "implausible":
                log.error("tcp follow: the controller's active TCP %s is not plausible (%s): kept the one in use",
                          [round(v, 5) for v in live], why)
                warned = "implausible"
            continue
        try:
            with open(stamp) as f:
                ago = time.time() - float(json.load(f).get("restarted_at", 0))
        except (OSError, ValueError, TypeError, AttributeError):
            ago = 1e9
        if ago < MIN_RESTART_GAP_S:
            if warned != "gap":
                log.error("tcp follow: the TCP still differs %.0f s after the last restart for it: not restarting again "
                          "(in use %s, controller %s); check the logs of ur-ros", ago, _in_use(), live)
                warned = "gap"
            continue
        if time.time() - (C.last_activity_end or 0) < IDLE_S or not C.lock.acquire(blocking=False):
            continue                     # the arm is in use: try again later
        if C.activity is not None:       # a request without the lock is running (should not happen): leave it
            C.lock.release()
            continue
        _apply(live, state_dir, stamp)   # keeps the lock until the process exits


def _apply(live, state_dir, stamp):
    xyz, rpy = _in_use()
    log.warning("tcp follow: the controller's active TCP changed (in use xyz %s rpy %s -> controller %s): saving it and "
                "restarting the API and the ROS side (about 30-60 s)", [round(v, 5) for v in xyz],
                [round(v, 5) for v in rpy], [round(v, 5) for v in live])
    ur_tcp.save(C.cell.tcp_file, live, C.cell.gripper_model, C.cell.robot_ip, "controller",
                "the controller's active TCP, followed while running (tcp_mode: follow)")
    with open(stamp, "w") as f:
        json.dump({"restarted_at": time.time(), "tcp": live}, f)
    restart_stack(state_dir, "tcp changed")


def restart_stack(state_dir, why):
    """Ask the ROS side to restart (state/restart_ros, ros/scripts/cell.sh), wait until it has stopped, exit the API
    (Docker starts both again: ~30-60 s). The caller holds C.lock, so no motion starts meanwhile. Does not return."""
    req = os.path.join(state_dir, "restart_ros")
    with open(req, "w") as f:
        f.write(why + "\n")
    t0 = time.time()                     # cell.sh deletes it once the ROS launch has stopped: then the new API
    while os.path.exists(req) and time.time() - t0 < ROS_ACK_WAIT_S:     # cannot pick up the old robot description
        time.sleep(0.5)
    if os.path.exists(req):
        log.error("tcp follow: the ROS side did not acknowledge the restart within %.0f s (is ur-ros running "
                  "ros/scripts/cell.sh?); restart it by hand: docker restart ur-ros ur-api", ROS_ACK_WAIT_S)
    os._exit(3)
