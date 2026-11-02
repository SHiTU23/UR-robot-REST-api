#!/usr/bin/env python3
"""Save the real arm's current joints as the pose the MOCK robot starts in (config/start_pose.yaml). READ-ONLY on the
robot. On the real arm MoveIt always starts from the real joints, so this file is not used there.
    tools/save_start_pose.py [robot ip]     (or ./run.sh save-start), then restart the stack"""
import math
import os
import socket
import struct
import sys
import time

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.56.101"
NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "start_pose.yaml")

s = socket.create_connection((IP, 30004), 3)
s.settimeout(3)


def send(cmd, payload=b""):
    s.sendall(struct.pack(">HB", 3 + len(payload), cmd) + payload)


def recv():
    n, c = struct.unpack(">HB", s.recv(3, socket.MSG_WAITALL))
    return c, s.recv(n - 3, socket.MSG_WAITALL) if n > 3 else b""


send(86, struct.pack(">H", 2))
recv()
send(79, struct.pack(">d", 10.0) + b"actual_q,actual_qd")
recv()
send(83)
while True:
    c, d = recv()
    if c == 85:
        v = struct.unpack(">12d", d[1:])
        break
send(80)
s.close()
q, qd = v[:6], v[6:]
if max(abs(x) for x in qd) > 0.01:
    sys.exit(f"the arm is moving ({max(abs(x) for x in qd):.3f} rad/s); nothing saved. Try again when it stands still.")
with open(OUT, "w") as f:
    f.write(f"# The pose the mock robot starts in, read from the real arm {time.strftime('%Y-%m-%d %H:%M')}"
            " by tools/save_start_pose.py. Radians.\n")
    for n, x in zip(NAMES, q):
        f.write(f"{n}: {x:.6f}\n")
print("saved:", [round(math.degrees(x), 2) for x in q], "deg")
