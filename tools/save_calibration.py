#!/usr/bin/env python3
"""Read the arm's factory calibration and save it for the driver and GET /robot/calibration. READ-ONLY on the robot.
    tools/save_calibration.py [robot ip]     (or ./run.sh save-calibration), then restart the stack"""
import os
import socket
import sys
import time

import yaml

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "ur_api"))
import ur_calibration as cal  # noqa: E402

IP = sys.argv[1] if len(sys.argv) > 1 else yaml.safe_load(open(os.path.join(ROOT, "config", "cell.yaml")))["robot"]["ip"]
OUT = os.path.join(ROOT, "config", "calibration", "ur_calibration.yaml")


def dashboard(*queries):
    out = {}
    with socket.create_connection((IP, 29999), timeout=3.0) as s:
        s.settimeout(3.0)
        s.recv(1024)
        for q in queries:
            s.sendall((q + "\n").encode())
            out[q] = s.recv(1024).decode().strip()
    return out


k = cal.read_kinematics(IP)
d = dashboard("get serial number", "get robot model", "version")
doc = {"kinematics": cal.to_kinematics(k),
       "source": {"robot_ip": IP, "serial": d["get serial number"], "model": d["get robot model"],
                  "polyscope": d["version"], "read_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                  "calibration_status": k["calibration_status"], "checksum": k["checksum"],
                  "dh": {n: k[n] for n in ("theta", "a", "d", "alpha")}}}
os.makedirs(os.path.dirname(OUT), exist_ok=True)
tmp = OUT + ".tmp"
with open(tmp, "w") as f:
    yaml.safe_dump(doc, f, sort_keys=False)
os.replace(tmp, OUT)
print(f"saved {os.path.normpath(OUT)}: {doc['kinematics']['hash']}, serial {d['get serial number']}, status {k['calibration_status']}")
