#!/usr/bin/env python3
"""Read the controller's active TCP now and save it as the TCP (robot.tcp_file in config/cell.yaml). READ-ONLY on
the robot.    tools/save_tcp.py [robot ip]   (or ./run.sh save-tcp), then ./run.sh restart.
The same as POST /robot/tcp/from_controller. Check first that the active TCP is the right one: the RG6 URCap switches it."""
import os
import sys

import yaml

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "ur_api"))
import ur_tcp  # noqa: E402
from gripper_select import selected_gripper  # noqa: E402

cell = yaml.safe_load(open(os.path.join(ROOT, "config", "cell.yaml")))
IP = sys.argv[1] if len(sys.argv) > 1 else cell["robot"]["ip"]
model = selected_gripper(cell["gripper"]["model"])[0]       # as the stack: env UR_GRIPPER, web page, cell.yaml
path = cell["robot"]["tcp_file"].replace("{model}", model).replace("/app/", ROOT + "/", 1)
doc = ur_tcp.save(path, ur_tcp.read_tcp(IP), model, IP)
print(f"active TCP on {IP}: xyz {doc['xyz_m']} m, rotation vector {doc['rotvec_rad']} rad (rpy {doc['rpy_rad']})")
print(f"saved to {os.path.normpath(path)} for gripper {model}; restart the stack to use it (./run.sh restart)")
