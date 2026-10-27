#!/usr/bin/env python3
"""Width table of a gripper model, computed from its URDF (not from a datasheet): for joint values over the
driving joint's range, the gap between the two finger pads' collision meshes along the closing axis (= the
gripper width) and the farthest fingertip point along the mount's z (= tip depth below the mount face).

    docker compose exec ros python3 /app/tools/gripper_table.py rg6
    docker compose exec ros python3 /app/tools/gripper_table.py robotiq_2f85

Prints the `table:` block for config/grippers/<model>.yaml. Needs xacro + the gripper's description package
(the ur-rest-api image has both)."""
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ur_api"))
from kinematics import Urdf  # noqa: E402


def gripper_urdf(model):
    src = (f'<robot name="g" xmlns:xacro="http://wiki.ros.org/xacro">'
           f'<xacro:include filename="{ROOT}/ros/urdf/grippers/{model}.xacro"/>'
           f'<link name="mount"/><xacro:gripper_{model} tf_prefix="" parent="mount" z="0" yaw="0"/></robot>')
    with tempfile.NamedTemporaryFile("w", suffix=".xacro") as f:
        f.write(src)
        f.flush()
        return subprocess.run(["xacro", f.name], check=True, capture_output=True, text=True).stdout


def pad_points(xml, links):
    """Collision mesh vertices of ``links`` in their link frames (origin offsets applied)."""
    root, out = ET.fromstring(xml), {}
    for l in root.findall("link"):
        if l.get("name") not in links:
            continue
        c = l.find("collision")
        m = c.find("geometry/mesh")
        path = m.get("filename").replace("file://", "")
        v = np.asarray(trimesh.load(path, force="mesh").vertices, float)
        o = c.find("origin")
        if o is not None and o.get("xyz"):
            v = v + np.array([float(x) for x in o.get("xyz").split()])     # (no rotated collision origins here)
        out[l.get("name")] = v
    return out


def main(model):
    cfg = yaml.safe_load(open(os.path.join(ROOT, "config", "grippers", f"{model}.yaml")))
    xml = gripper_urdf(model)
    u = Urdf(xml)
    j = cfg["joint"]
    left, right = cfg["pad_links"]
    pts = pad_points(xml, (left, right))
    lo, hi = u.joints[j]["limit"]
    rows = []
    for a in np.linspace(lo, hi, 41):
        P = {}
        for link in (left, right):
            T = u.fk(link, {j: float(a)}, "mount")
            P[link] = pts[link] @ T[:3, :3].T + T[:3, 3]
        ax = cfg["closing_axis_in_mount"]
        k = "xyz".index(ax)
        L, R = P[left][:, k], P[right][:, k]
        gap = (R.min() - L.max()) if R.mean() > L.mean() else (L.min() - R.max())
        tip = max(P[left][:, 2].max(), P[right][:, 2].max())
        rows.append((float(a), float(gap), float(tip)))
    print(f"# {model}: {u.joints[j]['name']} over [{lo}, {hi}], pads {left} / {right}, closing axis {ax}")
    print("table:   # [joint value, width m (pad gap), tip depth m (farthest pad point below the mount face)]")
    for a, w, t in rows:
        print(f"  - [{a:.5f}, {w:.5f}, {t:.5f}]")


if __name__ == "__main__":
    main(sys.argv[1])
