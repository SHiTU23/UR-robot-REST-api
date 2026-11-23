"""Box colliders of a gripper, fitted to its collision meshes (config/grippers/<model>.yaml collider_boxes).

    python3 tools/gripper_colliders.py <expanded urdf> <model> [--meshes <dir>] [--step-deg 0.5] [--pads-aligned]

Run where numpy and trimesh are (the ur-api container: docker exec ur-api python3 /app/tools/gripper_colliders.py ...).
The URDF: the cell expanded with this gripper, e.g. in ur-ros
    xacro /app/ros/urdf/ur_cell.urdf.xacro gripper:=<model> tcp:= camera_mount:= ur_type:=ur10e name:=ur
--meshes: a folder holding the collision mesh files by name, when the URDF's mesh paths are not readable here.

Prints the collider_boxes block (paste it into the gripper's yaml):
  - body: one box on the hand link, axis-aligned in it, around the hand link's mesh and every gripper link fixed to it
    (an adapter plate, a mount); it does not move;
  - every other gripper link with a mesh (the fingers): the smallest box around its mesh, turned about the axis the finger
    swings around (the axis of the nearest moving joint above it), in that link's frame. These boxes ride on the finger
    links, so they open and close with every check's gripper_width_m.
--pads-aligned: the pad links (the yaml's pad_links) get a box aligned with their own frame, not the smallest turned one. For a
gripper whose linkage keeps the pads parallel (the Robotiq 2F-85): the smallest box of its L-shaped fingertip is tilted and
its corner stands ~18 mm into the gap between the pads (open 85 mm: the boxes left 49.6 mm); aligned, the inner face is the pad.
(2026-11-22) Written for the Robotiq 2F-85; the RG6's boxes (2026-11-21) were made the same way by a one-off script.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ur_api"))
import mesh_parse  # noqa: E402


def rpy_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin_T(el) -> np.ndarray:
    T = np.eye(4)
    o = el.find("origin") if el is not None else None
    if o is not None:
        T[:3, :3] = rpy_R(*[float(v) for v in o.get("rpy", "0 0 0").split()])
        T[:3, 3] = [float(v) for v in o.get("xyz", "0 0 0").split()]
    return T


def axis_R(axis: int, th: float) -> np.ndarray:
    c, s = math.cos(th), math.sin(th)
    return {0: np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            1: np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            2: np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def mesh_points(link, meshes_dir) -> np.ndarray | None:
    """the link's collision mesh vertices in the link frame (collision origin and scale applied), None without a mesh"""
    pts = []
    for c in link.findall("collision"):
        m = c.find("geometry/mesh")
        if m is None:
            continue
        fn = m.get("filename")
        path = os.path.join(meshes_dir, os.path.basename(fn)) if meshes_dir else fn.replace("file://", "")
        v, _ = mesh_parse.load(path)
        v = v * np.array([float(x) for x in (m.get("scale") or "1 1 1").split()])
        T = origin_T(c)
        pts.append(v @ T[:3, :3].T + T[:3, 3])
    return np.vstack(pts) if pts else None


def fit(P: np.ndarray, axis: int, step_deg: float) -> tuple:
    """(centre, size, angle) of the smallest box around P turned about `axis` by angle (rad)"""
    best = None
    for th in np.radians(np.arange(0.0, 180.0, step_deg)):
        R = axis_R(axis, th)
        Q = P @ R                                  # the points in the turned box frame
        lo, hi = Q.min(0), Q.max(0)
        vol = float(np.prod(hi - lo))
        if best is None or vol < best[0] - 1e-12:
            best = (vol, R @ ((lo + hi) / 2), hi - lo, th)
    return best[1], best[2], best[3]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("urdf")
    ap.add_argument("model")
    ap.add_argument("--meshes")
    ap.add_argument("--grippers-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "grippers"))
    ap.add_argument("--step-deg", type=float, default=0.5)
    ap.add_argument("--pads-aligned", action="store_true")
    a = ap.parse_args()
    import yaml
    gm = yaml.safe_load(open(os.path.join(a.grippers_dir, a.model + ".yaml")))
    root = ET.parse(a.urdf).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    hand = gm["hand_link"]
    names = list(gm["links"])

    def fixed_to_hand(n):
        """T_hand_n when n hangs on the hand link by fixed joints only (either way), else None"""
        T, cur = np.eye(4), n                       # n below the hand
        while cur != hand:
            j = by_child.get(cur)
            if j is None or j.get("type") != "fixed":
                break
            T = origin_T(j) @ T
            cur = j.find("parent").get("link")
        else:
            return T
        T, cur = np.eye(4), hand                   # n above the hand (an adapter plate)
        while cur != n:
            j = by_child.get(cur)
            if j is None or j.get("type") != "fixed":
                return None
            T = origin_T(j) @ T
            cur = j.find("parent").get("link")
        return np.linalg.inv(T)

    def swing_axis(n) -> int:
        cur = n
        while cur in by_child:
            j = by_child[cur]
            if j.get("type") in ("revolute", "continuous"):
                v = np.abs([float(x) for x in j.find("axis").get("xyz").split()])
                return int(np.argmax(v))
            cur = j.find("parent").get("link")
        raise SystemExit(f"{n}: no moving joint above it")

    body, out = [], []
    for n in names:
        P = mesh_points(links[n], a.meshes)
        if P is None:
            continue
        T = fixed_to_hand(n)
        if T is not None:
            body.append(P @ T[:3, :3].T + T[:3, 3])
            continue
        ax = swing_axis(n)
        if a.pads_aligned and n in (gm.get("pad_links") or []):
            lo, hi = P.min(0), P.max(0)
            c, s, th = (lo + hi) / 2, hi - lo, 0.0
        else:
            c, s, th = fit(P, ax, a.step_deg)
        rpy = [0.0, 0.0, 0.0]
        rpy[ax] = th                                # one turn about a single axis: rpy holds it as is
        short = n
        for pre in ("robotiq_85_", "rg6_"):
            short = short.replace(pre, "")
        short = short.replace("_link", "")
        out.append({"name": short, "link": n, "center": c, "rpy": rpy, "size": s, "pts": len(P)})
    B = np.vstack(body)
    lo, hi = B.min(0), B.max(0)
    print(f"# {a.model}: fitted by tools/gripper_colliders.py from {os.path.basename(a.urdf)} (body {sum(len(b) for b in body)} points, "
          f"links {', '.join(n for n in names if fixed_to_hand(n) is not None)})")
    print("collider_boxes:")
    f = lambda v: "[" + ", ".join(f"{x:.4f}" for x in v) + "]"  # noqa: E731
    print(f"  - {{name: body, link: {hand}, center: {f((lo + hi) / 2)}, size: {f(hi - lo)}}}")
    for b in out:
        print(f"  - {{name: {b['name']}, link: {b['link']}, center: {f(b['center'])}, rpy: {f(b['rpy'])}, size: {f(b['size'])}}}")


if __name__ == "__main__":
    main()
