"""The whole ROS side of the UR cell: ur_robot_driver (with the cell description), move_group, and the gripper
finger state publisher. The REST API (ur_api/server.py) runs as a separate process and talks to these.

    ros2 launch /app/ros/launch/cell.launch.py robot_ip:=<controller IP>          # real arm or URSim
    ros2 launch /app/ros/launch/cell.launch.py use_mock_hardware:=true             # no robot at all

headless_mode (default true): the driver sends its control script itself, so the pendant needs no External
Control program -- but the robot must be in Remote Control mode (the RG6 pendant programs need that too)."""
import os
import sys

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.abspath(__file__))
CELL = yaml.safe_load(open(os.environ.get("UR_CELL_CONFIG", os.path.join(HERE, "..", "..", "config", "cell.yaml"))))
GRIPPER = os.environ.get("UR_GRIPPER") or CELL["gripper"]["model"]      # the same choice as the URDF / SRDF
KINEMATICS = CELL["robot"].get("kinematics_params_file")     # the arm's calibration, if this cell has one
GM = yaml.safe_load(open(os.path.join(HERE, "..", "..", "config", "grippers", GRIPPER + ".yaml")))
TCP_FILE = (CELL["robot"].get("tcp_file") or "").replace("{model}", GRIPPER) or None   # this gripper's saved TCP (ur_tcp.py)
sys.path.insert(0, os.path.join(HERE, "..", "..", "ur_api"))
import ur_tcp  # noqa: E402
import camera_mount  # noqa: E402


def _tcp(mock, ip):
    """Hand the saved TCP to the URDF through env UR_TCP. The saved file wins (tcp_mode: file, the default): the
    controller is read only to create it when there is none, and to warn when its active TCP differs. tcp_mode:
    controller / follow read the controller's active TCP at every start and save it when it differs (follow also keeps
    it up to date while running: ur_api/tcp_follow.py). None saved (or for another gripper): the model's.
    The mode: gripper.<model>.tcp_mode, else robot.tcp_mode (ur_tcp.mode_for)."""
    mode = ur_tcp.mode_for(CELL, GRIPPER)
    print(f"[cell.launch] tcp_mode {mode} for {GRIPPER}", flush=True)
    d = ur_tcp.load(TCP_FILE, GRIPPER) if TCP_FILE else None
    if TCP_FILE and not mock:
        try:
            live = ur_tcp.read_tcp(ip)
            if d is None or (mode in ("controller", "follow") and not ur_tcp.same(live, d["xyz_m"], d["rpy_rad"])):
                why = ur_tcp.check(live[:3])
                if why:
                    raise RuntimeError(f"the controller's active TCP {[round(v, 5) for v in live]} is not plausible: {why}")
                d = ur_tcp.save(TCP_FILE, live, GRIPPER, ip)
                print(f"[cell.launch] TCP read from the controller at {ip} and saved to {TCP_FILE}", flush=True)
            elif max(abs(a - b) for a, b in zip(live[:3], d["xyz_m"])) > 0.001 or \
                    max(abs(a - b) for a, b in zip(ur_tcp.rotvec_to_rpy(live[3:6]), d["rpy_rad"])) > 0.002:
                print(f"[cell.launch] NOTE: the controller's active TCP {[round(v, 5) for v in live]} differs from the "
                      f"saved one; the saved one is used (POST /robot/tcp/from_controller to take the controller's)",
                      flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[cell.launch] WARNING: could not read the TCP from {ip} ({e})", flush=True)
    os.environ["UR_TCP"] = ur_tcp.as_arg(d) if d else ""
    print(f"[cell.launch] TCP: {'xyz ' + str(d['xyz_m']) + ' rpy ' + str(d['rpy_rad']) + ' (' + ur_tcp.describe(d) + ', ' + TCP_FILE + ')' if d else 'from the gripper model (config/grippers/)'}", flush=True)


def _camera():
    """(2026-11-20) Hand the saved camera mount (the client's hand-eye calibration, PUT /robot/camera_mount) to the URDF
    through env UR_CAMERA_MOUNT; none saved: tool.camera_xyz_m / camera_rpy_rad of config/cell.yaml, as before."""
    # (2026-11-22) the mounted gripper SET's own camera position (else the shared file) and its own camera holder (camera_mount.tool_for)
    d, path, own = camera_mount.load_for(CELL, GRIPPER)
    os.environ["UR_CAMERA_MOUNT"] = camera_mount.as_arg(d, camera_mount.tool_for(CELL["tool"], GM)) if d else ""
    if d and not own:
        print(f"[cell.launch] camera mount: no hand-eye of the {GRIPPER} set of its own yet: the shared one ({path})", flush=True)
    print(f"[cell.launch] camera mount: {'xyz ' + str(d['xyz_m']) + ' rpy ' + str(d['rpy_rad']) + ' (' + path + ', ' + str((d.get('source') or {}).get('at')) + ')' if d else 'config/cell.yaml tool.camera_*'}", flush=True)


MODELS_DIR = os.path.join(HERE, "..", "..", "state", "robot_models")


def _other_models(ur_type):
    """(2026-11-22) the robot model of EVERY gripper (config/grippers/), not only the loaded one, to state/robot_models/<model>.urdf, so
    GET /robot/colliders?gripper=<model> can show the boxes of a gripper that is not mounted (pipeFit's 3-D view of a scene captured with
    it; planning always uses the loaded model). Each as its SET: its own camera position (camera_mount.load_for) and camera holder
    (tool_for); the loaded gripper with the TCP this start uses, the others with their saved TCP (robot.tcp_file), else their model's. In a thread: the start does not wait; a failure is only printed."""
    import subprocess
    import threading
    from gripper_select import gripper_models

    def run():
        os.makedirs(MODELS_DIR, exist_ok=True)
        tcp_tpl = CELL["robot"].get("tcp_file") or ""
        for m in gripper_models():
            if m == GRIPPER:
                tcp, cam = os.environ.get("UR_TCP", ""), os.environ.get("UR_CAMERA_MOUNT", "")
            else:
                d = ur_tcp.load(tcp_tpl.replace("{model}", m), m) if tcp_tpl else None
                tcp = ur_tcp.as_arg(d) if d else ""
                with open(os.path.join(HERE, "..", "..", "config", "grippers", m + ".yaml")) as f:
                    gm = yaml.safe_load(f)
                cm, _, _ = camera_mount.load_for(CELL, m)          # that SET's camera position and holder
                cam = camera_mount.as_arg(cm, camera_mount.tool_for(CELL["tool"], gm)) if cm else ""
            out = os.path.join(MODELS_DIR, m + ".urdf")
            try:
                r = subprocess.run(["xacro", os.path.join(HERE, "..", "urdf", "ur_cell.urdf.xacro"), f"gripper:={m}", f"tcp:={tcp}",
                                    f"camera_mount:={cam}", f"ur_type:={ur_type}", "name:=ur"],
                                   capture_output=True, text=True, timeout=120)
                if r.returncode != 0:
                    raise RuntimeError(r.stderr.strip()[-300:])
                with open(out + ".tmp", "w") as f:
                    f.write(r.stdout)
                os.replace(out + ".tmp", out)
                print(f"[cell.launch] robot model of {m} -> {out}", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"[cell.launch] robot model of {m} NOT written: {e}", flush=True)

    threading.Thread(target=run, daemon=True).start()


def _nodes(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    mock = lc("use_mock_hardware").lower() == "true"
    _tcp(mock, lc("robot_ip") or CELL["robot"]["ip"])
    _camera()
    _other_models(lc("ur_type"))
    calibration = {"kinematics_params_file": KINEMATICS} if KINEMATICS else {}
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])),
        launch_arguments={
            "ur_type": lc("ur_type"), "robot_ip": lc("robot_ip"), "use_mock_hardware": lc("use_mock_hardware"),
            "headless_mode": lc("headless_mode"), "launch_rviz": "false",
            "description_launchfile": os.path.join(HERE, "rsp.launch.py"),
            # mock hardware has no speed-scaling interface for the scaled controller
            "initial_joint_controller": "joint_trajectory_controller" if mock else "scaled_joint_trajectory_controller",
            **calibration,
        }.items())
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "moveit.launch.py")),
        launch_arguments={"ur_type": lc("ur_type"), "launch_rviz": lc("launch_rviz"),
                          "use_mock_hardware": lc("use_mock_hardware")}.items())
    fingers = Node(executable=sys.executable, arguments=[os.path.join(HERE, "..", "scripts", "finger_state_publisher.py")],
                   name="finger_state_publisher", output="screen",
                   parameters=[{"joint": GM["joint"], "max_width_m": float(GM.get("allowed_max_width_m", GM["max_width_m"])),   # (2026-11-21) the cell's limit, not the stroke
                                "table_joint": [float(r[0]) for r in GM["table"]],
                                "table_width": [float(r[1]) for r in GM["table"]]}])
    return [driver, moveit, fingers]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value=CELL["robot"]["ur_type"]),
        DeclareLaunchArgument("robot_ip", default_value=os.environ.get("UR_ROBOT_IP", CELL["robot"]["ip"])),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("headless_mode", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        OpaqueFunction(function=_nodes),
    ])
