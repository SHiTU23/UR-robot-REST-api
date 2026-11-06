"""The objects every route module needs, filled in by server.py at start (one process, one robot)."""
import threading


class Ctx:
    cell = None          # cell.Cell
    ros = None           # ros_io.RosIO
    kin = None           # kinematics.Kinematics
    motion = None        # motion.Motion
    dash = None          # dashboard.Dashboard (None with mock hardware)
    gripper = None       # gripper.RG6Gripper | SimGripper | NoGripper
    force = None         # force.ForceMonitor
    payload = None       # payload.PayloadIdentifier
    camera = None        # camera backend
    store = None         # file_store.FileStore
    core = None          # scene_core.SceneCore
    legacy = None        # legacy_scene.LegacyScene
    scene_io = None      # scene_io.MoveItSceneIO
    mock = False
    home_path = None     # where a home pose saved with POST /robot/home lives
    poses_path = None    # named poses saved with POST /robot/poses
    lock = threading.Lock()     # THE motion lock: motion + gripper endpoints, scene commits; non-blocking -> 409
    last_motion = None          # for /control/resume
    activity = None             # what holds the lock: {"what", "since"}, shown by /health and the panel


C = Ctx
