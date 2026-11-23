"""Which gripper model is in use: env UR_GRIPPER forces one, else the web page's choice, else the cell config.
No third-party imports, so host tools (tools/save_tcp.py) can use it too."""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# the gripper picked on the web page (POST /gripper/model); git-ignored, kept across restarts
GRIPPER_SELECTION = os.path.join(ROOT, "state", "gripper_model")


def gripper_models():
    """The gripper models this cell has a model for: config/grippers/<model>.yaml."""
    d = os.path.join(ROOT, "config", "grippers")
    return sorted(f[:-5] for f in os.listdir(d) if f.endswith(".yaml"))


def selected_gripper(config_model):
    """(model, source): env UR_GRIPPER forces one; else the web page's choice; else gripper.model in the cell config.
    ros/scripts/cell.sh applies the same order on the ROS side, so the URDF and the API always agree."""
    if os.environ.get("UR_GRIPPER"):
        return os.environ["UR_GRIPPER"], "env UR_GRIPPER"
    try:
        with open(GRIPPER_SELECTION) as f:
            picked = f.read().strip()
    except OSError:
        picked = ""
    if picked in gripper_models():
        return picked, "web page"
    return config_model, "cell config"
