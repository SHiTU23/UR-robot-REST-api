"""robot_state_publisher with the cell description (UR + tool). Passed to ur_control.launch.py as its
description_launchfile: it is ur_robot_driver's ur_rsp.launch.py with description_file pointing at
ros/urdf/ur_cell.urdf.xacro.

The including launch file's configurations (ur_type, robot_ip, use_mock_hardware, headless_mode, tf_prefix,
...) are visible here but are NOT inherited by a nested include that gets launch_arguments, so they are
forwarded explicitly. This matters: use_mock_hardware in the URDF decides between mock and real hardware."""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

HERE = os.path.dirname(os.path.abspath(__file__))
FORWARD = ("ur_type", "robot_ip", "use_mock_hardware", "mock_sensor_commands", "headless_mode", "tf_prefix",
           "safety_limits", "safety_pos_margin", "safety_k_position", "kinematics_params_file", "reverse_ip",
           "script_command_port", "reverse_port", "script_sender_port", "trajectory_port", "use_tool_communication")


def _include(context):
    args = {k: v for k, v in context.launch_configurations.items() if k in FORWARD}
    args["description_file"] = os.path.join(HERE, "..", "urdf", "ur_cell.urdf.xacro")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py"])),
        launch_arguments=args.items())]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_include)])
