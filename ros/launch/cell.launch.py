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
GM = yaml.safe_load(open(os.path.join(HERE, "..", "..", "config", "grippers", GRIPPER + ".yaml")))


def _nodes(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    mock = lc("use_mock_hardware").lower() == "true"
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])),
        launch_arguments={
            "ur_type": lc("ur_type"), "robot_ip": lc("robot_ip"), "use_mock_hardware": lc("use_mock_hardware"),
            "headless_mode": lc("headless_mode"), "launch_rviz": "false",
            "description_launchfile": os.path.join(HERE, "rsp.launch.py"),
            # mock hardware has no speed-scaling interface for the scaled controller
            "initial_joint_controller": "joint_trajectory_controller" if mock else "scaled_joint_trajectory_controller",
        }.items())
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, "moveit.launch.py")),
        launch_arguments={"ur_type": lc("ur_type"), "launch_rviz": lc("launch_rviz"),
                          "use_mock_hardware": lc("use_mock_hardware")}.items())
    fingers = Node(executable=sys.executable, arguments=[os.path.join(HERE, "..", "scripts", "finger_state_publisher.py")],
                   name="finger_state_publisher", output="screen",
                   parameters=[{"joint": GM["joint"], "max_width_m": float(GM["max_width_m"]),
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
