"""move_group for the UR cell: ur_moveit_config's configuration (kinematics, joint limits, OMPL, Pilz, CHOMP,
controllers) with the cell SRDF (planning tip = tcp). The robot description comes from the
robot_state_publisher topic, as in ur_moveit_config's own launch file."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

HERE = os.path.dirname(os.path.abspath(__file__))
SRDF = Path(HERE).parent / "srdf" / "ur_cell.srdf.xacro"


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    mc = (MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
          .robot_description_semantic(SRDF, {"name": ur_type})
          .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"], default_planning_pipeline="ompl")
          .to_moveit_configs())
    mock = LaunchConfiguration("use_mock_hardware")
    # MoveIt sends trajectories to the DEFAULT controller of moveit_controllers.yaml (the scaled one). On mock
    # hardware the driver runs joint_trajectory_controller instead (cell.launch.py), so make that the default.
    ctrl = {"moveit_simple_controller_manager.scaled_joint_trajectory_controller.default":
            PythonExpression(["'", mock, "' != 'true'"]),
            "moveit_simple_controller_manager.joint_trajectory_controller.default":
            PythonExpression(["'", mock, "' == 'true'"])}
    params = [mc.to_dict(), ctrl, {"publish_robot_description_semantic": True,
                             # the scene services the API uses; monitor everything for RViz
                             "publish_planning_scene": True, "publish_geometry_updates": True,
                             "publish_state_updates": True, "publish_transforms_updates": True}]

    wait = Node(package="ur_robot_driver", executable="wait_for_robot_description", output="screen")
    move_group = Node(package="moveit_ros_move_group", executable="move_group", output="screen", parameters=params)
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2_moveit", output="log", condition=IfCondition(launch_rviz),
                arguments=["-d", os.path.join(HERE, "..", "rviz", "cell.rviz")],
                parameters=[mc.robot_description_semantic, mc.robot_description_kinematics, mc.planning_pipelines,
                            mc.joint_limits])
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur10e"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        wait,
        RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=[move_group, rviz])),
    ])
