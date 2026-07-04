import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("multi_uav_sim_bringup"))
    worlds_dir = Path(get_package_share_directory("multi_uav_sim_worlds"))
    default_config = str(bringup_dir / "config" / "mock_demo.yaml")
    default_world = str(worlds_dir / "worlds" / "multi_uav_reacquire.sdf")
    model_path = str(worlds_dir / "models")

    existing_model_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    if existing_model_path:
        combined_model_path = f"{model_path}:{existing_model_path}"
    else:
        combined_model_path = model_path

    config = LaunchConfiguration("config")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    gazebo_gui_cmd = [
        "ign",
        "gazebo",
        "-r",
        world,
    ]

    # Fortress uses `ign gazebo -s` for server-only/headless mode.
    # Keep the launch argument string-based so it is easy to inspect.
    gazebo_headless_cmd = [
        "ign",
        "gazebo",
        "-s",
        "-r",
        world,
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("headless", default_value="true"),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", combined_model_path),
            ExecuteProcess(
                cmd=gazebo_headless_cmd,
                output="screen",
                condition=IfCondition(headless),
            ),
            ExecuteProcess(
                cmd=gazebo_gui_cmd,
                output="screen",
                condition=UnlessCondition(headless),
            ),
            Node(
                package="multi_uav_sim_nodes",
                executable="target_manager",
                name="target_manager",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="multi_uav_sim_nodes",
                executable="uav_control",
                name="uav_control",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="multi_uav_sim_nodes",
                executable="vision_mock",
                name="vision_mock",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="multi_uav_sim_nodes",
                executable="coop_tracker",
                name="coop_tracker",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="multi_uav_sim_nodes",
                executable="sim_recorder",
                name="sim_recorder",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="multi_uav_sim_nodes",
                executable="gazebo_pose_sync",
                name="gazebo_pose_sync",
                output="screen",
                parameters=[config],
            ),
        ]
    )
