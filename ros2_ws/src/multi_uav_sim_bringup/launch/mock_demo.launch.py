from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("multi_uav_sim_bringup"))
    default_config = str(bringup_dir / "config" / "mock_demo.yaml")
    config = LaunchConfiguration("config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to the mock demo YAML parameter file.",
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
        ]
    )

