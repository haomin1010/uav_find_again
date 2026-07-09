import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bridge_image_args(gz_topic: str, ros_topic: str):
    return [
        ros_topic + "@sensor_msgs/msg/Image@ignition.msgs.Image",
        "--ros-args",
        "-r",
        ros_topic + ":=" + ros_topic,
        "-p",
        "expand_gz_topic_names:=true",
    ]


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("multi_uav_sim_bringup"))
    worlds_dir = Path(get_package_share_directory("multi_uav_sim_worlds"))
    default_config = str(bringup_dir / "config" / "mock_demo.yaml")
    default_world = str(worlds_dir / "worlds" / "multi_uav_reacquire.sdf")
    model_path = str(worlds_dir / "models")

    existing_model_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    combined_model_path = f"{model_path}:{existing_model_path}" if existing_model_path else model_path

    config = LaunchConfiguration("config")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    bridge_pkg = LaunchConfiguration("bridge_pkg")

    camera_bridges = []
    for uav_id in ("uav_1", "uav_2", "uav_3"):
        gz_topic = (
            f"/world/multi_uav_reacquire/model/{uav_id}/link/base_link/"
            "sensor/front_camera/image"
        )
        ros_topic = f"/{uav_id}/front_camera/image"
        camera_bridges.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    bridge_pkg,
                    "parameter_bridge",
                    gz_topic + "@sensor_msgs/msg/Image@ignition.msgs.Image",
                    "--ros-args",
                    "-r",
                    gz_topic + ":=" + ros_topic,
                ],
                output="screen",
            )
        )

    gazebo_gui_cmd = ["ign", "gazebo", "-r", world]
    gazebo_headless_cmd = ["ign", "gazebo", "-s", "-r", world]

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument(
                "bridge_pkg",
                default_value="ros_ign_bridge",
                description="Bridge package: ros_ign_bridge for Fortress, ros_gz_bridge for newer Gazebo.",
            ),
            SetEnvironmentVariable("IGN_IP", "127.0.0.1"),
            SetEnvironmentVariable("IGN_PARTITION", "multi_uav_reacquire"),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", combined_model_path),
            ExecuteProcess(cmd=gazebo_headless_cmd, output="screen", condition=IfCondition(headless)),
            ExecuteProcess(cmd=gazebo_gui_cmd, output="screen", condition=UnlessCondition(headless)),
            *camera_bridges,
            Node(
                package="multi_uav_sim_nodes",
                executable="image_video_recorder",
                name="image_video_recorder",
                output="screen",
                parameters=[config],
            ),
        ]
    )

