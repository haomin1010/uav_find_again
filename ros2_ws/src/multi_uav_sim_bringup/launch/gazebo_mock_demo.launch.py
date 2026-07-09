import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera_bridge_processes(bridge_pkg, bridge_image_type, record_video):
    processes = []
    for uav_id in ("uav_1", "uav_2", "uav_3"):
        gz_topic = (
            f"/world/multi_uav_reacquire/model/{uav_id}/link/base_link/"
            "sensor/front_camera/image"
        )
        ros_topic = f"/{uav_id}/front_camera/image"
        processes.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    bridge_pkg,
                    "parameter_bridge",
                    [gz_topic, "@sensor_msgs/msg/Image@", bridge_image_type],
                    "--ros-args",
                    "-r",
                    gz_topic + ":=" + ros_topic,
                ],
                output="screen",
                condition=IfCondition(record_video),
            )
        )
    return processes


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
    pose_sync = LaunchConfiguration("pose_sync")
    record_video = LaunchConfiguration("record_video")
    bridge_pkg = LaunchConfiguration("bridge_pkg")
    bridge_image_type = LaunchConfiguration("bridge_image_type")

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
            DeclareLaunchArgument(
                "pose_sync",
                default_value="false",
                description="Sync ROS2 mock poses into Gazebo models through ign service.",
            ),
            DeclareLaunchArgument(
                "record_video",
                default_value="false",
                description="Bridge Gazebo front cameras to ROS2 and save them as mp4 files.",
            ),
            DeclareLaunchArgument(
                "bridge_pkg",
                default_value="ros_gz_bridge",
                description="Bridge package: ros_gz_bridge or ros_ign_bridge, depending on your Gazebo install.",
            ),
            DeclareLaunchArgument(
                "bridge_image_type",
                default_value="gz.msgs.Image",
                description="Gazebo image message type for parameter_bridge; use ignition.msgs.Image with ros_ign_bridge.",
            ),
            SetEnvironmentVariable("IGN_IP", "127.0.0.1"),
            SetEnvironmentVariable("GZ_IP", "127.0.0.1"),
            SetEnvironmentVariable("IGN_PARTITION", "multi_uav_reacquire"),
            SetEnvironmentVariable("GZ_PARTITION", "multi_uav_reacquire"),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", combined_model_path),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", combined_model_path),
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
                condition=IfCondition(pose_sync),
            ),
            *camera_bridge_processes(bridge_pkg, bridge_image_type, record_video),
            Node(
                package="multi_uav_sim_nodes",
                executable="image_video_recorder",
                name="image_video_recorder",
                output="screen",
                parameters=[config],
                condition=IfCondition(record_video),
            ),
        ]
    )
