import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def px4_process(instance, model_name, pose, px4_root, px4_model, px4_autostart, px4_world, start_px4):
    return ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            [
                "cd ",
                px4_root,
                " && PX4_SYS_AUTOSTART=",
                px4_autostart,
                " PX4_GZ_MODEL=",
                px4_model,
                " PX4_GZ_MODEL_NAME=",
                model_name,
                " PX4_GZ_WORLD=",
                px4_world,
                " PX4_GZ_STANDALONE=1",
                " PX4_GZ_MODEL_POSE=\"",
                pose,
                "\" ./build/px4_sitl_default/bin/px4 -i ",
                str(instance),
            ],
        ],
        output="screen",
        condition=IfCondition(start_px4),
    )


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("multi_uav_sim_bringup"))
    worlds_dir = Path(get_package_share_directory("multi_uav_sim_worlds"))
    default_config = str(bringup_dir / "config" / "sitl_demo.yaml")
    default_world = str(worlds_dir / "worlds" / "multi_uav_reacquire_sitl.sdf")
    model_path = str(worlds_dir / "models")
    default_px4_root = os.environ.get("PX4_ROOT", "external/PX4-Autopilot")

    existing_model_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    if existing_model_path:
        combined_model_path = f"{model_path}:{existing_model_path}"
    else:
        combined_model_path = model_path

    config = LaunchConfiguration("config")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_px4 = LaunchConfiguration("start_px4")
    start_agent = LaunchConfiguration("start_agent")
    px4_root = LaunchConfiguration("px4_root")
    px4_model = LaunchConfiguration("px4_model")
    px4_autostart = LaunchConfiguration("px4_autostart")
    px4_world = LaunchConfiguration("px4_world")
    auto_arm = LaunchConfiguration("auto_arm")
    auto_offboard = LaunchConfiguration("auto_offboard")

    gazebo_gui_cmd = ["ign", "gazebo", "-r", world]
    gazebo_headless_cmd = ["ign", "gazebo", "-s", "-r", world]

    px4_instances = [
        px4_process(1, "uav_1", "-24,-12,0,0,0,0.24", px4_root, px4_model, px4_autostart, px4_world, start_px4),
        px4_process(2, "uav_2", "-16,-18,0,0,0,1.57", px4_root, px4_model, px4_autostart, px4_world, start_px4),
        px4_process(3, "uav_3", "-7,-11,0,0,0,0.12", px4_root, px4_model, px4_autostart, px4_world, start_px4),
        px4_process(4, "target", "-16,-10,0,0,0,0", px4_root, px4_model, px4_autostart, px4_world, start_px4),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_px4", default_value="true"),
            DeclareLaunchArgument("start_agent", default_value="true"),
            DeclareLaunchArgument("px4_root", default_value=default_px4_root),
            DeclareLaunchArgument("px4_model", default_value="x500"),
            DeclareLaunchArgument("px4_autostart", default_value="4001"),
            DeclareLaunchArgument("px4_world", default_value="multi_uav_reacquire"),
            DeclareLaunchArgument("auto_arm", default_value="true"),
            DeclareLaunchArgument("auto_offboard", default_value="true"),
            SetEnvironmentVariable("IGN_IP", "127.0.0.1"),
            SetEnvironmentVariable("GZ_IP", "127.0.0.1"),
            SetEnvironmentVariable("IGN_PARTITION", "multi_uav_reacquire"),
            SetEnvironmentVariable("GZ_PARTITION", "multi_uav_reacquire"),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", combined_model_path),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", combined_model_path),
            ExecuteProcess(
                cmd=gazebo_headless_cmd,
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", start_gazebo, "' == 'true' and '", headless, "' == 'true'"])
                ),
            ),
            ExecuteProcess(
                cmd=gazebo_gui_cmd,
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", start_gazebo, "' == 'true' and '", headless, "' == 'false'"])
                ),
            ),
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
                output="screen",
                condition=IfCondition(start_agent),
            ),
            *px4_instances,
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
                executable="sitl_offboard_adapter",
                name="sitl_offboard_adapter",
                output="screen",
                parameters=[
                    config,
                    {
                        "auto_arm": auto_arm,
                        "auto_offboard": auto_offboard,
                    },
                ],
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
