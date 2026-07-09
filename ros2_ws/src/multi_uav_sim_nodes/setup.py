from setuptools import find_packages, setup


package_name = "multi_uav_sim_nodes"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="find_object_again",
    maintainer_email="todo@example.com",
    description="ROS2 Python nodes for multi-UAV cooperative reacquisition simulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "target_manager = multi_uav_sim_nodes.target_manager:main",
            "uav_control = multi_uav_sim_nodes.uav_control:main",
            "vision_mock = multi_uav_sim_nodes.vision_mock:main",
            "coop_tracker = multi_uav_sim_nodes.coop_tracker:main",
            "sim_recorder = multi_uav_sim_nodes.sim_recorder:main",
            "gazebo_pose_sync = multi_uav_sim_nodes.gazebo_pose_sync:main",
            "image_video_recorder = multi_uav_sim_nodes.image_video_recorder:main",
        ],
    },
)
