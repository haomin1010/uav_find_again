# find_object_again

多无人机协同视觉跟踪、目标丢失与再捕获的 Gazebo 仿真环境。

## 快速运行 mock demo

在 ROS2/Gazebo 环境安装完成前，可以先运行纯 Python 演示：

```bash
pip install -r requirements.txt
python run_mock_demo.py
```

输出目录类似：

```text
runs/20260704_164041_mock_reacquire/
  topdown.mp4
  events.jsonl
  metrics.json
  tracks.csv
  config.json
```

默认场景会演示 3 架无人机协同跟随目标，`uav_1` 在约 18.7 秒因视觉遮挡丢失目标，并基于其他无人机的目标估计进入协同再捕获，约 28.1 秒恢复本机视觉跟踪。

## ROS2 mock demo

安装 ROS2 Humble 后构建：

```bash
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

启动纯 ROS2 版本：

```bash
ros2 launch multi_uav_sim_bringup mock_demo.launch.py
```

启动 Gazebo Fortress + ROS2 mock 算法版本：

```bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py
```

默认是 headless/server-only：

```bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py headless:=true
```

需要打开 Gazebo GUI 时：

```bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py headless:=false
```

主要 topic：

```text
/target/ground_truth
/uav_1/pose2d
/uav_1/vision_observation
/uav_1/tracker_state
/swarm/target_estimate
/swarm/events
```

结果保存在：

```text
runs_ros2/<timestamp>_ros2_reacquire/
```

Gazebo 第一版使用 `gazebo_pose_sync` 节点通过 `ign service /world/multi_uav_reacquire/set_pose` 把当前 ROS2 mock 位姿同步到 Gazebo 模型。这个方案适合快速演示，后续会替换成 Gazebo 插件或标准 bridge 方式。

总体设计和开工文档见：

- [docs/gazebo_multi_uav_sim_design.md](docs/gazebo_multi_uav_sim_design.md)
