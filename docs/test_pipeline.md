# 测试 Pipeline

本文档从零开始列出当前项目的测试命令。默认环境：

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Fortress
conda env: uav_gazebo
project: /mnt/data2/lhm/uav/find_object_again
```

## 0. 进入项目

```bash
cd /mnt/data2/lhm/uav/find_object_again
pwd
```

## 1. 激活 conda 与 ROS2

如果 `conda activate` 可直接使用：

```bash
conda activate uav_gazebo
source /opt/ros/humble/setup.bash
```

如果 `conda activate` 不可用，先加载 conda 脚本：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate uav_gazebo
source /opt/ros/humble/setup.bash
```

如果你用的是 Anaconda：

```bash
source ~/anaconda3/etc/profile.d/conda.sh
conda activate uav_gazebo
source /opt/ros/humble/setup.bash
```

检查：

```bash
which python
python --version
which ros2
ros2 --help
```

## 2. 安装 Python 依赖

ROS2 相关包不要用 pip 安装。这里只安装算法、mock demo 和记录工具依赖：

```bash
cd /mnt/data2/lhm/uav/find_object_again
pip install -r requirements.txt
```

检查 Python 依赖：

```bash
python -c "import numpy, matplotlib, cv2; print('python deps ok')"
python -c "import rclpy; print('rclpy ok')"
```

如果 `import rclpy` 失败，重新按顺序执行：

```bash
conda activate uav_gazebo
source /opt/ros/humble/setup.bash
python -c "import rclpy; print('rclpy ok')"
```

## 3. 检查 Gazebo Fortress

Gazebo Fortress 使用 `ign` 命令，不是 `gz sim`。

```bash
which ign
ign gazebo --versions
ign gazebo --help
```

如果 `gz sim` 不存在但 `ign` 存在，这是正常的。

检查 bridge 包：

```bash
ros2 pkg list | grep -E "ros_gz|ros_ign"
```

如果没有 bridge，先尝试：

```bash
sudo apt install -y ros-humble-ros-gz
```

如果找不到包，再试：

```bash
sudo apt install -y ros-humble-ros-ign
```

## 4. 运行纯 Python mock demo

这一步不依赖 ROS2/Gazebo，用于验证算法闭环和视频输出。

```bash
cd /mnt/data2/lhm/uav/find_object_again
python run_mock_demo.py
```

预期输出类似：

```text
Run directory: runs/<timestamp>_mock_reacquire
Events: 3
Lost events: 1
Recovery successes: 1
Recovery failures: 0
```

查看结果：

```bash
ls -lh runs/*_mock_reacquire | tail
cat runs/*_mock_reacquire/metrics.json | tail -n 20
tail -n 20 runs/*_mock_reacquire/events.jsonl
```

如果服务器没有桌面，也可以只确认生成了视频文件：

```bash
find runs -name topdown.mp4 -printf "%p %k KB\n" | tail
```

## 5. 构建 ROS2 workspace

```bash
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

检查包是否可见：

```bash
ros2 pkg list | grep multi_uav
ros2 interface show multi_uav_sim_msgs/msg/VisionObservation
```

如果 conda 导致构建异常，使用系统 Python 重新构建：

```bash
conda deactivate
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 6. 运行 ROS2 mock demo

这个版本不启动 Gazebo，只测试 ROS2 topic、消息、状态机、记录器。

终端 1：

```bash
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch multi_uav_sim_bringup mock_demo.launch.py
```

终端 2 检查 topic：

```bash
source /opt/ros/humble/setup.bash
source /mnt/data2/lhm/uav/find_object_again/ros2_ws/install/setup.bash
ros2 topic list | grep -E "uav_|swarm|target"
ros2 topic echo /swarm/events
```

预期会看到类似事件：

```text
TARGET_LOST
COOP_REACQUIRE_START
TARGET_RECOVERED
```

默认优化后的演示应主要出现 `uav_1` 的指定丢失恢复：

```text
TARGET_LOST uav_1: OCCLUDED
COOP_REACQUIRE_START uav_1: using swarm target estimate
TARGET_RECOVERED uav_1: local vision reacquired target
```

如果仍然出现大量 `uav_2/uav_3 OUT_OF_FOV`，优先检查 `mock_demo.yaml` 中：

```text
camera_fov_deg: 105.0
camera_range: 45.0
enforce_support_visibility: true
support_uav_ids: [uav_2, uav_3]
```

停止终端 1 后查看记录：

```bash
cd /mnt/data2/lhm/uav/find_object_again
find runs_ros2 -maxdepth 2 -type f | sort | tail -n 20
cat runs_ros2/*_ros2_reacquire/metrics.json | tail -n 40
tail -n 20 runs_ros2/*_ros2_reacquire/events.jsonl
```

## 7. 运行 Gazebo + ROS2 mock demo

这个版本启动 Gazebo Fortress world，同时运行当前 ROS2 mock 算法。当前默认不启用 Gazebo 位姿同步，因为不同 Fortress 安装暴露的 set-pose 服务可能不同。

### 7.1 Headless 模式

适合服务器测试：

```bash
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py headless:=true
```

### 7.2 GUI 模式

如果服务器支持图形界面：

```bash
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py headless:=false
```

### 7.3 检查 Gazebo 服务

另开终端：

```bash
source /opt/ros/humble/setup.bash
source /mnt/data2/lhm/uav/find_object_again/ros2_ws/install/setup.bash
ign service -l | grep multi_uav_reacquire
ign service -l | grep set_pose
```

如果 launch 设置了 `IGN_PARTITION=multi_uav_reacquire`，另开终端查询 Ignition 服务时也要带同样的环境变量：

```bash
IGN_PARTITION=multi_uav_reacquire ign service -l | grep -E "pose|world|multi_uav"
IGN_PARTITION=multi_uav_reacquire ign topic -l
```

手动测试 Gazebo 是否接受按模型名设置位姿：

```bash
IGN_PARTITION=multi_uav_reacquire ign service \
  -s /world/multi_uav_reacquire/set_pose \
  --reqtype ignition.msgs.Pose \
  --reptype ignition.msgs.Boolean \
  --timeout 1000 \
  --req 'name: "target" position { x: 0 y: 0 z: 0 } orientation { w: 1 x: 0 y: 0 z: 0 }'
```

如果这个命令返回 false 或报类型错误，需要改 `gazebo_pose_sync` 的请求格式。

检查 ROS2 事件：

```bash
ros2 topic echo /swarm/events
```

`/swarm/events` 是事件流，不是连续流。如果启动 echo 时事件已经过去，它会一直等待。可以改看连续流：

```bash
ros2 topic echo --once /uav_1/vision_observation
ros2 topic echo --once /uav_1/tracker_state
ros2 topic echo --once /swarm/target_estimate
```

如果 Gazebo GUI 或 server 打印大量 `Host unreachable`，先禁用 pose sync 判断是不是同步服务调用导致：

```bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py headless:=false pose_sync:=false
```

如果禁用后不再刷 `Host unreachable`，说明 Gazebo 本体能启动，问题集中在 `/world/multi_uav_reacquire/set_pose` 服务调用或 Ignition Transport 网络配置。

当前 launch 默认就是：

```bash
pose_sync:=false
```

只有当下面命令能看到 set-pose 相关服务时，才建议开启：

```bash
ign service -l | grep set_pose
```

检查模型同步节点日志。如果看到 `/world/multi_uav_reacquire/set_pose` 相关错误，把完整报错贴出来。

## 8. 单独启动 Gazebo world

用于排查 world/model 是否能加载。

```bash
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=/mnt/data2/lhm/uav/find_object_again/ros2_ws/install/multi_uav_sim_worlds/share/multi_uav_sim_worlds/models:$IGN_GAZEBO_RESOURCE_PATH
ign gazebo -s -r /mnt/data2/lhm/uav/find_object_again/ros2_ws/install/multi_uav_sim_worlds/share/multi_uav_sim_worlds/worlds/multi_uav_reacquire.sdf
```

GUI：

```bash
ign gazebo /mnt/data2/lhm/uav/find_object_again/ros2_ws/install/multi_uav_sim_worlds/share/multi_uav_sim_worlds/worlds/multi_uav_reacquire.sdf
```

## 9. 保存 Gazebo 第一视角相机视频

当前提供了一个独立 launch，用于启动 Gazebo camera sensor、bridge 和本地 mp4 保存：

```bash
cd /home/kemove/lhm/uav/uav_find_again/ros2_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch multi_uav_sim_bringup gazebo_camera_record.launch.py headless:=false
```

如果你的 bridge 包是 `ros_gz_bridge`，改成：

```bash
ros2 launch multi_uav_sim_bringup gazebo_camera_record.launch.py headless:=false bridge_pkg:=ros_gz_bridge
```

保存目录：

```text
runs_ros2/gazebo_camera_record/videos/
  uav_1_front_camera.mp4
  uav_2_front_camera.mp4
  uav_3_front_camera.mp4
```

检查 ROS2 image topic：

```bash
ros2 topic list | grep front_camera
ros2 topic hz /uav_1/front_camera/image
ros2 topic echo --once /uav_1/front_camera/image
```

如果没有 image topic，先查 Gazebo 原始 topic：

```bash
IGN_PARTITION=multi_uav_reacquire ign topic -l | grep -E "camera|image|imu"
```

当前 launch 里默认假设 Gazebo topic 是：

```text
/world/multi_uav_reacquire/model/uav_1/link/base_link/sensor/front_camera/image
/world/multi_uav_reacquire/model/uav_2/link/base_link/sensor/front_camera/image
/world/multi_uav_reacquire/model/uav_3/link/base_link/sensor/front_camera/image
```

如果 `ign topic -l` 显示的名称不同，需要更新：

```text
ros2_ws/src/multi_uav_sim_bringup/launch/gazebo_camera_record.launch.py
```

## 10. 常用排查命令

查看构建错误：

```bash
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
colcon build --symlink-install --event-handlers console_direct+
```

查看 ROS2 包路径：

```bash
ros2 pkg prefix multi_uav_sim_bringup
ros2 pkg prefix multi_uav_sim_worlds
ros2 pkg prefix multi_uav_sim_nodes
```

查看 launch 参数：

```bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py --show-args
```

查看 topic 频率：

```bash
ros2 topic hz /uav_1/pose2d
ros2 topic hz /uav_1/vision_observation
ros2 topic hz /swarm/target_estimate
```

查看一帧消息：

```bash
ros2 topic echo --once /uav_1/tracker_state
ros2 topic echo --once /swarm/target_estimate
```

查看 Gazebo 服务：

```bash
ign service -l
ign service -l | grep set_pose
```

## 11. 推荐测试顺序

按下面顺序测，出错时更容易定位：

```text
1. python run_mock_demo.py
2. colcon build --symlink-install
3. ros2 launch multi_uav_sim_bringup mock_demo.launch.py
4. ign gazebo 单独加载 world
5. ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py headless:=true
6. ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py headless:=false
7. ros2 launch multi_uav_sim_bringup gazebo_camera_record.launch.py headless:=false
```

如果第 1 步失败，是 Python/conda 依赖问题。

如果第 2 步失败，是 ROS2 包或依赖问题。

如果第 3 步失败，是 ROS2 节点/topic/消息问题。

如果第 4 步失败，是 Gazebo world/model 路径问题。

如果第 5/6 步失败，是 Gazebo 与 ROS2 联合启动或位姿同步问题。
