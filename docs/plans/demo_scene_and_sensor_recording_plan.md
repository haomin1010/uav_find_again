# 演示场景优化与传感器录制方案

## 1. 目标

当前系统已经能跑通 ROS2 mock 算法和 Gazebo 场景，但演示还不够稳定：

- 部分无人机会在早期因为 `OUT_OF_FOV` 触发额外丢失事件。
- Gazebo 中模型主要是静态场景 + ROS2 算法链路，传感器数据还没有形成标准录制流程。
- 还没有保存每架无人机第一视角相机视频、IMU 等传感器数据。

下一阶段目标：

```text
1. 无人机始终处于飞行状态，不停在原地。
2. 演示事件收敛为一个清晰主线：
   uav_1 视觉丢失 -> 协同再捕获 -> 恢复跟随。
3. Gazebo 中每架无人机带相机和 IMU。
4. 本地保存：
   - 每架无人机第一视角视频
   - IMU 数据
   - 位姿/轨迹数据
   - 视觉观测、跟踪状态、事件日志
5. 支持 headless 服务器运行。
```

## 2. 复杂度评估

这部分复杂度中等，建议分阶段做。

不复杂的部分：

- 让无人机持续飞行。
- 调整初始位置、目标轨迹和视场，减少非预期丢失。
- 保存 ROS2 topic 到 rosbag。
- 保存当前 mock 算法的事件和轨迹。

中等复杂的部分：

- 给 Gazebo SDF 模型加相机和 IMU sensor。
- 用 bridge 把 Gazebo sensor topic 接到 ROS2。
- 写节点把 ROS2 image topic 保存成 mp4。
- 确保 headless 下相机仍能渲染。

相对复杂的部分：

- 真正让 Gazebo 中的无人机模型按 ROS2 算法轨迹稳定运动。
- 高质量同步 Gazebo 模型位姿、传感器输出和 ROS2 算法状态。
- 多机多相机视频同时保存，避免帧率、编码、时间戳混乱。

建议策略：

```text
先保证演示逻辑和记录链路稳定，
再提升 Gazebo 传感器真实度。
```

## 3. 演示场景设计

### 3.1 场景主线

推荐演示脚本：

```text
0-10s:
  三架无人机形成稳定编队，目标沿预设路线移动。

10-18s:
  无人机持续跟随，所有无人机都能观测目标。

18-28s:
  uav_1 进入指定遮挡/强制丢失窗口。
  uav_2/uav_3 保持观测。
  uav_1 进入 REACQUIRING。

28-35s:
  uav_1 重新看到目标，进入 RECOVERED。

35s 后:
  编队继续飞行，证明恢复后系统没有停止。
```

### 3.2 无人机持续飞行策略

当前控制逻辑可以升级为：

```text
TRACKING:
  围绕目标保持编队偏移点，而不是直接停在一个位置。

REACQUIRING:
  朝 swarm_target_estimate 附近移动，同时做小幅搜索轨迹。

RECOVERED:
  平滑回到编队位置。
```

推荐第一版控制策略：

```text
desired_position = target_estimate + formation_offset + orbit_offset
```

其中：

```text
formation_offset:
  每架无人机的固定编队偏移。

orbit_offset:
  小幅周期运动，让无人机始终在飞。
```

示例：

```text
uav_1: 目标后左侧
uav_2: 目标正后方
uav_3: 目标后右侧
```

### 3.3 减少非预期丢失事件

当前出现 `uav_2/uav_3 OUT_OF_FOV`，说明视场/初始朝向/跟随位置还不够稳。

优化方向：

- 增大第一版 mock camera FOV，例如从 78 deg 改到 95 deg。
- 增大 camera range，例如从 34 m 改到 45 m。
- 调整无人机初始位置，让三架无人机一开始都朝向目标。
- 跟随控制始终让 yaw 指向目标估计。
- 只对 `uav_1` 施加 scripted loss，避免其他无人机触发随机丢失。
- 对 `uav_2/uav_3` 设置更保守的编队位置，保证它们一直能看到目标。

目标：

```text
默认演示只出现一个主事件：
uav_1 OCCLUDED -> COOP_REACQUIRE_START -> TARGET_RECOVERED
```

## 4. Gazebo 传感器设计

### 4.1 每架无人机传感器

第一阶段建议每架无人机包含：

```text
RGB camera
IMU
ground truth pose
```

可选后续扩展：

```text
depth camera
segmentation camera
bounding box camera
lidar
```

### 4.2 Gazebo SDF sensor

`simple_uav/model.sdf` 中为 `base_link` 增加：

```text
camera sensor:
  name: front_camera
  update_rate: 20 Hz
  resolution: 640 x 480
  horizontal_fov: 1.396 rad
  topic: /world/multi_uav_reacquire/model/<uav_name>/camera

imu sensor:
  name: imu
  update_rate: 100 Hz
  topic: /world/multi_uav_reacquire/model/<uav_name>/imu
```

注意：

```text
如果 simple_uav 被 include 三次，sensor topic 可能带 model instance name。
需要实际用 ign topic -l 确认 topic 名称。
```

### 4.3 Headless 相机渲染风险

Gazebo headless 下 RGB 相机可能依赖 GPU/EGL 环境。

当前服务器已有：

```text
libEGL warning: egl: failed to create dri2 screen
```

如果 GUI 和渲染正常，这个 warning 可以先忽略。

如果相机图像为空或无法生成，需要再排查：

```bash
nvidia-smi
glxinfo | grep "OpenGL renderer"
echo $DISPLAY
echo $XDG_SESSION_TYPE
```

## 5. 数据录制设计

### 5.1 推荐输出目录

每次运行保存到：

```text
runs_ros2/<timestamp>_gazebo_reacquire/
  events.jsonl
  tracks.csv
  metrics.json
  rosbag/
  videos/
    uav_1_front_camera.mp4
    uav_2_front_camera.mp4
    uav_3_front_camera.mp4
  sensors/
    uav_1_imu.csv
    uav_2_imu.csv
    uav_3_imu.csv
```

### 5.2 rosbag 录制

第一阶段优先保存 rosbag：

```bash
ros2 bag record \
  /target/ground_truth \
  /uav_1/pose2d \
  /uav_2/pose2d \
  /uav_3/pose2d \
  /uav_1/vision_observation \
  /uav_2/vision_observation \
  /uav_3/vision_observation \
  /uav_1/tracker_state \
  /uav_2/tracker_state \
  /uav_3/tracker_state \
  /swarm/target_estimate \
  /swarm/events
```

加入 Gazebo sensor bridge 后，再增加：

```text
/uav_1/front_camera/image
/uav_2/front_camera/image
/uav_3/front_camera/image
/uav_1/imu
/uav_2/imu
/uav_3/imu
```

### 5.3 视频保存

推荐实现一个 `image_video_recorder` ROS2 节点：

输入：

```text
/uav_i/front_camera/image
```

输出：

```text
runs_ros2/<timestamp>/videos/uav_i_front_camera.mp4
```

依赖：

```text
cv_bridge
opencv-python-headless
sensor_msgs
```

如果暂时没有 `cv_bridge`，可以先用 rosbag 保存 image topic，后处理成 mp4。

### 5.4 IMU 保存

推荐实现一个 `sensor_csv_recorder` 节点：

输入：

```text
/uav_i/imu
```

输出：

```text
timestamp
orientation
angular_velocity
linear_acceleration
```

保存为：

```text
sensors/uav_i_imu.csv
```

## 6. ROS-Gazebo bridge 方案

根据你当前 Gazebo Fortress 环境，bridge 包可能是：

```text
ros-humble-ros-gz
或
ros-humble-ros-ign
```

实际确认：

```bash
ros2 pkg list | grep -E "ros_gz|ros_ign"
```

如果使用 `ros_ign_bridge`，典型 bridge 命令形态：

```bash
ros2 run ros_ign_bridge parameter_bridge \
  /topic_name@sensor_msgs/msg/Image@ignition.msgs.Image
```

如果使用 `ros_gz_bridge`，命令类似但包名为：

```bash
ros2 run ros_gz_bridge parameter_bridge ...
```

具体 topic 名称必须以实际输出为准：

```bash
ign topic -l
```

## 7. 代码改造计划

### Milestone A: 演示场景收敛

目标：

```text
无人机持续飞行。
默认只触发 uav_1 的指定丢失恢复事件。
```

改动：

- 修改 `uav_control.py`，加入 orbit_offset。
- 修改 `mock_demo.yaml`，增大 FOV/range。
- 修改 `vision_mock.py`，支持 per-uav scripted loss。
- 修改 `scenario.py`，调整目标轨迹和无人机初始位置。

验收：

```text
运行 mock_demo.launch.py。
事件日志主要出现 uav_1 的 OCCLUDED/REACQUIRE/RECOVERED。
```

当前已实现：

```text
uav_control:
  增加 orbit_radius 和 orbit_period。
  无人机跟随目标编队点时叠加小幅周期运动，使无人机持续飞行。

vision_mock:
  增加 scripted_loss_uav/scripted_loss_start_sec/scripted_loss_end_sec。
  默认只让 uav_1 在 18-28 秒进入 OCCLUDED。
  增加 enforce_support_visibility 和 support_uav_ids。
  默认保护 uav_2/uav_3 的支持观测，减少非预期 OUT_OF_FOV。

mock_demo.yaml:
  camera_range: 45.0
  camera_fov_deg: 105.0
  orbit_radius: 1.8
  orbit_period: 16.0
```

测试命令：

```bash
cd /home/kemove/lhm/uav/uav_find_again/ros2_ws
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select multi_uav_sim_nodes multi_uav_sim_bringup
source install/setup.zsh
ros2 launch multi_uav_sim_bringup mock_demo.launch.py
```

### Milestone B: Gazebo 传感器模型

目标：

```text
每架 simple_uav 带 camera 和 IMU。
ign topic -l 能看到 sensor topic。
```

改动：

- 修改 `simple_uav/model.sdf`。
- 必要时拆分三个 UAV 模型实例，给每个传感器明确 topic。
- 更新 `multi_uav_reacquire.sdf`。

验收：

```bash
ign topic -l | grep camera
ign topic -l | grep imu
```

### Milestone C: Bridge 到 ROS2

目标：

```text
ROS2 能看到 camera image 和 imu topic。
```

改动：

- 新增 launch：`gazebo_sensor_demo.launch.py` 或扩展现有 launch。
- 启动 `ros_ign_bridge` 或 `ros_gz_bridge`。

验收：

```bash
ros2 topic list | grep camera
ros2 topic list | grep imu
ros2 topic echo --once /uav_1/imu
```

### Milestone D: 本地保存视频与传感器

目标：

```text
保存每架无人机第一视角视频。
保存 IMU CSV。
保存 rosbag。
```

改动：

- 新增 `image_video_recorder.py`。
- 新增 `sensor_csv_recorder.py`。
- 扩展 `sim_recorder.py` 或新建统一 recorder launch。

验收：

```text
runs_ros2/<timestamp>/videos/*.mp4
runs_ros2/<timestamp>/sensors/*.csv
runs_ros2/<timestamp>/rosbag/
```

## 8. 推荐执行顺序

建议按下面顺序做：

```text
1. 先优化当前 ROS2 mock 演示事件。
2. 再给 Gazebo 模型加 camera/IMU。
3. 再确认 ign topic 能看到传感器。
4. 再做 bridge 到 ROS2。
5. 最后写 video/imu recorder。
```

不要一开始就同时做 Gazebo 运动同步、真实相机、bridge、视频保存和 IMU 保存。这样问题定位会很困难。

## 9. 当前决策

当前最优路线：

```text
短期:
  Gazebo 作为可视化场景 + ROS2 mock 算法稳定演示。

中期:
  Gazebo 提供 camera/IMU sensor topic，并保存本地数据。

后期:
  用 Gazebo sensor 替换 mock vision，进一步接真实检测器或 PX4。
```

优先级：

```text
P0: 演示事件稳定、无人机持续飞行。
P1: 保存 rosbag + 事件/轨迹/指标。
P2: Gazebo camera/IMU topic。
P3: 视频 mp4 与 IMU CSV 自动保存。
P4: 用真实图像检测替代 mock vision。
```
