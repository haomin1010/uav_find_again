# Gazebo 多无人机协同视觉跟踪仿真环境总体设计

## 1. 目标

本项目目标是构建一套易于在服务器上运行和演示的 Gazebo 仿真环境，用于验证多无人机协同跟随目标、视觉丢失检测、以及基于其他无人机视觉信息的目标再捕获算法。

第一阶段不追求完整真实飞控和真实视觉模型，而是优先搭建稳定、可控、可复现实验环境：

- 多架无人机在同一场地中运动。
- 场地中有一个可移动目标。
- 每架无人机拥有相机视场和视觉观测结果。
- 当某架无人机目标丢失时，可以接收其他无人机的目标观测信息。
- 丢失无人机根据协同信息调整搜索或跟随策略，并恢复目标观测。
- 支持服务器无界面运行，并保存演示视频、轨迹和日志。

核心演示闭环：

```text
目标移动
  -> 多无人机跟随
  -> 某架无人机因遮挡/视场外/噪声丢失目标
  -> 其他无人机继续观测目标并广播信息
  -> 丢失无人机融合协同信息
  -> 调整航向/位置
  -> 恢复观测并重新跟随
```

## 2. 技术选型

### 2.1 推荐基础栈

推荐使用：

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Fortress, using ign gazebo command
Python 3.10+
OpenCV
NumPy
Matplotlib
```

如果后续需要接入真实飞控链路，再加入：

```text
PX4 SITL
Micro XRCE-DDS Agent
px4_msgs
```

第一阶段建议不接 PX4，直接用自定义 ROS2 控制节点控制无人机模型位姿或速度。这样可以避免飞控、混控器、模型参数、DDS 通信等工程问题过早干扰算法验证。

### 2.2 Gazebo 在本项目中的职责

Gazebo 负责：

- 3D 场景加载。
- 多无人机模型和目标模型显示。
- 相机、深度、分割或 bounding box 类传感器扩展。
- 遮挡、视场、距离等几何关系。
- 后台仿真运行。

ROS2 负责：

- 多机状态管理。
- 目标轨迹生成。
- mock 视觉观测生成。
- 视觉丢失判断。
- 多机通信和观测融合。
- 恢复跟随控制。
- 日志和视频录制。

算法节点不应强依赖 Gazebo 内部 API。Gazebo 只作为仿真数据源和可视化环境，算法通过 ROS2 topic/service/action 交互。

## 3. 总体架构

### 3.1 模块划分

建议代码按 ROS2 workspace 组织：

```text
find_object_again/
  README.md
  docs/
    gazebo_multi_uav_sim_design.md
  ros2_ws/
    src/
      multi_uav_sim_bringup/
      multi_uav_sim_description/
      multi_uav_sim_worlds/
      multi_uav_sim_msgs/
      multi_uav_target_manager/
      multi_uav_vision_mock/
      multi_uav_coop_tracker/
      multi_uav_control/
      multi_uav_recorder/
```

各包职责如下：

```text
multi_uav_sim_bringup
  启动文件、参数文件、实验配置。

multi_uav_sim_description
  无人机模型、相机挂载、目标模型、SDF/URDF/Xacro。

multi_uav_sim_worlds
  Gazebo world 文件、场地、障碍物、遮挡物、演示场景。

multi_uav_sim_msgs
  自定义消息，例如视觉观测、跟踪状态、协同估计。

multi_uav_target_manager
  目标运动轨迹生成和目标 ground truth 发布。

multi_uav_vision_mock
  根据无人机位姿、相机视场、遮挡和噪声生成视觉观测。

multi_uav_coop_tracker
  每架无人机的本地目标状态机、协同观测融合、丢失恢复策略。

multi_uav_control
  无人机运动控制。第一阶段使用简化运动学控制。

multi_uav_recorder
  视频、轨迹、事件日志、指标保存。
```

### 3.2 数据流

推荐第一阶段数据流：

```text
Gazebo /tf /model_states
  -> target_manager 发布目标真实状态
  -> vision_mock 生成每架无人机视觉观测
  -> coop_tracker 判断 tracking/lost/reacquiring 状态
  -> control 生成无人机速度/位置指令
  -> Gazebo 更新无人机位姿
  -> recorder 保存视频、轨迹、事件和指标
```

逻辑上每架无人机都有独立状态：

```text
uav_i/pose
uav_i/camera_info
uav_i/vision_observation
uav_i/tracker_state
uav_i/control_cmd
```

集群共享信息：

```text
/swarm/target_observations
/swarm/target_estimate
/swarm/events
```

## 4. 仿真建模

### 4.1 场地

第一阶段场地建议简单但包含关键干扰：

- 平面地面。
- 场地边界。
- 2 到 5 个障碍物。
- 至少一个可造成遮挡的墙体或柱体。
- 一个目标移动区域。

示例场景：

```text
60m x 60m 平面场地
3 架无人机
1 个移动目标
若干矩形障碍物
目标以 waypoint 或随机游走方式移动
```

### 4.2 无人机模型

第一阶段无需精细气动模型。无人机可抽象为：

```text
position: x, y, z
yaw
velocity limit
acceleration limit
camera mount pose
field of view
```

Gazebo 中可以使用简单 quadrotor 外观模型，控制层直接发布位姿或速度指令。

推荐约束：

```text
最大水平速度: 3.0 m/s
最大垂直速度: 1.0 m/s
巡航高度: 6 到 12 m
相机 pitch: -20 到 -45 deg
水平 FOV: 70 到 90 deg
垂直 FOV: 45 到 60 deg
```

### 4.3 目标模型

目标可以是一个人形或车辆模型。第一阶段只要求有明确 ground truth：

```text
target_id
position
velocity
heading
bounding geometry
```

目标运动模式：

```text
waypoint_loop
random_walk
scripted_occlusion
scripted_escape
```

其中 `scripted_occlusion` 用于稳定复现实验：让目标在指定时间穿过遮挡区，使某些无人机丢失目标。

## 5. 视觉观测设计

### 5.1 第一阶段：mock 视觉观测

第一阶段不直接依赖真实图像检测器。`vision_mock` 根据几何关系生成观测：

```text
输入:
  uav pose
  camera pose
  camera fov
  target ground truth
  obstacle geometry
  noise config

输出:
  detected
  confidence
  bbox
  bearing
  elevation
  range_estimate
  target_position_estimate
  covariance
  loss_reason
```

建议定义丢失原因：

```text
OUT_OF_FOV
OCCLUDED
TOO_FAR
LOW_CONFIDENCE
RANDOM_DROPOUT
```

这样演示时可以明确说明“为什么丢失”和“如何恢复”。

### 5.2 第二阶段：Gazebo 相机图像

第二阶段引入 Gazebo RGB 相机：

```text
Gazebo camera topic
  -> ROS2 image topic
  -> detector/tracker
  -> vision_observation
```

检测器可以替换 `vision_mock`：

```text
YOLO / RT-DETR / ByteTrack / DeepSORT
```

但协同跟踪算法仍然只依赖统一的 `VisionObservation` 消息，不直接依赖检测器实现。

### 5.3 第三阶段：半真实视觉

也可以使用 Gazebo segmentation 或 bounding box 类传感器输出更干净的目标框，再人工加入噪声和漏检。这比真实 detector 更稳定，比纯 mock 更接近视觉链路。

## 6. 协同跟踪与再捕获算法接口

### 6.1 状态机

每架无人机维护本地跟踪状态：

```text
INIT
TRACKING
LOST
REACQUIRING
RECOVERED
FAILED
```

状态转移：

```text
INIT -> TRACKING
  本机检测到目标。

TRACKING -> LOST
  连续 N 帧未检测到目标，或 confidence 低于阈值。

LOST -> REACQUIRING
  收到其他无人机可用观测，生成目标搜索区域。

REACQUIRING -> RECOVERED
  本机重新检测到目标。

RECOVERED -> TRACKING
  连续 M 帧稳定检测。

REACQUIRING -> FAILED
  超过最大恢复时间仍未检测到目标。
```

### 6.2 协同信息

其他无人机广播的观测可以分层设计：

```text
Level 1: target_position_estimate in world frame
Level 2: bearing/elevation + observer pose
Level 3: bbox + camera info + observer pose
Level 4: raw image or cropped image
```

第一阶段推荐使用 Level 1 和 Level 2：

```text
observer_id
timestamp
observer_pose
detected
confidence
target_position_estimate
target_position_covariance
bearing
elevation
range_estimate
```

### 6.3 融合策略

第一阶段使用轻量融合即可：

```text
1. 过滤低 confidence 或过期观测。
2. 按 confidence 和 covariance 加权。
3. 得到 swarm_target_estimate。
4. 丢失无人机朝估计目标位置或搜索区域移动。
```

后续可升级为：

```text
EKF / UKF
particle filter
factor graph
multi-hypothesis tracking
```

### 6.4 再捕获控制

丢失无人机的恢复动作不要直接冲向目标点，建议使用搜索区域：

```text
target_estimate
uncertainty_radius
preferred_altitude
search_orbit_radius
camera_yaw_command
```

可选策略：

```text
direct_intercept
  直接朝融合目标位置移动。

orbit_search
  围绕目标估计位置做小半径环绕搜索。

front_intercept
  根据目标速度预测前方拦截点。

viewpoint_reassignment
  根据其他无人机位置选择更互补的视角。
```

第一阶段建议实现：

```text
TRACKING: 保持目标在相机中心附近。
LOST: 原地小范围 yaw scan。
REACQUIRING: 朝 swarm_target_estimate 移动，并保持相机朝向估计位置。
RECOVERED: 切回本机视觉闭环跟随。
```

## 7. ROS2 消息建议

### 7.1 VisionObservation.msg

```text
std_msgs/Header header
string observer_id
bool detected
float32 confidence
string loss_reason
geometry_msgs/Pose observer_pose
geometry_msgs/Point target_position_estimate
float32[9] target_position_covariance
float32 bearing
float32 elevation
float32 range_estimate
float32 bbox_cx
float32 bbox_cy
float32 bbox_w
float32 bbox_h
```

### 7.2 TrackerState.msg

```text
std_msgs/Header header
string uav_id
string state
bool local_detected
bool coop_available
float32 local_confidence
geometry_msgs/Point current_target_estimate
float32 uncertainty_radius
string reason
```

### 7.3 SwarmTargetEstimate.msg

```text
std_msgs/Header header
bool valid
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32[9] covariance
string[] contributing_uav_ids
float32 confidence
```

### 7.4 SimEvent.msg

```text
std_msgs/Header header
string event_type
string uav_id
string description
```

事件类型：

```text
TARGET_LOST
COOP_REACQUIRE_START
TARGET_RECOVERED
RECOVERY_FAILED
OCCLUSION_ENTER
OCCLUSION_EXIT
```

## 8. 视频与日志保存

### 8.1 Headless 运行

服务器上推荐无 GUI 启动 Gazebo：

```bash
ign gazebo -s -r path/to/world.sdf
```

上面命令适用于 Gazebo Fortress。Garden/Harmonic 等更新版本通常使用：

```bash
gz sim -s -r path/to/world.sdf
```

如果需要后台 RGB 相机渲染，服务器需要可用的 GPU 渲染环境。根据机器环境可能需要配置 EGL、NVIDIA 驱动或虚拟显示。

### 8.2 录制内容

建议保存四类结果：

```text
1. 全局俯视演示视频
   显示目标、多无人机、视场锥、障碍物、状态颜色。

2. 每架无人机第一视角视频
   来自 Gazebo camera topic 或 mock 渲染。

3. 结构化日志
   CSV/JSONL，记录每帧位姿、观测、状态和控制指令。

4. 指标汇总
   丢失次数、恢复成功率、恢复耗时、平均目标误差。
```

### 8.3 推荐 recorder 实现

第一阶段 recorder 可以不依赖 Gazebo GUI，直接用 Python 生成俯视视频：

```text
订阅:
  /target/ground_truth
  /uav_*/pose
  /uav_*/vision_observation
  /uav_*/tracker_state
  /swarm/target_estimate

输出:
  runs/<timestamp>/topdown.mp4
  runs/<timestamp>/events.jsonl
  runs/<timestamp>/metrics.json
  runs/<timestamp>/tracks.csv
```

视频生成方式：

```text
Matplotlib animation
或 OpenCV canvas drawing
```

后续如果使用 Gazebo RGB 相机：

```text
ROS2 image topic
  -> cv_bridge
  -> OpenCV VideoWriter
  -> uav_1_camera.mp4
```

## 9. 环境安装

以下以 Ubuntu 22.04 + ROS 2 Humble 为基线。这个组合适合服务器开发，系统包稳定，后续接 PX4 也比较方便。

需要特别注意 Gazebo 版本：

```text
推荐第一版:
  Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress

说明:
  对 Humble 来说，最稳妥的 Gazebo 版本是 Fortress。
  Fortress 的命令是 ign gazebo，不是 gz sim。
  如果 apt install gz-fortress 后 which ign 显示 /usr/bin/ign，这是正常状态。

不建议第一版直接强行组合:
  ROS 2 Humble + Gazebo Harmonic

原因:
  Harmonic 可以装在 Ubuntu 22.04 上，但它不是 Humble 的默认推荐组合。
  这种组合能做，但 ros_gz、Gazebo ABI、插件和 CMake 依赖更容易踩坑。
```

### 9.1 基础系统包

```bash
sudo apt update
sudo apt install -y \
  curl \
  gnupg \
  lsb-release \
  locales \
  software-properties-common \
  build-essential \
  cmake \
  git \
  wget \
  tmux \
  htop \
  ffmpeg \
  python3-pip \
  python3-venv \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

设置 locale：

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 9.2 安装 ROS 2 Humble

添加 ROS2 apt 源：

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

安装 ROS2：

```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros-base \
  ros-dev-tools
```

确认 ROS2 可用：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
ros2 pkg list | head
```

建议不要把 `source /opt/ros/humble/setup.bash` 直接写进 `~/.bashrc`，因为本项目会配合 conda 使用。推荐用后面的 `env.sh` 显式激活环境。

### 9.3 初始化 rosdep

第一次安装 ROS 后执行：

```bash
sudo rosdep init
rosdep update
```

如果 `sudo rosdep init` 提示已经存在，可以跳过：

```bash
rosdep update
```

### 9.4 安装 Gazebo Fortress 与 ROS-Gazebo bridge

推荐第一版使用 Gazebo Fortress：

```bash
sudo apt update
sudo apt install -y gz-fortress
```

Gazebo Fortress 使用 `ign` 命令。确认方式：

```bash
which ign
ign gazebo --versions
ign gazebo --help
```

如果 `which ign` 显示 `/usr/bin/ign`，但 `gz sim` 不存在，这是正常的。`gz sim` 是 Garden/Harmonic 等更新版本常见的命令风格。

检查已安装包：

```bash
dpkg -l | grep -E "fortress|ignition|gazebo|ros-humble-ros-gz|ros-humble-ros-ign"
```

安装 ROS2 与 Gazebo 的 bridge。优先尝试：

```bash
sudo apt install -y ros-humble-ros-gz
```

如果当前源中没有 `ros-humble-ros-gz`，使用 Humble/Fortress 常见的旧包名：

```bash
sudo apt install -y ros-humble-ros-ign
```

确认 bridge 包：

```bash
ros2 pkg list | grep -E "ros_gz|ros_ign"
```

如果后续明确需要 Gazebo Harmonic，可以单独安装 OSRF 源：

```bash
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic
```

但第一版不建议这样做。Humble + Gazebo Fortress 的组合更稳。

Harmonic 安装后通常使用：

```bash
gz sim --versions
gz sim --help
```

### 9.5 创建 conda 环境

推荐使用 Miniconda 或 Mambaforge。假设 conda 已安装，创建本项目环境：

```bash
conda create -y -n uav_gazebo python=3.10
conda activate uav_gazebo
python --version
```

安装第一阶段算法依赖：

```bash
cd /mnt/data2/lhm/uav/find_object_again
pip install --upgrade pip==24.0 setuptools==69.5.1 wheel==0.43.0
pip install -r requirements.txt
```

如果服务器需要显示图像窗口，可以把 `opencv-python-headless` 换成：

```bash
pip uninstall -y opencv-python-headless
pip install opencv-python==4.8.1.78
```

不要用 pip 安装 ROS2 相关包：

```text
rclpy
geometry_msgs
std_msgs
launch_ros
ament_index_python
ros_gz
ros_ign
```

这些必须来自 apt 安装的 `/opt/ros/humble`。如果 pip/conda 中混入同名或类似包，容易导致 `colcon build`、`ros2 launch` 或 Python 节点导入异常。

如果你之前手动装依赖导致冲突，建议在 conda 环境里清理后重装：

```bash
conda activate uav_gazebo
pip uninstall -y \
  numpy \
  scipy \
  pandas \
  PyYAML \
  matplotlib \
  opencv-python \
  opencv-python-headless \
  rclpy \
  geometry_msgs \
  std_msgs \
  launch_ros \
  ament_index_python
pip install -r /mnt/data2/lhm/uav/find_object_again/requirements.txt
```

### 9.6 conda 与 ROS2 的冲突和解决方式

ROS2 Humble 的 apt 包默认是按系统 Python 3.10 构建的。conda 激活后会覆盖：

```text
python
pip
PYTHONPATH
LD_LIBRARY_PATH
CMAKE_PREFIX_PATH
AMENT_PREFIX_PATH
```

常见问题：

```text
1. conda 激活后 import rclpy 失败。
2. colcon build 找不到 ament_cmake 或 rosidl。
3. ros2 命令存在，但 Python 节点启动时报 ModuleNotFoundError。
4. OpenCV、libstdc++、Qt、OpenGL 库被 conda 版本抢先加载。
```

推荐规则：

```text
先 activate conda，再 source ROS2。
不要先 source ROS2 再 activate conda。
```

正确顺序：

```bash
conda activate uav_gazebo
source /opt/ros/humble/setup.bash
```

如果已经构建过 workspace：

```bash
conda activate uav_gazebo
source /opt/ros/humble/setup.bash
source /mnt/data2/lhm/uav/find_object_again/ros2_ws/install/setup.bash
```

检查当前 Python 和 ROS2 Python 包：

```bash
which python
python -c "import sys; print(sys.executable); print(sys.version)"
python -c "import rclpy; print('rclpy ok')"
python -c "import numpy, cv2, matplotlib, pandas, yaml; print('python deps ok')"
```

如果 `import rclpy` 失败，先看 ROS2 的 Python 路径是否在 `PYTHONPATH` 中：

```bash
echo "$PYTHONPATH" | tr ':' '\n' | grep humble
```

如果没有，重新按顺序执行：

```bash
conda activate uav_gazebo
source /opt/ros/humble/setup.bash
```

如果仍然失败，可以临时补系统 Python dist-packages：

```bash
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
python -c "import rclpy; print('rclpy ok')"
```

如果出现 `GLIBCXX`、`libstdc++`、`libffi` 相关错误，通常是 conda 动态库覆盖系统库。优先尝试：

```bash
conda install -y -c conda-forge libstdcxx-ng libgcc-ng libffi
```

如果 Gazebo 或 ROS2 仍被 conda 动态库影响，运行 Gazebo 时可以临时停用 conda：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
ign gazebo --versions
```

更稳的工程拆分方式：

```text
Gazebo server / ros_gz or ros_ign / ROS2 基础节点:
  使用系统 ROS 环境运行。

算法节点 / 视觉模型 / recorder:
  使用 conda 环境运行。

两边通过 ROS2 topic 通信。
```

第一阶段为了开发方便，可以所有 Python 节点都在 conda 中运行；如果遇到库冲突，再拆成两个终端。

### 9.7 推荐环境脚本

建议创建：

```text
scripts/env.sh
```

内容：

```bash
#!/usr/bin/env bash
set -e

if command -v conda >/dev/null 2>&1; then
  :
elif [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
  source "$HOME/anaconda3/etc/profile.d/conda.sh"
fi

conda activate uav_gazebo
source /opt/ros/humble/setup.bash

WS_DIR=/mnt/data2/lhm/uav/find_object_again/ros2_ws
if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
fi

export RCUTILS_COLORIZED_OUTPUT=1
export ROS_DOMAIN_ID=42
```

使用方式：

```bash
source scripts/env.sh
```

注意：`conda activate` 在非交互 shell 中可能不可用。如果遇到这个问题，先执行：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate uav_gazebo
```

如果你的 conda 安装在其他路径，把 `~/miniconda3` 改成实际路径。

### 9.8 安装项目依赖

workspace 创建后执行：

```bash
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
conda activate uav_gazebo
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
```

如果 `src` 目录还不存在，先跳过这一步。

### 9.9 最小环境自检

```bash
conda activate uav_gazebo
source /opt/ros/humble/setup.bash

python -c "import rclpy; print('rclpy ok')"
python -c "import numpy, cv2, matplotlib, pandas, yaml; print('python deps ok')"
ros2 pkg list | grep -E "ros_gz|ros_ign"
ign gazebo --versions
```

如果服务器没有显示器，后续有 world 文件后再测试 server/headless 启动：

```bash
ign gazebo -s -r path/to/world.sdf
```

如果需要先验证命令行参数，可以看帮助：

```bash
ign gazebo --help
```

## 10. 构建与运行流程

### 10.1 构建

```bash
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
conda activate uav_gazebo
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

如果 conda 环境导致构建异常，使用系统 Python 构建 ROS2 workspace：

```bash
conda deactivate
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

然后运行纯算法节点时再打开 conda 终端：

```bash
conda activate uav_gazebo
source /opt/ros/humble/setup.bash
source /mnt/data2/lhm/uav/find_object_again/ros2_ws/install/setup.bash
```

### 10.2 启动仿真

推荐提供一个总启动文件：

```bash
ros2 launch multi_uav_sim_bringup demo_three_uavs.launch.py
```

启动内容：

```text
Gazebo server
world
3 架无人机
1 个目标
target_manager
vision_mock
coop_tracker for each UAV
control for each UAV
recorder
```

### 10.3 无界面运行

```bash
ros2 launch multi_uav_sim_bringup demo_three_uavs.launch.py headless:=true record:=true
```

### 10.4 查看结果

```text
runs/<timestamp>/
  topdown.mp4
  uav_1_camera.mp4
  uav_2_camera.mp4
  uav_3_camera.mp4
  events.jsonl
  tracks.csv
  metrics.json
  config.yaml
```

## 11. 配置文件设计

建议所有实验参数放在 YAML 中：

```yaml
experiment:
  name: three_uav_reacquire_demo
  duration_sec: 120
  seed: 7

world:
  size_x: 60.0
  size_y: 60.0

target:
  mode: scripted_occlusion
  speed: 1.5
  waypoints:
    - [-20.0, -10.0, 0.0]
    - [0.0, 0.0, 0.0]
    - [20.0, 10.0, 0.0]

uavs:
  - id: uav_1
    init_pose: [-12.0, -8.0, 8.0, 0.0]
  - id: uav_2
    init_pose: [0.0, -14.0, 8.0, 0.0]
  - id: uav_3
    init_pose: [12.0, -8.0, 8.0, 0.0]

camera:
  horizontal_fov_deg: 80.0
  vertical_fov_deg: 55.0
  max_detection_range: 35.0
  pitch_deg: -35.0

vision_mock:
  dropout_probability: 0.02
  position_noise_std: 0.5
  confidence_noise_std: 0.05
  lost_frame_threshold: 8

communication:
  delay_ms: 100
  packet_loss_probability: 0.0
  observation_timeout_sec: 0.5

reacquire:
  max_recovery_time_sec: 15.0
  uncertainty_radius_min: 2.0
  uncertainty_radius_growth: 0.8
  search_orbit_radius: 5.0

recorder:
  enabled: true
  fps: 20
  output_dir: runs
```

## 12. 第一阶段验收标准

第一阶段完成后应能稳定演示：

```text
1. 一条命令启动 3 架无人机 + 1 个目标。
2. 无人机正常跟随目标。
3. 场景中存在可复现的目标丢失事件。
4. 至少一架无人机进入 LOST 状态。
5. 其他无人机仍处于 TRACKING 状态并广播目标观测。
6. 丢失无人机进入 REACQUIRING 状态。
7. 丢失无人机根据 swarm_target_estimate 调整位置或朝向。
8. 丢失无人机重新检测到目标并进入 RECOVERED/TRACKING 状态。
9. 生成 topdown.mp4、events.jsonl、metrics.json。
```

关键指标：

```text
recovery_success_rate
mean_recovery_time
max_recovery_time
target_position_error
lost_duration
communication_delay
```

## 13. 迭代路线

### Milestone 1: 纯算法 mock 仿真

不依赖 Gazebo 相机图像，只用 Gazebo 或 Python 几何环境提供位姿和遮挡。

交付：

```text
多机位姿
目标轨迹
mock 视觉观测
协同恢复状态机
俯视视频
日志和指标
```

### Milestone 2: Gazebo 场景与模型完善

加入更完整的 Gazebo world、无人机模型、目标模型和障碍物。

交付：

```text
Gazebo world
无人机 SDF/URDF
目标 SDF/URDF
headless launch
Gazebo/ROS topic bridge
```

### Milestone 3: Gazebo RGB 相机或分割观测

替换部分 mock 视觉链路。

交付：

```text
Gazebo camera topic
图像保存
可选 segmentation/bbox sensor
统一 VisionObservation 输出
```

### Milestone 4: 真实检测/跟踪器接入

加入目标检测与跟踪模型。

交付：

```text
detector node
tracker node
camera calibration config
mock/real vision backend switch
```

### Milestone 5: PX4 SITL 接入

如果需要更真实飞控，再接 PX4。

交付：

```text
PX4 SITL 多机启动
offboard control
状态同步
真实控制约束
```

## 14. 风险与处理

### 14.1 Gazebo headless RGB 渲染不稳定

处理：

```text
第一阶段不依赖 RGB 相机。
优先保存 topdown.mp4。
RGB 视频作为第二阶段能力。
```

### 14.2 真实检测器影响算法验证

处理：

```text
算法只依赖 VisionObservation。
mock vision 和 real detector 可切换。
先验证协同逻辑，再验证视觉鲁棒性。
```

### 14.3 PX4 多机链路复杂

处理：

```text
第一阶段使用简化运动学控制。
PX4 放到后续里程碑。
```

### 14.4 多机通信延迟与时间同步

处理：

```text
所有消息使用 ROS time。
fusion 节点过滤过期观测。
配置化 delay 和 packet loss。
日志保存 timestamp 方便复盘。
```

## 15. 推荐立即开工顺序

建议按下面顺序实现：

```text
1. 创建 ROS2 workspace 和包结构。
2. 定义 VisionObservation、TrackerState、SwarmTargetEstimate、SimEvent。
3. 实现 target_manager，发布目标轨迹。
4. 实现简化 uav_control，控制 3 架无人机运动。
5. 实现 vision_mock，生成 detected/confidence/target estimate。
6. 实现 coop_tracker 状态机和融合逻辑。
7. 实现 recorder，保存 topdown.mp4/events/metrics。
8. 接入 Gazebo world 和简单模型。
9. 增加 headless launch。
10. 增加真实相机 topic 和视频保存。
```

第一版演示不应超过以下复杂度：

```text
3 架无人机
1 个目标
1 个遮挡事件
1 种融合方法
1 种恢复控制策略
1 个 topdown 视频
```

这样可以快速验证核心假设，并为后续真实视觉、Gazebo 相机、PX4 接入保留清晰接口。

## 16. 当前第一版 mock demo

在 ROS2/Gazebo 环境安装期间，先提供一个纯 Python demo，用于尽快验证协同再捕获闭环。

代码入口：

```text
run_mock_demo.py
sim/mock_demo.py
```

安装最小依赖：

```bash
pip install -r requirements.txt
```

运行：

```bash
python run_mock_demo.py
```

输出：

```text
runs/<timestamp>_mock_reacquire/
  topdown.mp4
  events.jsonl
  metrics.json
  tracks.csv
  config.json
```

默认演示内容：

```text
3 架无人机
1 个移动目标
uav_1 在约 18.7 秒视觉丢失
uav_1 立即使用 swarm target estimate 进入 REACQUIRING
uav_1 在约 28.1 秒恢复本机视觉观测
```

该 demo 暂不依赖 ROS2/Gazebo，目的是先把算法状态机、协同信息流、视频输出和指标保存跑通。后续迁移到 ROS2 时，`VisionObservation`、`TrackerState`、`SwarmTargetEstimate` 等接口应保持同样语义。

## 17. 当前 ROS2 第一版

ROS2 第一版已经按工作空间组织：

```text
ros2_ws/src/
  multi_uav_sim_msgs/
  multi_uav_sim_nodes/
  multi_uav_sim_bringup/
```

消息包：

```text
multi_uav_sim_msgs/msg/VisionObservation.msg
multi_uav_sim_msgs/msg/TrackerState.msg
multi_uav_sim_msgs/msg/SwarmTargetEstimate.msg
multi_uav_sim_msgs/msg/SimEvent.msg
multi_uav_sim_msgs/msg/UavPose2D.msg
```

节点包：

```text
target_manager
  发布 /target/ground_truth。

uav_control
  发布 /uav_i/pose2d，订阅 /uav_i/tracker_state。

vision_mock
  订阅目标和无人机位姿，发布 /uav_i/vision_observation。

coop_tracker
  订阅视觉观测，发布 /uav_i/tracker_state、/swarm/target_estimate、/swarm/events。

sim_recorder
  记录 events.jsonl、tracks.csv、metrics.json。
```

构建：

```bash
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

启动：

```bash
ros2 launch multi_uav_sim_bringup mock_demo.launch.py
```

当前 ROS2 版本仍然不依赖 Gazebo，作用是先把 topic、message、node 和 launch 链路打通。下一步接 Gazebo 时，优先替换：

```text
uav_control -> Gazebo/PX4 位姿或控制接口
target_manager -> Gazebo target model 控制
vision_mock -> Gazebo camera/segmentation/bbox backend
```

## 18. 当前 Gazebo 第一版接入

Gazebo 第一版已经加入：

```text
ros2_ws/src/multi_uav_sim_worlds/
  worlds/multi_uav_reacquire.sdf
  models/simple_uav/
  models/target_marker/
  models/occlusion_wall/
```

启动文件：

```text
ros2_ws/src/multi_uav_sim_bringup/launch/gazebo_mock_demo.launch.py
```

运行：

```bash
cd /mnt/data2/lhm/uav/find_object_again/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py
```

默认 headless：

```bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py headless:=true
```

打开 Gazebo GUI：

```bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py headless:=false
```

当前 Gazebo 接入方式：

```text
ign gazebo 启动 Fortress world
target_manager 发布 /target/ground_truth
uav_control 发布 /uav_i/pose2d
gazebo_pose_sync 订阅 ROS2 位姿
gazebo_pose_sync 调用 /world/multi_uav_reacquire/set_pose
Gazebo 中的 uav_1/uav_2/uav_3/target 模型随算法状态运动
```

说明：

```text
gazebo_pose_sync 使用 ign service 命令更新模型位姿。
这是为了快速完成服务器演示闭环。
后续如果需要更高频、更稳定的同步，应替换为 Gazebo system plugin 或 ros_gz/ros_ign bridge 方式。
```
