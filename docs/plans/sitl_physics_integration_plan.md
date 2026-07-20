# SITL Physical Simulation Integration Plan

本文档给出把当前 Gazebo mock 无人机位姿同步方案替换为 SITL 物理仿真的实施计划。目标不是一次性接入完整飞行任务流程，而是先让现有“跟踪/丢失/再捕获”算法驱动物理多旋翼飞行，观察轨迹跟随、延迟、超调、转向和编队保持效果。

## 当前状态

当前工程已有 ROS2 节点和 Gazebo 场景：

- `uav_control` 根据 `TrackerState` 计算每架无人机的理想二维跟随位置，并发布 `/{uav_id}/pose2d`。
- `gazebo_pose_sync` 订阅 `/{uav_id}/pose2d`，通过 `ign service /world/multi_uav_reacquire/set_pose` 直接设置 Gazebo 模型位姿。
- `vision_mock` 和 `sim_recorder` 也订阅 `/{uav_id}/pose2d`，因此当前记录和视觉判定看到的是算法理想位姿，不是物理飞行结果。
- `simple_uav` 模型有视觉、碰撞和 IMU，但没有多旋翼电机、飞控接口和姿态动力学插件，且 `base_link` 关闭了重力。

因此，接 SITL 的核心不是继续强化 `simple_uav`，而是把它替换为飞控支持的多旋翼模型，并把 `uav_control` 的理想位姿改造成飞控 setpoint。

## 推荐路线

已确认使用 PX4 SITL + Ubuntu 22.04 Gazebo Fortress/Gz + ROS2 Offboard。

理由：

- 当前工程已经使用 ROS2 和 Ignition/Gazebo Gz 命令体系，和 PX4 的 Gz 模拟路线一致。
- PX4 官方支持 Gazebo Gz 多机仿真，每个 PX4 instance 可以用独立 instance id 和模型名启动。
- PX4 ROS2 Offboard 可直接发布 `OffboardControlMode`、`TrajectorySetpoint`、`VehicleCommand` 等 topic，不需要额外绕 MAVROS。
- 对本项目第一阶段需求来说，PX4 的 position/velocity setpoint 足够验证“算法轨迹进入物理闭环后的飞行效果”。

PX4 源码加入本仓库，建议以 git submodule/vendor dependency 管理，路径固定为 `external/PX4-Autopilot`，避免把 PX4 源码直接混入本项目 ROS2 包结构。

## 简化目标

第一阶段只验证飞行效果，故意忽略以下复杂度：

- 不做真实任务级起飞流程。启动后由适配节点自动 arm、切 offboard，并保持指定高度。
- 不做复杂门控逻辑。只要 PX4 状态进入可控，就持续发送 setpoint。
- 不做真实视觉检测。继续使用 `vision_mock`，但输入位姿必须来自 SITL 反馈，而不是理想位姿。
- 不处理失控恢复、返航、降落、避障等真机安全逻辑。
- 不引入多机协同通信链路的真实性，只验证本项目跟踪控制输出驱动物理机体后的运动质量。
- 三架跟踪无人机和目标都使用 PX4 物理无人机模型。当前实现中跟踪机保持 `8 m`，目标保持 `7 m`，这样与原场景高度关系一致。

这条路径是可行的。关键风险不在算法，而在坐标系、topic 命名、多机启动和 setpoint 平滑。

## 目标架构

建议把系统改成三层：

```text
target_manager
  -> /target/desired_position

coop_tracker + vision_mock
  -> /uav_X/tracker_state

uav_control
  -> /uav_X/desired_pose2d

sitl_offboard_adapter
  subscribes:
    /uav_X/desired_pose2d
    /target/desired_position
    /px4_X/fmu/out/vehicle_local_position
    /px4_X/fmu/out/vehicle_attitude
  publishes:
    /px4_X/fmu/in/offboard_control_mode
    /px4_X/fmu/in/trajectory_setpoint
    /px4_X/fmu/in/vehicle_command
    /uav_X/pose2d
    /target/ground_truth

vision_mock + sim_recorder
  subscribes:
    /uav_X/pose2d
```

其中：

- `/uav_X/desired_pose2d` 是算法期望位置。
- `/uav_X/pose2d` 是 SITL 反馈后的真实物理位置，供视觉 mock、tracker 和 recorder 使用。
- `/target/desired_position` 是目标路径期望点。
- `/target/ground_truth` 是目标 PX4 物理无人机反馈位置，供视觉 mock、tracker 和 recorder 使用。
- 原 `gazebo_pose_sync` 在 SITL 模式下关闭。

如果短期内不想改 topic 名，也可以保留 `uav_control -> /uav_X/pose2d`，新增 `sitl_pose_feedback -> /uav_X/sitl_pose2d`。但这会迫使 `vision_mock` 和 `sim_recorder` 增加参数选择输入 topic。长期看，拆出 `desired_pose2d` 更清晰。

## 实施步骤

### 1. 冻结当前 mock 行为

先确保现有 mock demo 可复现，作为 SITL 接入后的对照：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py headless:=false pose_sync:=true
```

记录当前 `tracks.csv` 中 target 与 UAV 的相对距离、恢复时间、轨迹形状。SITL 接入后不要只看 Gazebo 画面，要和这些指标对比。

### 2. 引入 PX4 SITL 运行环境

准备 PX4-Autopilot、Gazebo Fortress/Gz、Micro XRCE-DDS Agent 和 `px4_msgs`。

官方 PX4 Gz 仿真当前支持 `make px4_sitl gz_x500`，多机模式支持多个 PX4 instance，并通过 `MicroXRCEAgent udp4 -p 8888` 暴露 ROS2 topic。PX4 Offboard 模式要求持续发送生命信号，ROS2 路线下通常是持续发布 `OffboardControlMode`，实际位置 setpoint 发布到 `TrajectorySetpoint`。

PX4 源码放在本仓库 `external/PX4-Autopilot`，建议用 submodule 管理。launch 仍保留 `px4_root` 参数，默认可指向这个目录：

```text
px4_root:=external/PX4-Autopilot
px4_model:=x500
px4_world:=multi_uav_reacquire
```

### 3. 替换 Gazebo UAV 模型

第一阶段用 PX4 自带 `gz_x500` 模型，而不是改造 `simple_uav`。

需要处理两件事：

- 保留本项目的 world 静态环境和遮挡物。
- 让 PX4 启动时在同一个 Gz world 中 spawn 四架 x500，初始位置对应当前 `uav_1/uav_2/uav_3` 和目标起点。

建议新建一个 SITL world 或 launch 变体：

```text
ros2_ws/src/multi_uav_sim_worlds/worlds/multi_uav_reacquire_sitl.sdf
```

已新增 `multi_uav_reacquire_sitl.sdf`。这个 world 中移除 `simple_uav` 和 `target_marker` include，只保留：

- ground
- central_wall
- pipe_gate
- tower_cluster
- factory_block
- safety_barriers
- 必要的 physics/sensors/user commands plugins

多旋翼模型由 PX4 启动流程生成，避免 world 里模型名和 PX4 自动 spawn 冲突。目标也由 PX4 生成，逻辑名为 `target`，PX4 namespace 建议用 `px4_4`。

### 4. 新增 `sitl_offboard_adapter`

新增 ROS2 Python 节点，职责是把本项目的二维期望位姿转换成 PX4 Offboard setpoint，并把 PX4 本地位置反馈转换回 `UavPose2D`。

建议参数：

```yaml
sitl_offboard_adapter:
  ros__parameters:
    uav_ids: ["uav_1", "uav_2", "uav_3"]
    px4_namespaces: ["px4_1", "px4_2", "px4_3"]
    command_rate_hz: 20.0
    hold_altitude_m: 8.0
    max_xy_speed_mps: 3.0
    max_yaw_rate_dps: 60.0
    position_mode: true
    auto_arm: true
    auto_offboard: true
    target_enabled: true
    target_px4_namespace: px4_4
    target_system_id: 4
    target_hold_altitude_m: 7.0
```

节点行为：

- 订阅 `/{uav_id}/desired_pose2d`。
- 订阅 `/target/desired_position`。
- 持续以 20 Hz 发送 `OffboardControlMode`。
- 持续发送 `TrajectorySetpoint`，位置为 `(x, y, z)`，yaw 为期望朝向。
- 启动后先发送至少 1 秒 setpoint，再自动 arm 和切 offboard。
- 将 PX4 local position 和 attitude 转为 `/{uav_id}/pose2d`，供 `vision_mock`、`coop_tracker`、`sim_recorder` 使用。
- 将目标 PX4 local position 转为 `/target/ground_truth`，供 `vision_mock`、`coop_tracker`、`sim_recorder` 使用。

坐标转换要作为重点实现：

- ROS/Gazebo 常用 ENU：`x` 前方/东，`y` 左方/北，`z` 向上。
- PX4 local position/setpoint 常用 NED：`x` 北，`y` 东，`z` 向下。
- 如果本项目 world 坐标继续按当前 `x/y/z up` 使用，则 adapter 必须集中处理 ENU/NED 转换，其他算法节点不要感知 PX4 坐标。
- PX4 local frame 通常以每架机启动/home 位置为原点。adapter 需要保存每架机的 Gazebo world spawn offset，发送 setpoint 时先转为相对 local 坐标，反馈时再加回 world offset。

### 5. 修改 `uav_control` 输出语义

把 `uav_control` 的发布 topic 从 `/{uav_id}/pose2d` 改为 `/{uav_id}/desired_pose2d`，消息类型仍可暂时复用 `UavPose2D`。

当前代码通过参数 `pose_topic_suffix` 实现：mock 配置默认仍发布 `pose2d`，SITL 配置发布 `desired_pose2d`。

保留参数：

- `uav_speed`
- `follow_radius`
- `orbit_radius`
- `orbit_period`
- `world_half_size`

但要注意 `uav_speed` 此时不再是物理无人机速度，而是理想参考点的移动速度。真实速度由 PX4 position controller 和 `sitl_offboard_adapter` 限速共同决定。

### 6. 修改 `vision_mock` 和 `sim_recorder`

这两个节点必须继续订阅 `/{uav_id}/pose2d`，但该 topic 现在由 `sitl_offboard_adapter` 发布，代表真实/估计的物理位姿。

这样可以直接观察：

- 飞行器是否能跟上目标；
- 再捕获逻辑是否因为物理滞后而延迟；
- 机头 yaw 是否导致目标出 FOV；
- 多机编队是否因惯性和速度限制变形。

### 7. 新增 launch 文件

建议新增：

```text
ros2_ws/src/multi_uav_sim_bringup/launch/px4_sitl_demo.launch.py
```

启动内容：

- Gz world：`multi_uav_reacquire_sitl.sdf`
- 4 个 PX4 SITL instance，模型为 `x500`，其中 `px4_4` 是目标无人机
- `MicroXRCEAgent udp4 -p 8888`
- `target_manager`
- `uav_control`
- `vision_mock`
- `coop_tracker`
- `sim_recorder`
- `sitl_offboard_adapter`
- 可选 camera bridge 和 recorder

建议 launch 参数：

```text
headless:=true
px4_root:=external/PX4-Autopilot
px4_model:=x500
px4_world:=multi_uav_reacquire
start_px4:=true
start_agent:=true
auto_arm:=true
auto_offboard:=true
```

推荐启动命令：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch multi_uav_sim_bringup px4_sitl_demo.launch.py headless:=false
```

当前已新增第一版：

- `ros2_ws/src/multi_uav_sim_bringup/launch/px4_sitl_demo.launch.py`
- `ros2_ws/src/multi_uav_sim_bringup/config/sitl_demo.yaml`
- `ros2_ws/src/multi_uav_sim_nodes/multi_uav_sim_nodes/sitl_offboard_adapter.py`
- `ros2_ws/src/multi_uav_sim_worlds/worlds/multi_uav_reacquire_sitl.sdf`

为了避免一开始被多机启动脚本卡住，可以先手动启动 PX4 四机和 MicroXRCEAgent，然后用 `start_px4:=false start_agent:=false` 只启动本项目 ROS 节点。确认 topic 和飞行闭环打通后，再把 PX4 进程纳入 launch。

### 8. 第一阶段验收标准

只做最小闭环验收：

- `ros2 topic list` 能看到 `/px4_1/fmu/out/vehicle_local_position` 到 `/px4_4/fmu/out/vehicle_local_position`。
- 三架跟踪机和目标机在 Gazebo 中不是被 `set_pose` 瞬移，而是通过 PX4 物理闭环移动。
- `/{uav_id}/pose2d` 和 `/target/ground_truth` 来自 PX4 反馈，并被 `vision_mock` 使用。
- `tracks.csv` 中的 UAV 位置有真实滞后，不再完全贴合理想轨迹。
- scripted loss 期间仍能触发 `LOST -> REACQUIRING -> RECOVERED` 或暴露出因为物理滞后导致的失败。
- 画面上三架机能围绕目标形成大致编队，不要求精确经过所有点。

## 风险和处理

### 坐标系错误

最常见问题是 ENU/NED 轴向或 yaw 符号错，表现为无人机朝 90 度错误方向飞、向地下飞、或 yaw 反向。

处理方式：

- adapter 中单独实现并单测 `enu_to_ned_position`、`ned_to_enu_position`、`enu_yaw_to_ned_yaw`。
- adapter 中为每架机保存 `origin_enu`，避免把 Gazebo world 坐标直接当 PX4 local setpoint。
- 先只控制 `uav_1` 飞到固定点，再接入完整跟踪逻辑。

### Offboard 没切成功

PX4 Offboard 需要持续 setpoint/生命信号，且通常要先发送一段时间再 arm/switch mode。

处理方式：

- adapter 启动后先进入 `WARMUP` 状态，持续发布 hold setpoint。
- 1 秒后发送 arm 和 offboard command。
- 即使切换失败，也继续发送 setpoint，并周期性重试，直到进入 offboard。

### 物理响应跟不上算法参考点

当前 `uav_control` 的理想轨迹可能过于“几何化”，物理机体会超调或追不上。

处理方式：

- 在 adapter 中增加限速、限加速度、yaw rate 限制。
- 第一阶段把 `uav_speed` 降到 `1.5-2.0 m/s`，`orbit_radius` 降低或关闭。
- 先固定高度 `8 m`，只观察水平跟随。

### 多机 topic 命名不一致

PX4 多机 ROS2 topic 通常带 `/px4_1`、`/px4_2` 之类 namespace，但实际命名会受 instance id 和 PX4 版本影响。

处理方式：

- 把 PX4 namespace 做成配置参数。
- launch 启动后先用 `ros2 topic list` 校验，再启动 adapter。

### 相机模型变化

PX4 `gz_x500` 默认相机能力可能和当前 `simple_uav` 的 `front_camera` 不一致。

处理方式：

- 第一阶段继续使用 `vision_mock`，不依赖真实图像检测。
- 需要录像时，可以后续选 `gz_x500_depth` 或自定义带前视 camera 的 PX4 模型。

## 建议里程碑

### Milestone A: 单机固定点 Offboard

- 启动 1 架 PX4 x500。
- 新增最小 `sitl_offboard_adapter`。
- 让 `uav_1` 在 `z=8 m` 保持，再移动到 2-3 个固定点。
- 输出 `/uav_1/pose2d`。

### Milestone B: 单机接跟踪算法

- `uav_control` 输出 `/uav_1/desired_pose2d`。
- adapter 驱动 PX4 飞行。
- `vision_mock` 使用 `/uav_1/pose2d`。
- 验证 yaw/FOV 和目标跟随。

### Milestone C: 三机并行

- 启动 3 个 PX4 instance。
- adapter 同时管理三架机。
- `vision_mock`、`coop_tracker`、`sim_recorder` 恢复三机流程。
- 对比 mock 与 SITL 的恢复时间和轨迹误差。

### Milestone D: 清理 launch 和配置

- 新增 `px4_sitl_demo.launch.py`。
- 新增 `sitl_demo.yaml`。
- 文档化启动命令、PX4 路径、常见问题。
- 保留原 `gazebo_mock_demo.launch.py`，作为快速非物理对照。

## 已确认决策和仍需校正项

已确认：

- 飞控使用 PX4。
- 系统环境是 Ubuntu 22.04 + Gazebo Fortress。
- PX4 加入本仓库，路径为 `external/PX4-Autopilot`。
- 三架跟踪无人机一起接入。
- 目标也使用 PX4 物理无人机模型。
- 第一阶段高度由实现决定：跟踪机 `8 m`，目标 `7 m`。

仍需第一次实跑时校正：

- PX4 多机在本机实际暴露的 ROS2 namespace 是否为 `/px4_1` 到 `/px4_4`。
- 当前 PX4 版本的 Gz spawn 环境变量是否完全匹配 `PX4_GZ_MODEL`、`PX4_GZ_MODEL_NAME`、`PX4_GZ_MODEL_POSE`、`PX4_GZ_WORLD`。
- `x500` 默认模型是否满足录像需求；第一阶段继续用 `vision_mock`，真实相机后续再补。

## 参考资料

- PX4 Gazebo Gz simulation: https://docs.px4.io/main/en/sim_gazebo_gz/
- PX4 multi-vehicle simulation with Gazebo: https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation
- PX4 ROS2 Offboard control example: https://docs.px4.io/main/en/ros/ros2_offboard_control
- PX4 Offboard mode requirements: https://docs.px4.io/main/en/flight_modes/offboard.html
