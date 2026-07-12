# 空中目标与工业障碍演示场景实现计划

## 1. 背景

当前仿真流程已经跑通：

- 三架无人机跟踪目标。
- 本机视觉丢失后，可以使用集群中其他无人机的信息进入再捕获流程。
- Gazebo 中有基础 UAV、target、遮挡墙和相机录制链路。

但当前演示仍然偏简化：

- target 是地面附近的红色标记，不像无人机相机视角中的空中目标。
- 场景障碍物数量少，视觉上缺少复杂环境。
- 丢失目标主要依赖 `vision_mock` 中的 scripted loss，和 Gazebo 场景的视觉遮挡关系不够明显。
- 感知 mock 基本按 2D 平面处理，`elevation` 固定为 0，target 高度没有进入可见性判断。

本期目标不是重写跟踪/再捕获算法，而是把演示场景做得更可信：

```text
空中 target 在 UAV 摄像头范围内可见，
target 穿过障碍区域，
某一架 UAV 有明显丢失目标的过程，
其他 UAV 仍能保持观测，
后续算法升级可以继续接在当前状态机上。
```

## 2. 实现原则

本期采用低风险路线：

- 控制和跟踪主链路继续保持 2D `x/y`。
- target ground truth、Gazebo 展示、vision mock 可见性扩展为轻量 3D。
- Gazebo 场景用 SDF 几何体搭建工业风格障碍，不引入外部 mesh。
- scripted loss 保留为兜底，但时间和空间上要与障碍区域对齐，让丢失看起来由场景触发。
- 不实现复杂避障规划，不重写再捕获算法。

## 3. 目标效果

完成后默认 Gazebo demo 应该呈现：

```text
0-10s:
  三架 UAV 在 8m 左右高度飞行，target 是一个空中小型飞行器。
  target 进入编队前方，至少一个 UAV 前置相机可以看到 target。

10-18s:
  target 沿固定 3D 轨迹飞向工业障碍区。
  UAV 编队持续跟随，uav_2/uav_3 保持稳定观测。

18-28s:
  target 穿过高墙/门架/塔柱附近。
  uav_1 因遮挡或视角约束进入 OCCLUDED/LOST。
  uav_2/uav_3 继续观测，swarm estimate 仍有效。

28-35s:
  target 离开主要遮挡区。
  uav_1 重新具备可见条件，后续可恢复本机视觉跟踪。

35s 后:
  编队继续移动，证明 demo 不是一次性静态事件。
```

## 4. 改动范围

### 4.1 Python 节点

预计修改：

```text
ros2_ws/src/multi_uav_sim_nodes/multi_uav_sim_nodes/scenario.py
ros2_ws/src/multi_uav_sim_nodes/multi_uav_sim_nodes/target_manager.py
ros2_ws/src/multi_uav_sim_nodes/multi_uav_sim_nodes/vision_mock.py
ros2_ws/src/multi_uav_sim_nodes/multi_uav_sim_nodes/gazebo_pose_sync.py
ros2_ws/src/multi_uav_sim_nodes/multi_uav_sim_nodes/ros_utils.py
```

可能轻微修改：

```text
ros2_ws/src/multi_uav_sim_nodes/multi_uav_sim_nodes/uav_control.py
ros2_ws/src/multi_uav_sim_nodes/multi_uav_sim_nodes/coop_tracker.py
```

原则：

- `uav_control.py` 和 `coop_tracker.py` 仍然按 `x/y` 控制和融合。
- 如果需要读取 3D point，也只取 `x/y`，不把本期扩大成完整 3D tracker。

### 4.2 Gazebo SDF

预计修改：

```text
ros2_ws/src/multi_uav_sim_worlds/worlds/multi_uav_reacquire.sdf
ros2_ws/src/multi_uav_sim_worlds/models/target_marker/model.sdf
ros2_ws/src/multi_uav_sim_worlds/models/occlusion_wall/model.sdf
```

可能新增模型：

```text
ros2_ws/src/multi_uav_sim_worlds/models/industrial_block/
ros2_ws/src/multi_uav_sim_worlds/models/industrial_tower/
ros2_ws/src/multi_uav_sim_worlds/models/pipe_gate/
```

如果时间紧，可以不新增模型，直接在 world 文件里用多个 box/cylinder 组合。

### 4.3 配置

预计修改：

```text
ros2_ws/src/multi_uav_sim_bringup/config/mock_demo.yaml
```

配置目标：

- target 速度与丢失窗口对齐。
- camera range/FOV 能覆盖合理的空中跟踪距离。
- scripted loss 时间窗与障碍区域穿越时间一致。
- `support_uav_ids` 保持 `uav_2/uav_3`，保证主线清晰。

## 5. 详细实现步骤

### 5.1 扩展 scenario 中的 target 轨迹

当前 `default_target_waypoints()` 返回 2D `np.ndarray([x, y])`。

建议改成返回 3D：

```python
np.array([x, y, z], dtype=float)
```

推荐初版轨迹：

```text
P0  (-16, -10, 7.0)  起点，编队前方偏左
P1  (-6,  -6,  7.5)  建立稳定跟踪
P2  (4,   0,   8.2)  接近工业障碍区
P3  (12,  7,   7.2)  穿过门架/高墙附近
P4  (22,  10,  8.8)  离开遮挡区
P5  (12,  20,  7.5)  转向回场景上方
P6  (-10, 16,  8.0)  继续巡航
```

注意：

- UAV 默认高度是 `gazebo_pose_sync.uav_altitude = 8.0`。
- target 高度应围绕 8m 小幅变化，避免前置相机垂直角过大。
- 轨迹中的 `x/y` 应穿过障碍物的 2D 投影附近，方便 mock occlusion 对齐。

同时新增辅助函数，避免大量调用方自己判断 2D/3D：

```text
xy_from_vec(vec) -> np.ndarray([x, y])
xyz_from_point(point) -> np.ndarray([x, y, z])
point_from_xyz(vec) -> geometry_msgs/Point
```

### 5.2 TargetManager 发布 3D ground truth

当前逻辑：

```text
self.position = 2D waypoint
msg.point = point_from_xy(self.position)
```

改成：

```text
self.position = 3D waypoint
msg.point.x/y/z = self.position[0/1/2]
```

移动方式：

- 第一版可以使用 3D 直线插值。
- `target_speed` 表示 3D 空间速度。
- 如果希望 `x/y` 速度稳定，也可以只按平面距离推进，再插值高度；但第一版没必要复杂化。

验收：

```bash
ros2 topic echo /target/ground_truth
```

应该能看到 `point.z` 在 6-9m 范围内变化。

### 5.3 GazeboPoseSync 同步 target 高度

当前 `on_target()` 写死：

```python
Pose2D(msg.point.x, msg.point.y, 0.0, 0.0)
```

改成：

```python
Pose2D(msg.point.x, msg.point.y, msg.point.z, target_yaw)
```

第一版 `target_yaw` 可以先保持 0。

更好的实现：

- 在 `GazeboPoseSync` 内记录上一帧 target `x/y`。
- 当位移大于阈值时，用 `atan2(dy, dx)` 估计 yaw。
- 这样 target 飞行器朝向会沿轨迹转动。

验收：

- Gazebo 中 target 不再贴地。
- target 高度与 `/target/ground_truth.point.z` 一致。

### 5.4 改造 target_marker 模型

当前是红色 cylinder。

建议用 SDF 基础几何组合一个清晰的小型空中目标：

```text
body:
  box, 0.9 x 0.22 x 0.18, 亮橙或亮红

wing/arms:
  box, 0.18 x 1.2 x 0.08, 深色

nose:
  small box 或 sphere, 黄色/白色

rotor markers:
  4 个小 cylinder/sphere，放在四角，黑色或白色
```

设计要求：

- 尺寸略大于真实小无人机，保证相机画面可见。
- 颜色高对比，但不要是地面红球。
- `collision` 可以简单用一个 box，不追求真实。
- `gravity` 可设为 false，因为位姿由 `gazebo_pose_sync` 强制同步。

推荐模型尺寸：

```text
body length: 1.0m
arm span:    1.4m
height:      0.2m
rotor size:  0.18m
```

### 5.5 工业障碍场景

目标是视觉复杂、实现简单。

推荐障碍组合：

```text
1. central_wall
   高墙，制造主要遮挡。
   位置大致在 x=6..14, y=3..8，高度 8-11m。

2. pipe_gate
   两根立柱 + 顶部横梁，target 从附近穿过。
   视觉上像工业门架/管廊。

3. tower_cluster
   3-4 根 cylinder 或 box 塔柱。
   放在主轨迹侧边，增加层次。

4. low_factory_blocks
   几个 3-5m 高的建筑块。
   不一定遮挡 target，但让画面不空。

5. safety_barriers
   低矮长条，丰富地面工业感。
```

如果新增模型成本高，直接在 world 中写多个 `model` 即可：

```xml
<model name="industrial_block_a">
  <static>true</static>
  ...
</model>
```

建议第一版直接在 world 中内联几何体，后续稳定后再抽模型。

### 5.6 同步算法遮挡区域

当前 `default_obstacles()` 返回 2D `RectObstacle`。

第一版继续使用 2D 矩形近似工业障碍投影。

推荐障碍投影：

```text
central_wall:      RectObstacle("central_wall", 4.0, 2.0, 15.0, 8.5)
pipe_gate_left:    RectObstacle("pipe_gate_left", 8.0, 8.0, 10.0, 13.0)
pipe_gate_right:   RectObstacle("pipe_gate_right", 15.0, 7.0, 17.0, 13.0)
tower_cluster_a:   RectObstacle("tower_cluster_a", 1.0, 10.0, 5.0, 15.0)
factory_block_b:   RectObstacle("factory_block_b", -2.0, 2.0, 2.0, 7.0)
```

注意：

- 不要把所有障碍都加入 occlusion，否则 `uav_2/uav_3` 也可能频繁丢失。
- 主遮挡应重点影响 `uav_1`。
- 对 `support_uav_ids` 仍可保留 `enforce_support_visibility=True`，保证 demo 主线稳定。

### 5.7 VisionMock 加入轻量 3D 可见性

当前 `VisionMock`：

- target 只读取 `x/y`。
- range 是 2D 距离。
- bearing 是水平角。
- elevation 固定 0。
- bbox `cy` 固定 0.5。

建议改成：

```text
target_xyz: np.ndarray([x, y, z])
observer_xyz: np.ndarray([x, y, uav_altitude])
rel_xyz = target_xyz - observer_xyz
horizontal_rel = rel_xyz[:2]
range_3d = norm(rel_xyz)
bearing = atan2(rel_y, rel_x) - observer_yaw
elevation = atan2(rel_z, horizontal_distance)
```

新增参数：

```text
camera_vertical_fov_deg: 55.0
uav_altitude: 8.0
```

检测条件：

```text
in_range = range_3d <= camera_range
in_horizontal_fov = abs(bearing) <= camera_fov / 2
in_vertical_fov = abs(elevation) <= camera_vertical_fov / 2
occluded = 2D obstacle check
detected = all conditions and not forced_loss/dropout
```

loss reason 优先级建议：

```text
TOO_FAR
OUT_OF_FOV
OCCLUDED
RANDOM_DROPOUT
VISIBLE
```

如果水平或垂直 FOV 失败，都先归为 `OUT_OF_FOV`，避免改消息定义。

bbox 简化：

```text
bbox_cx = 0.5 + 0.5 * bearing / (horizontal_fov / 2)
bbox_cy = 0.5 - 0.5 * elevation / (vertical_fov / 2)
bbox_w/h = 根据 range_3d 缩放
```

注意：

- `target_position_estimate` 仍然可以只写 `x/y` 或写完整 `x/y/z`。
- `coop_tracker` 目前只读 `x/y`，所以写 `z` 不会破坏现有逻辑。

### 5.8 参数调优

建议 `mock_demo.yaml` 初始值：

```yaml
target_manager:
  ros__parameters:
    dt: 0.1
    target_speed: 1.8

vision_mock:
  ros__parameters:
    dt: 0.1
    camera_range: 45.0
    camera_fov_deg: 105.0
    camera_vertical_fov_deg: 60.0
    uav_altitude: 8.0
    observation_noise_std: 0.45
    dropout_probability: 0.0
    scripted_loss_uav: uav_1
    scripted_loss_start_sec: 17.0
    scripted_loss_end_sec: 27.0
    enforce_support_visibility: true
    support_uav_ids:
      - uav_2
      - uav_3

gazebo_pose_sync:
  ros__parameters:
    uav_altitude: 8.0
```

调参顺序：

```text
1. 先让 target 高度和 Gazebo 展示正确。
2. 再保证 uav_2/uav_3 稳定 detected。
3. 最后调整 uav_1 scripted loss 和障碍位置对齐。
```

## 6. 推荐任务拆分

### Task A: 3D target ground truth

改动：

- `scenario.py`
- `target_manager.py`
- `ros_utils.py`

验收：

- `/target/ground_truth.point.z` 非 0。
- 纯 ROS2 demo 不报错。
- tracker/control 仍能使用 `x/y` 正常运行。

### Task B: Gazebo target 空中同步

改动：

- `gazebo_pose_sync.py`
- `target_marker/model.sdf`
- `multi_uav_reacquire.sdf`

验收：

- target 在 Gazebo 中飞在空中。
- target 模型清晰可辨。
- UAV 模型仍保持 8m 高度。

### Task C: 工业几何场景

改动：

- `multi_uav_reacquire.sdf`
- 可选新增 model 目录。
- `scenario.py` 中更新 `default_obstacles()`。

验收：

- Gazebo 场景中存在高墙、门架、塔柱、建筑块。
- target 轨迹穿过障碍区。
- 顶视图或 GUI 中能看出遮挡关系。

### Task D: 3D vision mock

改动：

- `vision_mock.py`
- `mock_demo.yaml`

验收：

- `VisionObservation.elevation` 不再固定 0。
- `bbox_cy` 会随 elevation 变化。
- `uav_1` 在指定窗口出现 `OCCLUDED`/`LOST`。
- `uav_2/uav_3` 在主线中保持 detected 或提供 swarm estimate。

### Task E: 演示调参和记录

改动：

- `mock_demo.yaml`
- 可能微调 waypoints/obstacles。

验收：

- 默认 launch 下主线稳定。
- `runs_ros2/.../events.jsonl` 中出现预期事件。
- Gazebo 相机录制时能看到空中 target 和工业障碍。

## 7. 验证命令

构建：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

启动 Gazebo mock demo：

```bash
ros2 launch multi_uav_sim_bringup gazebo_mock_demo.launch.py headless:=false
```

检查 target 高度：

```bash
ros2 topic echo /target/ground_truth
```

检查事件：

```bash
ros2 topic echo /swarm/events
```

检查单机视觉：

```bash
ros2 topic echo /uav_1/vision_observation
ros2 topic echo /uav_2/vision_observation
```

录制相机：

```bash
ros2 launch multi_uav_sim_bringup gazebo_camera_record.launch.py headless:=false
```

## 8. 验收标准

必须满足：

- target 是空中目标，`z` 不为 0。
- target 模型不是地面红色圆珠/柱体。
- Gazebo 场景比当前更复杂，有工业几何障碍。
- target 轨迹穿过障碍区域。
- UAV 摄像头画面中能看到空中 target。
- `uav_1` 有一段可解释的目标丢失。
- `uav_2/uav_3` 能维持 demo 主线，不因场景复杂化频繁误丢。
- 现有跟踪/再捕获状态机不被破坏。

建议满足：

- target yaw 大致沿运动方向。
- `VisionObservation.elevation` 和 `bbox_cy` 有合理变化。
- 事件时间与 target 穿越障碍区时间接近。
- headless 和 GUI 模式都能运行，GUI 用于视觉验收。

## 9. 风险与边界

### 9.1 不做完整 3D 控制

本期 UAV 控制仍固定高度，跟踪估计仍以 `x/y` 为主。

原因：

- 当前消息和控制器都围绕 2D 建模。
- 完整 3D 控制会扩大修改范围。
- 本期重点是演示场景可信度，不是算法升级。

### 9.2 不做真实物理避障

场景中会有障碍物，但 UAV 不做真实动力学避障。

可以通过以下方式制造“因为避障/遮挡跟丢”的感觉：

- target 穿过障碍区。
- uav_1 的 scripted loss 与障碍区对齐。
- uav_1 编队位置和 yaw 使其更容易被主墙遮挡。
- uav_2/uav_3 作为支援视角保持观测。

### 9.3 工业场景用几何体搭建

不使用外部 mesh 的优点：

- 好维护。
- 不依赖额外资源下载。
- Gazebo 加载稳定。
- 容易和 2D occlusion 矩形对齐。

缺点：

- 视觉精细度有限。

本期可接受。

## 10. 后续可扩展方向

本期完成后，可以继续做：

- 将 tracker/control 升级为 3D 状态估计。
- 用真实 Gazebo 相机图像替代 `vision_mock`。
- 基于图像检测输出真实 bbox。
- 用 3D bounding box 或 ray casting 替代 2D occlusion。
- 给 UAV 加真实避障策略，而不是 scripted loss。
- 增加多目标或动态障碍物。

