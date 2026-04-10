# 仿真伪高度计接入说明

## 目的

为仿真环境提供一个稳定的“伪激光测距高度计”话题，供 `PX4CtrlFSM` 的 `/scan` 订阅使用，完成起飞-飞行-降落全流程。

## 实现方式

新增节点：`fake_lidar_altimeter_node`

- 优先数据源：`/gazebo/model_states`（读取 Gazebo 里无人机真实 z）
- 回退数据源：`/mavros/local_position/pose`
- 输出话题：`/scan`（`sensor_msgs/LaserScan`，单束）
- 输出值定义：
  - `range = vehicle_z - landing_surface_z`
  - 可选高斯噪声
  - 受 `min_range` / `max_range` 限制

## 文件改动

- 新增节点源码：`px4_utils_land/src/fake_lidar_altimeter_node.cc`
- 构建接入：`px4_utils_land/CMakeLists.txt`
- 依赖接入：`px4_utils_land/package.xml`
- 仿真 launch 接入：`px4_utils_land/launch/dev_px4_fsm_interface_simulation.launch`

## 关键参数

在 `dev_px4_fsm_interface_simulation.launch` 中配置：

- `drone_model_name`：Gazebo 中无人机模型名，默认 `iris`
- `landing_surface_z`：降落面的世界坐标 z
  - 地面降落通常设为 `0.0`
  - 若降落在平台/墙顶，设为该平台顶面 z
- `publish_rate`：发布频率
- `noise_stddev`：模拟测距噪声
- `source_timeout_sec`：数据源超时阈值

## 与 FSM 的关系

`PX4CtrlFSM` 当前通过 `/scan` 判断接地阈值：

- 当测得高度低于 `px4fsm/range_threshold`，且高度数据新鲜时，可切换 `AUTO_LAND`
- 当高度计数据缺失或过期时，保持 `SOFT_LAND`

## 使用建议

1. 先确认 Gazebo 模型名和 `drone_model_name` 一致。
2. 如果仿真中有额外障碍/平台降落，修改 `landing_surface_z` 到目标表面高度。
3. 若需要更接近真实传感器，可将 `noise_stddev` 调成 `0.01~0.05`。
4. 确保仿真时没有其他节点同时向 `/scan` 发布冲突数据。
