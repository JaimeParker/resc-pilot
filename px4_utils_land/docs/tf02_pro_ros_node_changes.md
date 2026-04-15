# TF02-Pro ROS 节点改造说明（完整变更记录）

日期：2026-04-15

## 背景与目标

你提出的目标是：

1. 不再只是串口测试脚本，而是作为 ROS 节点发布真实高度计数据。
2. 数据直接供 `PX4CtrlFSM` 使用（即发布到 `/scan`，消息类型 `sensor_msgs/LaserScan`）。

本次修改围绕上述目标完成。

---

## 本次改了哪些文件

### 1) 新增：TF02-Pro 串口转 ROS 的节点脚本

文件：`px4_utils_land/scripts/tf02_pro_altimeter_node.py`

新增内容要点：

1. 新建 ROS Python 节点 `tf02_pro_altimeter_node`。
2. 通过 pyserial 打开串口并读取 TF02-Pro 数据帧（9 字节协议，帧头 `0x59 0x59`）。
3. 对每帧进行 checksum 校验，解析：
   - 距离 `distance_cm`
   - 信号强度 `strength`
4. 将距离转成米（`distance_cm / 100.0`）。
5. 发布单束 `sensor_msgs/LaserScan` 到 `/scan`（默认）。
6. 支持参数化：
   - 串口参数：`port`、`baudrate`、`serial_timeout`
   - 发布参数：`scan_topic`、`frame_id`、`publish_rate`
   - 有效性参数：`range_min`、`range_max`、`signal_threshold`
   - 异常恢复：`reconnect_delay`
7. 异常处理策略：串口断开/读失败后自动重连。
8. 质量门控：
   - 信号低于门限或超量程时，发布 `NaN`。

为什么这样改：

1. 让 TF02-Pro 真实数据直接进入 ROS 图。 
2. 与 `PX4CtrlFSM` 当前订阅 `/scan` 的逻辑保持兼容。
3. 用参数化方式避免硬编码，便于在不同 Linux 设备迁移。

---

### 2) 新增：节点启动文件

文件：`px4_utils_land/launch/tf02_pro_altimeter.launch`

新增内容要点：

1. 新增 `<node pkg="px4_utils_land" type="tf02_pro_altimeter_node.py" ...>`。
2. 预置了常用参数默认值：
   - `port=/dev/ttyUSB0`
   - `baudrate=115200`
   - `scan_topic=/scan`
   - `frame_id=base_link`
   - `publish_rate=20.0`
   - `range_min=0.05`
   - `range_max=30.0`
   - `signal_threshold=0`

为什么这样改：

1. 提供可直接 `roslaunch` 启动的入口。
2. 让实机部署时只改 launch 参数即可，不必改代码。

---

### 3) 修改：CMake 安装 Python 节点

文件：`px4_utils_land/CMakeLists.txt`

修改内容：

新增 `catkin_install_python(...)`，将脚本安装到 catkin 可执行目录：

1. `scripts/tf02_pro_altimeter_node.py`
2. `DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}`

为什么这样改：

1. 让 `rosrun/roslaunch` 能正确定位并执行该 Python 节点。
2. 避免仅靠源码路径直接执行导致的部署不一致。

---

### 4) 修改：补充运行依赖

文件：`px4_utils_land/package.xml`

修改内容：

1. 新增：`<exec_depend>python3-serial</exec_depend>`

为什么这样改：

1. 新节点使用 pyserial 读取 UART。
2. 通过包依赖声明降低“运行时缺库”的概率。

---

## 与 FSM 的兼容关系

当前 `PX4CtrlFSM` 使用 `/scan` + `sensor_msgs/LaserScan` 作为高度输入，并在降落逻辑中检查：

1. 数据是否新鲜（时间窗）。
2. 距离是否在阈值范围内（`range_threshold`）。

本次 TF02 节点发布模型与此接口一致，因此可直接接入，不需要改 FSM 代码。

---

## 发布数据格式（节点输出）

`LaserScan` 的关键字段如下：

1. `header.stamp`：当前时间戳。
2. `header.frame_id`：默认 `base_link`。
3. `angle_min=0.0`。
4. `angle_max=0.0`。
5. `angle_increment=0.0`。
6. `time_increment=0.0`。
7. `scan_time=1.0/publish_rate`。
8. `range_min`、`range_max`：参数给定。
9. `ranges=[value]`：单束距离（米）。
10. `intensities=[strength]`：信号强度（浮点形式承载）。

这与项目里 fake altimeter 的“单束 LaserScan”接口保持一致。

---

## 启动与验证建议（Linux）

1. 编译工作区并 source 环境。
2. 启动节点：
   - `roslaunch px4_utils_land tf02_pro_altimeter.launch`
3. 查看输出：
   - `rostopic echo /scan`
4. 确认 FSM 可见：
   - FSM 运行时应不再出现长期“无新鲜高度数据”的状态。

---

## 注意事项

1. 串口权限：Linux 下可能需要 `dialout` 组权限。
2. 串口路径：不同机器可能是 `/dev/ttyUSB0`、`/dev/ttyUSB1` 等。
3. 量程与门限：
   - 若现场噪声大，可提高 `signal_threshold`。
   - `range_min/range_max` 与传感器实际规格保持一致。
4. 话题冲突：不要同时让多个节点发布 `/scan`（除非有明确仲裁）。

---

## 本次未改动项

1. 未修改 `PX4CtrlFSM` 逻辑。
2. 未替换现有 fake altimeter 节点。
3. 未将 TF02 节点自动并入现有主 launch（仅新增独立 launch，便于按需接入）。

---

## 结论

本次改动已把“TF02-Pro 串口读取脚本”升级为“可部署的 ROS 高度计节点”，并完成了：

1. 节点实现。
2. 启动入口。
3. 构建安装接入。
4. 依赖声明。

目标链路已经打通：`TF02-Pro UART -> ROS /scan -> PX4CtrlFSM`。
