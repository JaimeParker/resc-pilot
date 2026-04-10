# 降落逻辑变更说明

日期：2026-04-09
更新：2026-04-10

## 目标

本次调整的目标是把 `PX4CtrlFSM` 的降落阶段改成以下行为：

1. 保留高度计接口。
2. 当飞机距离地面低于阈值时，仍然可以切换到 `AUTO_LAND`。
3. 如果高度计缺失或数据过期，系统继续保持 `SOFT_LAND`，不执行 `AUTO_LAND` 切换。
4. 在 `SOFT_LAND` 过程中，继续使用 RTK 经纬度做横向二次校正，但每次输出的 `dx`、`dy` 都要限幅，避免单次移动过大。

## 修改 1：保留高度计阈值触发 `AUTO_LAND`

### 改了什么

- 在 `PX4CtrlFSM::fsmSoftLand()` 中增加了“高度计数据是否新鲜”的判断。
- `PX4CtrlFSM::checkHeightForTargetReached()` 继续负责判断当前测得的高度是否低于 `range_threshold_`。

### 为什么这样改

软降落只是中间阶段，最后仍然需要一个明确的“接近地面”条件，把控制权交给 PX4 的 `AUTO.LAND`。

### 改完后的行为

- 高度计数据有效且高度低于阈值时，状态从 `SOFT_LAND` 切换到 `AUTO_LAND`。
- 高度计数据过期或缺失时，不切换到 `AUTO_LAND`。

## 修改 2：高度计异常时保持 `SOFT_LAND`

### 改了什么

- 新增了 `hasFreshLandingHeightMeasurement()`，用于判断高度计数据是否仍然有效。
- `SOFT_LAND` 的超时逻辑不再在高度计无效时强制跳转到 `AUTO_LAND`。
- 当高度计无效时，控制器继续发布软降落 setpoint，并保持在 `SOFT_LAND`。

### 为什么这样改

如果高度计已经失效，再把超时兜底也转成 `AUTO_LAND`，会让系统依赖一份不可信的地面距离信息，这和降落安全目标相冲突。

### 改完后的行为

- 高度计可用：超时后仍可按原逻辑进入 `AUTO_LAND`。
- 高度计缺失或过期：继续软降落，不执行 `AUTO_LAND` 切换。

## 修改 3：在 `SOFT_LAND` 中加入限幅 RTK 二次校正

### 改了什么

- `fsmSoftLand()` 在每个周期都使用 `current_global_position_` 和 `target_global_position_` 计算横向误差。
- 计算出来的 `dx`、`dy` 先做限幅，再应用到当前降落 setpoint。
- 默认单次 RTK 修正上限为 `landing_rtk_max_step_ = 0.10 m`。

### 为什么这样改

RTK 可以提升最后阶段的落点对准精度，但原始经纬度误差换算出来的横向修正可能偏大。限幅可以避免单次 setpoint 跳变过大，保证降落阶段的连续性和可控性。

### 改完后的行为

- RTK 数据可用时，软降落阶段会持续做受限的横向修正。
- RTK 数据不可用时，软降落仍然继续，只是不做横向校正。

## 修改 4：`TRAJ_CMD` 在无 `/scan` 时也允许进入 `SOFT_LAND`

### 改了什么

- 在 `PX4CtrlFSM::handlePrecisionPositioning()` 中，把 `TRAJ_CMD -> SOFT_LAND` 的触发条件从“必须高度计满足阈值”改为：
	- 高度计可用：仍要求高度阈值满足；
	- 高度计不可用（无 `/scan` 或数据过期）：允许仅基于 GPS 接近阈值进入 `SOFT_LAND`。
- 新增了对应日志，明确区分“有高度计触发”与“无高度计兜底触发”。

### 为什么这样改

在仿真或实机异常场景中，`/scan` 可能不存在或短时中断。若把高度计作为 `TRAJ_CMD -> SOFT_LAND` 的硬门槛，会导致飞机长期停在 `TRAJ_CMD`，无法进入降落流程。

### 改完后的行为

- 有高度计且数据新鲜：保持原有安全门控，满足高度阈值后进入 `SOFT_LAND`。
- 无高度计或数据过期：当 GPS 接近任务点阈值后，允许进入 `SOFT_LAND`，避免卡死在 `TRAJ_CMD`。
- 进入 `SOFT_LAND` 后，是否切换 `AUTO_LAND` 仍由高度计可用性门控，不会因为缺失高度计强制进入 `AUTO_LAND`。

## 代码位置

- [px4_utils_land/src/utils/PX4CtrlFSM.cc](../src/utils/PX4CtrlFSM.cc)
- [px4_utils_land/include/px4_utils_land/PX4CtrlFSM.h](../include/px4_utils_land/PX4CtrlFSM.h)

## 运行说明

- `range_threshold_` 仍然表示切换到 `AUTO_LAND` 的高度阈值。
- 最终是否真正着地，仍然以 PX4 的 landed-state 反馈为准。
- 视觉降落路径仍然保持禁用。

## 正常飞行主流程

当前推荐的仿真主流程是：

1. `INIT` -> `OFFBOARD` -> `ARM` -> `TAKEOFF`
2. 起飞完成后进入 `HOLD`
3. 在 `HOLD` 中发布目标点到 `/move_base_simple/goal`
4. 规划器输出轨迹后，`PX4CtrlFSM` 进入 `TRAJ_CMD`
5. 在 `TRAJ_CMD` 中持续飞行，直到接近任务目标
6. 进入 `SOFT_LAND`
7. 如果高度计有效且低于阈值，则切到 `AUTO_LAND`
8. PX4 判定着地后进入 `LANDED`，随后 `DISARM`

### 这一流程的依据

- 这样可以把“任务点发布”和“轨迹跟随”分离开，避免在起飞阶段过早进入任务控制。
- `HOLD` 负责发布目标点，`TRAJ_CMD` 负责真正沿轨迹飞行，`SOFT_LAND` 负责收口。
- 这也是当前仿真里从正常起飞到降落的推荐路径。
