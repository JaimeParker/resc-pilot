# 降落改动逐行目标映射

日期：2026-04-09

## 目标编号

- G1：保留高度计接口，距离地面小于阈值时切换 `AUTO_LAND`
- G2：高度计缺失或故障时，降落阶段保持 `SOFT_LAND`，不切换 `AUTO_LAND`
- G3：`SOFT_LAND` 过程中持续使用 RTK 二次校正，并对每次 `dx`、`dy` 输出限幅
- G4：`TRAJ_CMD` 在高度计缺失时，不再被 `/scan` 阻塞，允许按 GPS 接近条件切入 `SOFT_LAND`
- G0：与本次三项目标无直接对应（历史改动或语义注释）

---

## 文件 1：`px4_utils_land/include/px4_utils_land/PX4CtrlFSM.h`

### Hunk A（@@ -131,6 +131,7）

- 新增 `double landing_rtk_max_step_ = 0.10;`
  - 对应目标：G3
  - 原因：提供 RTK 横向修正单次限幅参数，防止每周期位移过大。

### Hunk B（@@ -197,6 +198,7）

- 新增 `bool hasFreshLandingHeightMeasurement() const;`
  - 对应目标：G1、G2
  - 原因：把“高度计数据是否可用”单独抽象成接口，作为是否允许切换 `AUTO_LAND` 的门控。

---

## 文件 2：`px4_utils_land/src/utils/PX4CtrlFSM.cc`

### Hunk A（@@ -374,24 +374,27）

- `SOFT_LAND` 分支从 `fsmVisionLand();` 改为 `fsmSoftLand();`
  - 对应目标：G0（该项属于前置改动，非本次三项目标本体）
  - 原因：视觉降落路径已禁用，当前以软降落为主链路。
- 视觉降落初始化代码整体改为注释保留
  - 对应目标：G0
  - 原因：保留历史实现，便于后续恢复。

### Hunk B（@@ -698,7 +701,8）

- `traj_target_.position.z` 从 `quad_pos_cmd_.position.z + origin_point_.z` 改为 `quad_pos_cmd_.position.z`
  - 对应目标：G0
  - 原因：该项不是本次三项目标要求内容，属于已有历史改动。

### Hunk C（@@ -722,6 +726,8）

- 新增 `static int land_num = 0;`
  - 对应目标：G3（辅助）
  - 原因：用于周期性打印 RTK 修正信息，降低日志频率。
- 新增 `land_num++;`
  - 对应目标：G3（辅助）
  - 原因：驱动周期计数。

### Hunk D（@@ -735,8 +741,35）

- 保留 `hold_pos_.z() -= 0.005;`
  - 对应目标：G2（上下文）
  - 原因：在高度计异常时仍应继续软降落，保持下降动作。

- 新增 `if (global_position_received_ && global_setpoint_received_) {`
  - 对应目标：G3
  - 原因：仅在 RTK 数据可用时启用二次校正。

- 新增 `raw_rtk_correction = calculateGPSVector(...)`
  - 对应目标：G3
  - 原因：计算当前 RTK 到目标的横向误差。

- 新增 `limited_rtk_correction = raw_rtk_correction;`
  - 对应目标：G3
  - 原因：保留原值并独立执行限幅，便于对比与日志追踪。

- 新增 `limited_rtk_correction.x() = std::max(-landing_rtk_max_step_, std::min(...))`
  - 对应目标：G3
  - 原因：对 `dx` 单轴限幅。

- 新增 `limited_rtk_correction.y() = std::max(-landing_rtk_max_step_, std::min(...))`
  - 对应目标：G3
  - 原因：对 `dy` 单轴限幅。

- 新增 `if (limited_rtk_correction.norm() > landing_rtk_max_step_) ...`
  - 对应目标：G3
  - 原因：进一步做向量模长限幅，防止合成位移超过阈值。

- 新增 `hold_pos_.x() = pos_.x() + limited_rtk_correction.x();`
  - 对应目标：G3
  - 原因：应用受限 `dx`。

- 新增 `hold_pos_.y() = pos_.y() + limited_rtk_correction.y();`
  - 对应目标：G3
  - 原因：应用受限 `dy`。

- 新增 RTK 原始值/限幅值日志输出（`raw(dx,dy)` 与 `limited(dx,dy)`）
  - 对应目标：G3
  - 原因：可观测性要求，便于验证限幅是否生效。

- 新增 RTK 不可用日志（`skipped because global GPS data is unavailable`）
  - 对应目标：G3（降级路径）
  - 原因：明确说明此周期未做校正。

### Hunk E（同一段）

- 新增 `const bool landing_height_available = hasFreshLandingHeightMeasurement();`
  - 对应目标：G1、G2
  - 原因：后续切换逻辑统一由高度计有效性门控。

### Hunk F（@@ -745,15 +778,38）

- 新增 `if (landing_height_available && checkHeightForTargetReached()) { ... changeFSMState(AUTO_LAND); ... }`
  - 对应目标：G1
  - 原因：仅在高度计有效且高度低于阈值时切换 `AUTO_LAND`。

- 将超时逻辑改为：
  - `if (landing_height_available) { ... changeFSMState(AUTO_LAND); }`
    - 对应目标：G1
    - 原因：高度计有效时，保留超时切换兜底。
  - `else { ... keep SOFT_LAND; land_start_time = ros::Time::now(); }`
    - 对应目标：G2
    - 原因：高度计无效时禁止切换 `AUTO_LAND`，继续软降落。

### Hunk G（同段新增函数）

- 新增 `bool PX4CtrlFSM::hasFreshLandingHeightMeasurement() const { ... }`
  - 对应目标：G1、G2
  - 原因：统一高度计有效性判定规则：必须有扫描数据且时间戳不超过 1 秒。

### Hunk H（`void PX4CtrlFSM::fsmVisionLand()` 前新增注释）

- 新增“视觉降落已停用”说明注释
  - 对应目标：G0
  - 原因：历史说明，不直接影响本次三项目标行为。

### Hunk I（@@ -1427,7 +1483,7）

- 日志：`[SimpleEgoPlanner] Laser scan data is too old!` -> `[PX4 FSM]: Landing height data is too old!`
  - 对应目标：G1、G2（可观测性）
  - 原因：该函数属于降落门控，日志应准确反映模块归属。

### Hunk J（@@ -1436,7 +1492,7）

- 日志：`Target reached ... Ready for GPS correction` -> `Landing height ... below threshold ...`
  - 对应目标：G1（可观测性）
  - 原因：语义改为“高度阈值触发 `AUTO_LAND`”而非“GPS 校正”。

### Hunk K（`handlePrecisionPositioning()` 条件扩展，2026-04-10）

- 新增 `const bool landing_height_available = hasFreshLandingHeightMeasurement();`
  - 对应目标：G4（并与 G1/G2 对齐）
  - 原因：在 `TRAJ_CMD` 阶段显式区分“有无可用高度计数据”，避免把 `/scan` 作为唯一路径。

- 新增 `height_ready_for_landing` 仅在高度计可用时调用 `checkHeightForTargetReached()`
  - 对应目标：G4
  - 原因：避免在高度计缺失时反复执行无意义门控，清晰表达双路径条件。

- 触发条件从
  - `gps_distance < 0.1 && checkHeightForTargetReached()`
  - 调整为
  - `gps_distance < 0.1 && (height_ready_for_landing || !landing_height_available)`
  - 对应目标：G4
  - 原因：当 `/scan` 缺失或过期时，仍可从 `TRAJ_CMD` 切入 `SOFT_LAND`，防止任务卡死。

- 新增无高度计分支日志：`... no fresh /scan, switching to SOFT_LAND without height gate.`
  - 对应目标：G4（可观测性）
  - 原因：运行日志可直接区分本次切换是“高度计门控”还是“无高度计兜底”。

---

## 文件 3：`px4_utils_land/docs/landing_logic_changes.md`

- 新建并更新完整变更说明文档
  - 对应目标：G1、G2、G3（文档留存）
  - 原因：满足“每一步修改都要有清晰、准确文档留存解释”的要求。

---

## 结论

- G1 已覆盖：高度计有效 + 低于阈值触发 `AUTO_LAND`。
- G2 已覆盖：高度计缺失/过期时保持 `SOFT_LAND`。
- G3 已覆盖：RTK 每周期校正，`dx`/`dy` 单轴与向量双重限幅。
- G4 已覆盖：`TRAJ_CMD` 在无高度计时可按 GPS 接近条件切入 `SOFT_LAND`，不再被 `/scan` 阻塞。
- G0 项（视觉停用、历史 z 赋值改动）已在映射中单独标注，不混入本次目标。