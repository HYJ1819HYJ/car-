# 实车三次测试问题整改技术方案（供 Codex 执行，含第一批 + 第二批）

> **文档日期**：2026-11-26  
> **目标仓库**：`/home/asuka/Desktop/college-level competition/xiaoche`（分支 `asuka-main`）  
> **证据来源**：`outputs/competition-fields/mock-1126-1456/` 下的 `map.yaml` 与最近三次实车录包：
> - `bags/attempt-04-20261126-181308-zdzjU1/navigation_0.bag`（253.9 s）
> - `bags/attempt-05-20261126-182130-Xeypl4/navigation_0.bag`（101.7 s）
> - `bags/attempt-06-20261126-182603-7QPbFf/navigation_0.bag`（140.8 s）

---

## 0. 背景、硬约束与不可破坏的不变量

### 0.1 必须保留的本地未提交改动（严禁回退或覆盖）
当前工作区在 `HEAD (6b10b26)` 之上有以下本地未提交改动，**必须完整保留**：
1. 地图选择器与自动场次识别：`tools/competition_terminal.sh`
2. 纯 ROS 侧云台俯仰限位 `[-25°, 0°]` 与初始俯头 `-10°`：`src/stm32_bridge/include/stm32_bridge/gimbal_status.hpp`、`src/stm32_bridge/src/gimbal_bridge_node.cpp`、`src/stm32_bridge/test/gimbal_bridge_parser_test.cpp`、`tools/trigger_race_go.py`、`src/maze_bridge/launch/navigation_stack.launch`、`src/maze_bridge/launch/navigation_visual_handoff.launch`
3. 操作手册：`docs/实车比赛完整操作与自检手册.md`、`docs/实车比赛启动命令.md`
4. 本地 Gazebo 相关文件（`src/competition_sim/...`、`start-competition-gazebo.sh`、`tools/generate_gazebo_competition_worlds.py`）：**不要改动或误提交**。

### 0.2 环境与硬件硬约束
- **ROS 环境**：宿主机 ROS1 Noetic 通过 `bash -ic 'use_ros1 && <命令>'`；竞赛运行镜像通过 `bash tools/competition_terminal.sh prepare` 重建 `outputs/competition-noetic`。禁止混合 ROS1/ROS2，禁止修改系统 Python。
- **仅改 ROS 侧，不动 STM32 固件**：本方案所有改动限定在 ROS 侧（`xiaoche/src` 与 `xiaoche/tools`），不要编译或烧录任何 STM32 工程。
- **车体非对称轮廓**：24 顶点凸多边形（`src/maze_bridge/config/footprint_stl.yaml`），驱动轮轴原点到车头 `+0.3483 m`、到车尾 `-0.1517 m`、半宽 `±0.2450 m`，车头对角回转半径 `0.426 m`（加 `2 cm` 保护带为 `0.45 m`）。
- **车尾 80° 激光雷达完全盲区（关键安全约束）**：三次录包均证实 `/scan_raw` 在 `[-180°, -140°]` 与 `[+140°, +180°]` 扇区有效点数恒为 **`0.0%`**，`/rear_obstacle_distance` 恒为 `inf`。
  - **严禁放开无约束的自由高速/长距盲倒**；
  - 所有自主倒车**必须同时满足**：低速（`<= 0.15 m/s`）、限距（单次连续倒退 `<= 0.40 m`）、完整多边形在代价地图已知自由区（禁止 `LETHAL=254` 和 `NO_INFORMATION=255`）扫掠无碰撞，且保留 `serial_bridge` 对侧后方 `±135°` 实时雷达的全车体轨迹碰撞预测拦截。
- **云台 MCU 现行硬件限制（不可更改）**：`TRACK_PERIOD_MS=40ms`、`MAX_STEP_PER_UPDATE=1.5°`、`SERVO_INTERVAL_MS=80ms`，水平跟踪角速度硬上限仅 **`37.5°/s`**；`STAT` 文本上报约 `1.94 Hz`（每 `~516ms` 阻塞发送 `~8.3ms`）。
- **未标定相机实测焦距偏差**：三次测试在 `0.93 m / 3.22 m / 3.74 m` 三个距离上，`fallback_hfov_deg=88.0°` 算出的视觉水平距离恒为激光真实距离的 `1/1.26`（系统偏短 `21%`），反推真实水平视场角约为 **`75°`**。

---

## 1. 第一批整改（P0 核心：解冻 + 脱困倒车 + 消打转 + 终端交接）

---

### WP1.1：`SmoothGlobalPlanner` 解除 `start_free=false` 死锁 + 4× 加速重规划 + 受控短距倒车基元

#### 问题根因
1. **永久死锁**：`attempt-04 t=95.7~252.5s`（157 s 冻结，1,738 次失败）与 `attempt-06 t=105~139s`（34 s 冻结，359 次失败），车辆真实轮廓（`pad=0`）未碰撞，仅因 `+2cm` 膨胀边轻触静态墙栅格，`smooth()` 与 `footprintReplan()`（`footprint_search.cpp:15-16`）直接因 `!poseFree(start, ..., 0.02)` 拒算。
2. **绕柱超时**：`attempt-05 t=59~69s` 与 `attempt-06 t=20~84s`，`GlobalPlanner` 选了中柱左侧窄缝，多边形检查失败后进入 `footprintReplan()`，其在 `2.5cm` 细网格上做全图 Chebyshev 距离场 BFS 耗尽 `120ms` 预算，未及搜索中柱右侧宽道。
3. **无倒车基元**：`footprint_search.cpp:140` 只有前进与原地转，车头扎入半包围拐角（`attempt-06 t=20~45s`）时原地转必扫墙，数学上无解。

#### 涉及文件
- `src/maze_smooth_planner/include/maze_smooth_planner/smoother.h`
- `src/maze_smooth_planner/src/smoother.cpp`
- `src/maze_smooth_planner/src/footprint_search.cpp`
- `src/maze_smooth_planner/src/planner.cpp`
- `src/maze_bridge/launch/smooth_global_planner_params.yaml`

#### 详细设计

1. **配置项扩展（`smoother.h` + `planner.cpp` + `smooth_global_planner_params.yaml`）**
   在 `maze_smooth_planner::Config` 中新增（默认值保证现有单测行为完全兼容）：
   ```cpp
   bool allow_reverse = false;              // 单测默认 false；launch 配置为 true
   double max_reverse_distance = 0.40;      // 单次连续倒车上限 (m)
   double reverse_cost_multiplier = 4.0;    // 倒车边代价倍率，保证有前向解时绝不乱倒车
   bool allow_start_padding_escape = true;  // 仅当 pad=0 自由而 pad=collision_padding 轻触时允许脱离起步
   ```
   在 `smooth_global_planner_params.yaml` 中配置：
   ```yaml
   replan_compute_ms: 200.0
   allow_reverse: true
   max_reverse_distance: 0.40
   reverse_cost_multiplier: 4.0
   allow_start_padding_escape: true
   ```
   并在 `SmoothGlobalPlanner::initialize()` 中读取上述 4 个参数并做合法性校验（`max_reverse_distance >= 0`, `reverse_cost_multiplier >= 1.0`）。

2. **暴露 `collisionCells` 辅助函数（`smoother.h` + `smoother.cpp`）**
   将 `smoother.cpp:66` 中原本 `static` 的 `collisionCells(...)` 声明到 `namespace maze_smooth_planner` 的 `smoother.h` 中（或新增 `int footprintCollisionCells(const Pose&, const costmap_2d::Costmap2D&, const std::vector<geometry_msgs::Point>&, double padding)`），供 `footprint_search.cpp` 和 `planner.cpp` 复用，避免重复实现 SAT 碰撞计算。

3. **4× 加速 `footprintReplan()` 距离场计算（`footprint_search.cpp:35-56`）**
   - **当前瓶颈**：`clearance` BFS 运行在全分辨率 `mw × mh`（`2.5cm`）上，而搜索网格是 `2×` 粗网格 `nx × ny`（`step = 2 * res = 5cm`）。
   - **改造方案**：
     - 直接在 `nx × ny` 粗网格上建 `coarse_clearance`（大小 `nx * ny`，细胞数减少 **4 倍**）：
       - 粗格 `(cx, cy)` 只要边界上或其对应的 `2×2` 细格 `(2*cx+ox, 2*cy+oy)` 中任一细格满足 `cost >= 254`（含 `LETHAL_OBSTACLE=254` 与 `NO_INFORMATION=255`），即标记 `coarse_clearance = 0` 并入队。
       - 8 邻域 BFS 求粗格 Chebyshev 距离。
     - `room(p)` 保守下界：`coarse_clearance[cy * nx + cx] * step - step * std::sqrt(2.0)`（因为粗格量化最多引入 `1` 个粗格对角线误差，绝不会把靠近障碍的姿态误判为安全！近墙时依然走全分辨率 `2.5cm` 精确 `poseFree`/`pathFree`）。
     - `guide_deadline` 保持 `std::min(8.0, 0.12 * cfg.replan_compute_ms)`。

4. **解除 `start_free=false` 死锁（`footprint_search.cpp:15-16, 120-167` + `planner.cpp:72-86`）**
   - **严格安全边界**：
     - 如果 `!poseFree(goal, map, body, cfg.collision_padding)` -> 仍然返回 `false`（目标点必须保留完整保护带）。
     - 如果 `!poseFree(start, map, body, 0.0)` -> 仍然返回 `false`（**真实物理车身已压入障碍或越界时严禁强行规划**，同时保证现有单测 `BlockedStartAndInvalidBudgetAreRejected` 通过）。
     - 仅当 `!poseFree(start, map, body, cfg.collision_padding)` **且** `cfg.allow_start_padding_escape` **且** `poseFree(start, map, body, 0.0)` 时，进入 **Start Padding Escape**：
       - 在 `start` 周围生成候选脱离动作序列（无需复杂搜索，直接确定性评估）：
         - 候选 1：沿 8 个方向/前后微移 `{+0.05, +0.10, +0.15, -0.05, -0.10, -0.15, -0.20, -0.25}` m（仅当 `cfg.allow_reverse` 为真时允许负位移）和原地微转 `{±10°, ±20°, ±30°}`；
         - 选出满足以下全部条件的**最短脱离位姿 `escape_pose`**：
           1. 整个扫掠段 `pathFree({start, escape_pose}, map, body, 0.0)` 为真（真实车体零碰撞，不含未知格）；
           2. 沿途每步的 `footprintCollisionCells(..., cfg.collision_padding)` **单调非增**（绝不越扎越深）；
           3. 终点 `poseFree(escape_pose, map, body, cfg.collision_padding)` 为真（已完全脱离保护带！）。
       - 若找到 `escape_pose`：
         - 以 `escape_pose` 作为 `footprintReplan(escape_pose, goal, ...)` 的起点求后续全保护带路径 `tail`；
         - 将 `start` 插回 `tail` 头部组成完整路径 `out = [start, escape_pose, ...]`；
         - 最终独立校验：`pathFree({start, escape_pose}, map, body, 0.0)` **且** `pathFree(tail, map, body, cfg.collision_padding)`。
   - **诊断增强（`planner.cpp:78-85`）**：在 `/move_base/SmoothGlobalPlanner/diagnostics` JSON 中追加 `"start_unpadded_free":true/false` 与 `"start_escaped":true/false`。

5. **受控短距倒车基元（`footprint_search.cpp:139-160`）**
   - 当 `cfg.allow_reverse && cfg.max_reverse_distance >= step` 时，在 `footprintReplan()` 中扩展移动基元：
     - `move = 0`：前进 `1` 粗格（`+dx[h], +dy[h]`），重置连续倒车计数；
     - `move = 1, 2`：原地左/右转 `45°`，保持连续倒车计数；
     - `move = 3`：后退 `1` 粗格（`-dx[h], -dy[h]`），仅当**连续倒车距离 `<= cfg.max_reverse_distance`** 时允许扩展！
   - 状态表示：为避免状态空间爆炸，可在 `std::vector<double> rev_used(count, 0.0)` 中记录到达状态 `id` 时使用的连续倒车距离（前进一格清零；或仅在 `distance[id] < 1.2m` 的起点脱困区内允许倒车，或者允许全程但限制连续 `rev_used <= cfg.max_reverse_distance`）。推荐使用 `rev_used[id]` 记录到达 `id` 的连续倒车米数，当 `move == 3` 时要求 `rev_used[id] + step_len <= cfg.max_reverse_distance + 1e-6`，且边权为：
     ```cpp
     double edge = step_len * cfg.reverse_cost_multiplier;
     ```
     同时保留完整的 `free(next)` + `pathFree({a, b}, map, body, cfg.collision_padding)` 扫掠检查（自动禁止进入 `NO_INFORMATION=255` 或障碍）。
   - **向后兼容**：`Config::allow_reverse` 默认为 `false`，所有未显式开启倒车的旧单测（含 `RecordedTurnFindsDifferentSweptSafeRoute`）行为 100% 不变。

---

### WP1.2：可控安全自主倒车贯通局部控制层与串口安全层 + 修复 `front_slow` 原地转误降速

#### 问题根因
1. `ForwardTebLocalPlanner`（`forward_teb.cpp:33-52`）、`teb_local_planner_params_v2.yaml`、`serial_bridge_node.cpp:345` 三层全部硬锁禁倒车。
2. `serial_bridge_node.cpp:393-401` 在 `v > 1e-3 && front_clearance < 0.60` 时等比例缩减 `w`，导致 TEB 发出 `v=0.01, w=-0.45` 的近似原地转被压成 `-0.08 rad/s`，原地慢磨 `8.6 秒`（`attempt-04 t=82.8~91.4s`）。

#### 涉及文件
- `src/maze_smooth_planner/src/forward_teb.cpp`
- `src/maze_bridge/launch/teb_local_planner_params_v2.yaml`
- `src/maze_bridge/launch/navigation_stack.launch`
- `src/maze_bridge/launch/navigation_visual_handoff.launch`
- `src/stm32_bridge/src/serial_bridge_node.cpp`

#### 详细设计

1. **`ForwardTebLocalPlanner` 受控安全倒车放行（`forward_teb.cpp`）**
   - 新增私有参数（在 `initialize()` 中从 `ros::NodeHandle nh("~/" + name)` 读取）：
     - `allow_controlled_reverse`（默认 `false`；在 `navigation_stack.launch` 中透传 arg `allow_controlled_reverse`，比赛默认 `true`）
     - `max_reverse_speed`（默认 `0.15` m/s，硬上限 `0.20`）
     - `max_continuous_reverse_dist`（默认 `0.40` m）
     - `reverse_check_horizon`（默认 `0.60` s）
     - `reverse_collision_padding`（默认 `0.02` m）
   - 维护车辆在 `local_costmap` 坐标系下的连续倒车累计距离 `continuous_reverse_dist_` 与上次位置 `last_pose_`：
     - 每次 `computeVelocityCommands()` 获取当前位姿 `start`：计算沿车头方向的投影位移 `ds = (start.x - last.x)*cos(last.yaw) + (start.y - last.y)*sin(last.yaw)`；
     - 若 `ds > 0.0`（前进了），则 `continuous_reverse_dist_ = std::max(0.0, continuous_reverse_dist_ - ds)`（每前进 20cm 以上逐步恢复倒车额度）；若 `ds < 0.0`（倒退了），则 `continuous_reverse_dist_ += -ds`。
     - `setPlan()` 收到新全局路径且目标变化明显时重置计数保护（或保留连续倒车计数以防反复发目标无限盲倒——**必须保留连续倒车计数，仅靠真实向前位移消减**！）。
   - 当 `proposed.linear.x < 0` 时：
     1. 若 `!allow_controlled_reverse_` 或 `continuous_reverse_dist_ >= max_continuous_reverse_dist_` -> 走原有原地转向回退逻辑（`checked_turn_in_place` / `reverse_rejected_turn_blocked`）。
     2. 否则构造受控倒车候选指令：
        - 按 `max_reverse_speed_` 保曲率限幅：若 `proposed.linear.x < -max_reverse_speed_`，令 `scale = -max_reverse_speed_ / proposed.linear.x`，`cand_v = -max_reverse_speed_`，`cand_w = proposed.angular.z * scale`；
        - 按常量 `(cand_v, cand_w)` 积分未来 `reverse_check_horizon_`（每 `0.05s` 采样一点，构造 `std::vector<Pose> arc`），在 `local_costmap` 快照上调用 `pathFree(arc, snapshot, map_->getUnpaddedRobotFootprint(), reverse_collision_padding_)`：
          - `pathFree` 会拒绝任何 `>= 254`（包含 `LETHAL=254` 和 `NO_INFORMATION=255`），确保车尾不会倒进障碍或代价地图未知区！
        - 若 `arc` 全部通过：输出 `command.linear.x = cand_v; command.angular.z = cand_w;`，发布诊断 `report("controlled_reverse", proposed)`，返回 `true`。
        - 若 `arc` 未通过：降级尝试纯原地转 `checked_turn_in_place`；仍不通则返回 `false`（`report("reverse_rejected_blocked", proposed)`）。

2. **TEB 参数调优（`teb_local_planner_params_v2.yaml`）**
   ```yaml
   allow_init_with_backwards_motion: true
   max_vel_x_backwards: 0.15
   # 保留显著前向偏好，常规走廊仍优先前进；仅当半包围拐角无法前向脱困时才允许短距倒车
   weight_kinematics_forward_drive: 40.0
   ```
   > 确认 `penalty_epsilon: 0.005 < max_vel_x_backwards (0.15)` 满足 TEB 约束。

3. **`serial_bridge_node.cpp` 受限自主倒车 + 修复 `front_slow` 误减速**
   - 新增私有参数 `navigation_reverse_max_speed`（默认 `0.15`，硬上限 `0.20`）。
   - 修改 `serial_bridge_node.cpp:345-363`：
     - 当 `control_source == "cmd_vel" && v < -1e-3`:
       - 若 `!navigation_allow_reverse_`：保持原逻辑（清零 `v, w`，`safety_stopped = true`，`+navigation_reverse_blocked`）。
       - 若 `navigation_allow_reverse_`：按 `navigation_reverse_max_speed_` 保曲率限幅（若 `v < -navigation_reverse_max_speed_`，`scale = -navigation_reverse_max_speed_ / v; v = -navigation_reverse_max_speed_; w *= scale;` 并追加 `+nav_reverse_limited`），随后正常进入下方 `cmd_max_w_` 限幅、`rear_clearance_ <= safety_rear_stop_distance_` 检查以及 `trajectoryBlocked(v, w)` 完整车体实时雷达扫掠检查。
   - 修改 `serial_bridge_node.cpp:376-409`（修复近似原地转被 `front_slow` 拖成 9 秒慢爬）：
     ```cpp
     const bool near_pure_rotation =
         control_source == "cmd_vel" && std::fabs(v) < 0.03 && std::fabs(w) > 0.10;
     ```
     - 当 `v > 1e-3 && front_clearance_ <= safety_stop_distance_` 且 `near_pure_rotation`：仅清零 `v = 0.0`，**保留 `w`** 交给后面的 `trajectoryBlocked(0.0, w)` 做完整多边形旋转扫掠安全判定（不再因为 1cm/s 的前向微抖把整条原地转指令掐死）。
     - 当 `v > 1e-3 && front_clearance_ < safety_slow_distance_` 且 `near_pure_rotation`：直接设 `v = 0.0`（消除低速静摩擦前拱），**不缩放 `w`**，由 `trajectoryBlocked(0.0, w)` 独立兜底；常规前向弧线（`v >= 0.03`）保持原有的 `v *= scale; w *= scale;` 保曲率减速不变。
   - 在 `navigation_stack.launch` 中新增 arg：
     - `<arg name="navigation_allow_reverse" default="false"/>`（保持独立启动 `navigation_stack.launch` 时默认关闭，不破坏安全基线）；
     - `<arg name="allow_controlled_reverse" default="$(arg navigation_allow_reverse)"/>`；
     - 透传到 `serial_bridge` 的 `navigation_allow_reverse` / `navigation_reverse_max_speed (0.15)` 以及 `move_base/ForwardTebLocalPlanner/allow_controlled_reverse`。
   - 在 `navigation_visual_handoff.launch` 中新增 `<arg name="allow_controlled_reverse" default="true"/>` 并传给 `navigation_stack.launch` 的 `navigation_allow_reverse`。

---

### WP1.3：`blue_goal_navigator` & `corridor_explorer` 入口连通性校验、APPROACH 停滞早退与防目标抖动

#### 问题根因
1. `attempt-05 t=69.7~100.8s`：`selectFinishEntrance()` 把入口选在静态地图左墙外未知区（`370x raw_plan_unavailable`）。
2. `attempt-06 t=51.3~84.6s`：`plannerDiagnosticsCb()` 只监听 `GoalKind::SEARCH`，导致不可达 `APPROACH` 目标白白冻结 33 秒。
3. `attempt-04/06`：`goal_update_period=0.60s, goal_update_distance=0.12m` 导致 `APPROACH_GOAL` 高频跳变，全局路线左右翻跳引发 180° 反复打转。

#### 涉及文件
- `src/maze_planner/include/maze_planner/corridor_explorer.h`
- `src/maze_planner/src/blue_goal_navigator.cpp`
- `src/maze_bridge/launch/navigation_visual_handoff.launch`

#### 详细设计

1. **`CorridorGrid` 连通性辅助函数 + `selectFinishEntrance()` 过滤孤岛与静态地图外区域（`corridor_explorer.h`）**
   - 在 `maze_planner` 中提供或公开一个从机器人当前点 `robot` 出发的连通区计算函数 `reachableMask(const CorridorGrid& grid, const Point2& robot)`（基于 `CorridorExplorer::distances` 或简单 4/8 邻域 BFS，通行判据 `cost[i] >= 0 && cost[i] < 99`，起点 `robot` 若位于 `99` 允许先从其周围 `3×3` 自由格起步）：
   - 扩展 `selectFinishEntrance()`（保持旧参数默认值兼容现有单测）：
     ```cpp
     inline bool selectFinishEntrance(
         const CorridorGrid& grid, const WallObservation& wall,
         const std::vector<Point2>& footprint,
         const std::vector<Point2>& failed, Point2* goal,
         double padding = 0.0, const Point2* robot = nullptr,
         double robot_yaw = 0.0, bool alternate_view = false,
         const nav_msgs::OccupancyGrid* static_map = nullptr,
         bool require_reachable = false)
     ```
   - 当 `static_map != nullptr` 时：把候选点 `p` 投影到 `static_map`，要求其位于地图内且 `static_map->data[idx] == 0`（拒绝 `-1` 未知区与 `>= 50` 静态墙）。
   - 当 `require_reachable && robot != nullptr` 时：用一次 BFS 计算 `from_robot` 可达掩码，**直接剔除与当前车位不连通的候选入口点**（彻底杜绝墙后孤岛）。

2. **`APPROACH` 规划连续失败早退机制（`blue_goal_navigator.cpp:1183-1193` & `runEntrance()`）**
   - 修改 `plannerDiagnosticsCb()`：
     ```cpp
     if (!goal_active_ || (goal_kind_ != GoalKind::SEARCH && goal_kind_ != GoalKind::APPROACH)) return;
     ```
   - 在 `sendMoveBaseGoal()` 时重置 `plan_failure_since_ = ros::Time();` 并记录 `approach_progress_pose_ = robot; approach_progress_time_ = now;`（或在 `runEntrance()` 中发送 `APPROACH` 目标时初始化）。
   - 在 `runEntrance()` 中，当 `goal_active_ && goal_kind_ == GoalKind::APPROACH && !action.isDone()` 时：
     - 若车移动 `>= 0.08m`，刷新 `approach_progress_pose_` 和 `approach_progress_time_`；
     - 判断：
       ```cpp
       const bool approach_stalled = !plan_failure_since_.isZero() &&
           (now - last_plan_failure_).toSec() <= 1.5 &&
           (now - plan_failure_since_).toSec() >= 3.0 &&
           (now - approach_progress_time_).toSec() >= 3.5;
       ```
     - 若 `approach_stalled` 为真：记录 `ROS_WARN("Approach entrance (%.2f, %.2f) stalled with continuous invalid plans", ...)`，立即取消当前 `move_base` 目标，将该入口加入 `rejected_entrances_` 并立即选下一个候选入口（或满 4 个后立刻回归探索），不再干等 `8.2s`。

3. **降低 `APPROACH` 目标高频抢占抖动（`navigation_visual_handoff.launch:103-104` + `blue_goal_navigator.cpp:1443`）**
   - 在 `navigation_visual_handoff.launch` 中调整：
     - `goal_update_distance`: `0.12` -> `0.25`
     - `goal_update_period`: `0.60` -> `1.50`
   - 在 `runEntrance()` 的刷新分支（`blue_goal_navigator.cpp:1416-1444`）：
     - 若 `maze_planner::norm(robot - current) <= 1.0` 且 `current` 仍然 `footprintClear` 且当前目标已锚定墙面（`locked_target_.wall_valid`），跳过小幅度目标刷新（除非新旧距离 `>= 0.30m` 或角度差 `>= 0.25 rad`），避免临门一脚被来回抢占。

---

### WP1.4：`visual_handoff_manager` 终端交接死锁解除与拒绝原因可观测性

#### 问题根因
`attempt-04 t=96~252s` 车距蓝纸仅 `0.88m`、正对蓝纸（`pan=-0.4°`），但因墙面与车身夹角 `35.4° > 17.2°`（`0.30 rad`）且 `lateral=0.69m > 0.32m`，`visual_handoff_manager` 拒绝交接，而 `move_base` 已卡死。同时 `corridorReady` 的拒绝原因用 `ROS_DEBUG_THROTTLE` 打印，录包中完全看不到原因。

#### 涉及文件
- `src/maze_planner/src/visual_handoff_manager.cpp`
- `src/maze_bridge/launch/navigation_visual_handoff.launch`

#### 详细设计
1. **新增可调参数（`visual_handoff_manager.cpp`）**：
   - `max_handoff_lateral`（默认 `0.38` m，行进间交接横向偏置上限）
   - `max_handoff_yaw_rad`（默认 `0.40` rad ≈ `23°`，行进间交接墙法向夹角上限）
   - `entrance_max_handoff_lateral`（默认 `0.75` m，入口到达或近距滞留时交接横向偏置上限）
   - `entrance_max_handoff_yaw_rad`（默认 `0.65` rad ≈ `37°`，入口到达或近距滞留时交接墙法向夹角上限）
2. **分级交接条件（`visual_handoff_manager.cpp:238-257`）**：
   - `close_and_aligned`（行进间交接）：
     `norm(wall.marker) <= max_handoff_distance_ && lateral <= max_handoff_lateral_ && (!competition_mode_ || std::abs(parkingYaw(wall)) <= max_handoff_yaw_rad_)`
   - `entrance_or_stalled_close`（已到入口，或进入 `APPROACH:` 且近距 `norm <= max_handoff_distance_ && depth <= 1.15` 停滞超过 `1.5s`）：
     `nav_state_.find("APPROACH: entrance reached") == 0 || nav_state_.find("APPROACH: waiting for a clear") == 0` 且 `depth <= 1.15 && norm(wall.marker) <= max_handoff_distance_ && lateral <= entrance_max_handoff_lateral_ && (!competition_mode_ || std::abs(parkingYaw(wall)) <= entrance_max_handoff_yaw_rad_)`
   - **安全不变量不变**：两种路径都必须继续通过 `handoffCorridorClear(...)` 完整扫掠+激光可见性校验，并且连续 `required_clear_scans_`（3 帧）通过，才切换给 `direct_visual_controller`（其内置 `wallAlignedSteeringBearing` + `final_yaw_align` + `reactivePathClear` 会完成斜角对准与最终入框停靠）。
3. **升级日志可观测性（`visual_handoff_manager.cpp:304`）**：
   - 当 `nav_state_.find("APPROACH:") == 0` 且 `corridorReady(&reason)` 返回 `false` 时，将 `ROS_DEBUG_THROTTLE(1.0, ...)` 改为 `ROS_INFO_THROTTLE(2.0, "Terminal handoff waiting: %s", reason.c_str())`（并在 `reason` 中带出关键数值如 `norm/depth/lat/yaw`），确保后续任何测试录包都能一眼定位交接门限状态。

---

## 2. 第二批整改（云台速度匹配 + 测距校准 + 假目标过滤 + CPU 减负）

---

### WP2.1：底盘与云台角速度匹配 + 接近期自动降速 + `/odom` 角速度前馈 + 串口拍频避让

#### 问题根因
1. 底盘峰值转向 `80°~91°/s`、线速 `1.25 m/s`，远超未改 MCU 云台 `37.5°/s` 的物理跟踪上限，导致三场测试 `BLUE_LOST` 达 131 / 54 / 61 次。
2. 静止时每 `2.58s`（`5 × 516ms = 43 × 60ms`）ROS 发的 6 字节 `0xAA` 视觉包与 MCU 阻塞式 `HAL_UART_Transmit` `STAT` 撞车，诱发规律 `EVT,BLUE_LOST`。

#### 涉及文件
- `src/maze_bridge/launch/navigation_visual_handoff.launch`
- `src/maze_planner/src/visual_handoff_manager.cpp`
- `src/stm32_bridge/src/gimbal_bridge_node.cpp`
- `src/stm32_bridge/include/stm32_bridge/gimbal_status.hpp`（如需把前馈纯函数抽出方便单测）

#### 详细设计

1. **巡航速度下调匹配云台能力（`navigation_visual_handoff.launch:16-17`）**
   - `max_speed`: `1.00` -> `0.60` m/s
   - `max_turn`: `1.00` -> `0.60` rad/s（`34.4°/s`，落入云台 `37.5°/s` 可跟踪区间内）
   - 新增参数：`approach_max_speed` 默认 `0.35` m/s、`approach_max_turn` 默认 `0.40` rad/s（`22.9°/s`），传给 `visual_handoff_manager`。

2. **接近期自动保曲率降速（`visual_handoff_manager.cpp:126-128`）**
   - 在 `VisualHandoffManager::navCmdCb()` 中，当 `mode_ == Mode::NAVIGATION` 且 `nav_state_.find("APPROACH:") == 0` 时，对转发到 `/cmd_vel` 的 `Twist` 执行**保曲率限幅**：
     ```cpp
     geometry_msgs::Twist out = *msg;
     if (nav_state_.find("APPROACH:") == 0) {
       const double vs = std::abs(out.linear.x) > approach_max_speed_
           ? approach_max_speed_ / std::abs(out.linear.x) : 1.0;
       const double ws = std::abs(out.angular.z) > approach_max_turn_
           ? approach_max_turn_ / std::abs(out.angular.z) : 1.0;
       const double scale = std::min(vs, ws);
       out.linear.x *= scale;
       out.angular.z *= scale;
     }
     command_pub_.publish(out);
     ```
   - 这样不改变规划器轨迹曲率，同时保证一旦锁定蓝纸开始转向/接近，底盘角速度被稳稳压在 `<= 23°/s`，云台不会甩丢目标。

3. **云台水平角速度前馈补偿（`gimbal_bridge_node.cpp` + `gimbal_status.hpp`）**
   - 在 `GimbalBridgeNode` 中订阅 `/odom`（`nav_msgs/Odometry`），记录最新 `odom_wz_` 与接收时间 `last_odom_time_`（超过 `0.30s` 视为无效清零）。
   - 新增参数：
     - `yaw_feedforward_sec`（默认 `0.12` s，`0.0` 可随时关闭）
     - `max_yaw_feedforward_px`（默认 `120.0` px）
   - 在 `transmitObservation()` 中，计算 `source_x` **之前**（在原始未镜像图像坐标系中）叠加前馈：
     - **物理符号核对**：车体向左转（ROS `odom_wz_ > 0`）时，静止目标在相机画面中向**右**移动（`center_x` 增大），因此预测目标像素横坐标为：
       ```cpp
       const double ff_px = clamp(fx * fresh_odom_wz * yaw_feedforward_sec_,
                                  -max_yaw_feedforward_px_, max_yaw_feedforward_px_);
       double source_x = clamp(static_cast<double>(latest_observation_.center_x) + ff_px,
                               0.0, static_cast<double>(latest_observation_.image_width - 1U));
       ```
     - 随后再走原有的 `invert_target_x_` 镜像与 `640` 缩放，以及 `GimbalTiltController::compute()`。
   - **抽取纯函数**（如 `stm32_bridge::applyYawFeedforwardPx(...)` 到 `gimbal_status.hpp`），便于在 `gimbal_bridge_parser_test.cpp` 中做符号与限幅单元测试。

4. **ROS 侧 `STAT` 窗口发包避让（缓解 `2.58s` 周期误报 `BLUE_LOST`）（`gimbal_bridge_node.cpp:212-250`）**
   - 新增参数 `avoid_status_tx_window`（默认 `true`）与 `status_period_est_sec`（默认 `0.516` s）。
   - 在 `transmitObservation()` 中：若 `avoid_status_tx_window_` 为真且当前距上次收到 `STAT` 的时间 `since_stat = (now - last_status_time_).toSec()` 落在 `[status_period_est_sec_ - 0.025, status_period_est_sec_ + 0.025]`（即 MCU 即将阻塞发送 `STAT` 的 `±25ms` 敏感窗），本次跳过发送（`20ms` 后下一拍再发），避开拍频碰撞；由于 `VISION_TIMEOUT_MS=500ms`，延迟 1 个 `20ms` 周期完全安全。
   - *(注：物理层位翻转如 `eVT,BLUE_LOST` 仍需电控侧后续查串口线/共地，及 MCU 侧改为非阻塞 UART1 发送彻底根治。)*

---

### WP2.2：统一校准 `fallback_hfov_deg`（`88° → 75°`）与场外假目标空间收紧

#### 问题根因
1. `fallback_hfov_deg=88.0°` 导致全距离段视觉测距系统偏短 `21%`，且 `visual_handoff_manager` 节点甚至漏传了该参数。
2. 场外右上角蓝色物体（估算高度 `z=0.42~0.61m`，位于静态地图边缘外）在 `attempt-05/06` 中仍通过了 `max_target_z=0.72` 与 `insideMapMargin=0.50m`，把车引诱进死胡同。

#### 涉及文件
- `src/maze_bridge/launch/navigation_visual_handoff.launch`
- `src/maze_planner/src/blue_goal_navigator.cpp`

#### 详细设计
1. **统一 `fallback_hfov_deg` 参数源（`navigation_visual_handoff.launch`）**：
   - 在顶部新增 `<arg name="fallback_hfov_deg" default="75.0"/>`；
   - 统一传递给 **4 个消费者**（一个都不能漏）：
     1. `<include file="$(find maze_bridge)/launch/navigation_stack.launch">` 的 `fallback_hfov_deg`（供 `gimbal_bridge`）；
     2. `handoff_blue_navigator` 的 `fallback_hfov_deg`；
     3. `handoff_direct_visual` 的 `fallback_hfov_deg`；
     4. `visual_handoff_manager` 的 `fallback_hfov_deg`（**补齐原缺失的参数**）。
2. **收紧目标物理高度与地图外容差**：
   - 在 `navigation_visual_handoff.launch` 中把 `handoff_blue_navigator` 的 `max_target_z` 从 `0.72` 收紧为 `0.52`（真实蓝纸底边 `0.15m`、中心 `0.2985m`、顶边 `0.447m`，`0.52m` 保留了 `>7cm` 俯仰测量余量，同时拦掉 `0.55m+` 蓝垃圾桶/售货机）。
   - 在 `blue_goal_navigator.cpp:822-831` 将 `insideMapMargin()` 中写死的 `.50` 参数化为 `map_target_margin`（默认 `0.50` 保持兼容，在 `navigation_visual_handoff.launch` 中设为 `0.15`），拦截墙外蓝物。

---

### WP2.3：`move_base` 控制频率减负与 `robotPose()` 短时 TF 外推容错

#### 问题根因
1. `attempt-05` 出现 87 次 `Control loop missed 12Hz`（单周期达 `0.20s`）与 40 次 `Map update missed`（达 `0.255s`）。
2. `odom_publisher`（20 Hz = `50ms`）慢于相机（30 Hz = `33ms`），导致 `blue_goal_navigator::robotPose()` 每场报 16~63 次 `1~16ms` 未来时间戳 TF 外推异常并丢帧。

#### 涉及文件
- `src/maze_bridge/launch/navigation_stack.launch`
- `src/maze_bridge/launch/local_costmap_params.yaml`
- `src/maze_planner/src/blue_goal_navigator.cpp`

#### 详细设计
1. **频率调优**：
   - `navigation_stack.launch:388`：`controller_frequency` 从 `12.0` 调至 `10.0`；
   - `local_costmap_params.yaml:8`：`update_frequency` 从 `12.0` 调至 `10.0`。
2. **TF 极短超前时间戳容错（`blue_goal_navigator.cpp:543-572`）**：
   - 保持精确时间查询优先；当捕获 `tf2::ExtrapolationException` 且 `stamp` 处于 `now ± 0.06s` 内时，等待 `0.025s` 重试，若仍超前则回退取最新变换 `ros::Time(0)`，并允许 `|stamp - transform.header.stamp| < 0.06s`（原 `0.05s` 刚好卡在 20Hz 里程计的 `50ms` 边界上），避免正常 `1~16ms` 相机/编码器相位差导致频繁丢帧与告警刷屏。

---

## 3. 验收标准与必须执行的测试矩阵

Codex 完成所有修改后，必须逐项跑通以下全部验证并汇报真实输出，**零错误、零失败、不跳过**：

### 3.1 新增单元测试（必须补齐）
1. **`src/maze_smooth_planner/test/smoother_geometry_test.cpp`**：
   - `StartPaddingGrazeEscapesWithoutPhysicalCollision`：复现 `attempt-04 t=105s`（`pad=0` 无碰撞、`pad=0.02` 轻触 `9` 格），验证 `footprintReplan()` / `SmoothGlobalPlanner` 在 `allow_start_padding_escape=true` 下能成功规划出合法脱离路径，且 `pad=0` 碰撞时仍严格拒绝。
   - `SemiEnclosedCornerEscapesWithBoundedReverse`：复现 `attempt-06 t=25s` 半包围袋口（前向原地转必扫墙，仅后退 `0.25m` 可掉头），验证 `allow_reverse=true` 时能规划出后退距离 `<= 0.40m` 的完整无碰撞路径，且后退段绝不进入未知格（`255`）或障碍格；同时验证开放场景中仍优先前进。
   - `MockRaceNarrowPassageSnapshotsRespectPlanningBudget`：确认 `4×` 距离场优化后原有 `120ms` 预算单测耗时显著下降且全过。
2. **`src/maze_planner/test/corridor_explorer_test.cpp`**（或对应测试文件）：
   - `SelectFinishEntranceRejectsDisconnectedPocketAndStaticUnknown`：复现 `attempt-05 t=75s`（`global_costmap` 局部自由但与机器人不连通、且静态 `/map == -1`），验证 `selectFinishEntrance` 拒绝墙后孤岛并选择连通区合法入口。
3. **`src/maze_planner/test/handoff_corridor_test.cpp` / `visual_handoff_ros_mock.py`**：
   - 验证入口到达/近距滞留场景下，斜墙夹角 `35°`、通道扫掠清空时允许触发 `TERMINAL_PARK` 交接，且通道有遮挡时依然严格拒绝。
4. **`src/stm32_bridge/test/gimbal_bridge_parser_test.cpp`**：
   - 验证 `applyYawFeedforwardPx` 在正/负角速度下的方向符号（左转 `wz > 0` 像素右移 `+`）与 `±max_px` 限幅。
5. **`src/stm32_bridge/test/safety_geometry_test.cpp`**（或新增单测）：
   - 验证 `serial_bridge` 近似原地转（`v=0.01, w=-0.45`, `front_clearance=0.22m`）不再把 `w` 误缩放到 `-0.08`，且受控倒车在超速时保曲率限幅到 `-0.15 m/s`、在 `trajectoryBlocked` 碰撞时立即全停。

### 3.2 全量构建与回归命令
1. **宿主机工作区编译与全量单测**：
   ```bash
   bash -ic 'use_ros1 && cd "/home/asuka/Desktop/college-level competition/xiaoche" && catkin_make -j4 && catkin_make run_tests && catkin_test_results build'
   ```
   要求：**0 errors, 0 failures**。
2. **竞赛容器工作区重建**：
   ```bash
   cd "/home/asuka/Desktop/college-level competition/xiaoche" && bash tools/competition_terminal.sh prepare
   ```
3. **Launch 参数解析校验**：
   使用 `roslaunch --dump-params` 验证 `navigation_visual_handoff.launch` 下所有新增/修改参数（`fallback_hfov_deg=75.0` 四节点一致、`max_speed=0.60`、`max_turn=0.60`、`navigation_allow_reverse=true`、`allow_controlled_reverse=true`、`max_target_z=0.52`、`controller_frequency=10.0`、`update_frequency=10.0`、`replan_compute_ms=200.0`）全部生效且无语法错误。
4. **隔离场景回归**：
   跑通现有全部隔离仿真场景（含假目标证伪、窄口避障、墙面锚定交接、整车入框静止 `>= 3s`），确认无任何碰撞或回归。

---

## 4. 实车复测前的现场准备（非代码改动）

完成上述 ROS 整改并验证通过后，实车测试前请先做两件事：
1. **重新建图并保存新场地地图**（`mock-1126-1456` 左墙已与当前场地不一致，禁止继续复用该旧图测试）。
2. 如后续安排电控组更新 MCU 固件（基于 `32端` UART4 版，**严禁**用旧 `CAN1` 目录），再一并落地：云台 `TRACK_PERIOD_MS=20ms / MAX_STEP=3.5° / SERVO_INTERVAL=35ms`、`10Hz` 非阻塞 `STAT` + `*XX` 校验、以及云台 `USART1` 线缆共地排查。
