# 比赛末段终点识别提速与轨迹对准优化技术方案（供 Codex 执行）

> **文档日期**：2026-11-27  
> **ROS 目标仓库**：`/home/asuka/Desktop/college-level competition/xiaoche`（分支 `asuka-main`，基线 `HEAD 26176e5`）  
> **STM32 参考工程**：`/home/asuka/Desktop/college-level competition/firmware/STM32_1NEW_UART5_PC12_PD2/Core/Src/gimbal_tracker.c`（现场烧录时请核对实际使用的 Keil 工程目录，严禁误改旧 `CAN1` 目录）  
> **启动入口**：`bash tools/competition_terminal.sh ready ...` + `bash tools/competition_terminal.sh go "$FIELD"`  
> **证据来源**：
> - 地图：`outputs/competition-fields/match-1127-1228/{map.yaml, map.pgm, start_pose.json}`（`match-01` 复用此图）
> - 实车录包：`outputs/competition-fields/match-01/bags/attempt-01` ～ `attempt-06`（6 轮全部完赛，实测行驶耗时 `19.5s ~ 23.0s`，极端三箱窄道 `attempt-06` 为 `57.8s`）
> - 现场实测确认：蓝色 A4 纸中心离地高度 **`15 cm`**；`trigger_race_go.py` 显示俯角为正（ROS 约定下俯为正），实际镜头亦处于下俯状态；现场支持修改并重新烧录 STM32 云台固件。

---

## 0. 背景、量化根因与不可破坏的不变量

### 0.1 必须保留的本地未提交改动（严禁回退或覆盖）
当前 `xiaoche` 工作区在 `HEAD (26176e5)` 之上有以下本地未提交改动，**必须完整保留**：
1. `tools/competition_terminal.sh`、`tools/trigger_race_go.py`
2. `docs/实车比赛启动命令.md`、`docs/实车比赛完整操作与自检手册.md`
3. 本地 Gazebo 仿真文件（`src/competition_sim/...`、`start-competition-gazebo.sh`、`tools/generate_gazebo_competition_worlds.py`）：**不要改动或误提交**。

### 0.2 录包量化诊断结论
对 `attempt-01`～`attempt-06` 共 6 轮完赛录包进行离线回放与几何回归，定位出拖慢“右转过弯后终点识别与对准”的 5 条因果链：

| 序号 | 根因 | 录包量化证据 | 预计单轮收益 |
| --- | --- | --- | --- |
| 1 | **云台/相机俯仰零偏 `+3.23°`** 导致 `plausibleTargetHeight()` 误拒所有 `d > 2.65 m` 的真终点 | 对 1,855 帧已确认蓝纸观测回归 $d\tan(\text{down}) = 0.2712 + 0.05641\,d$（RMS `2.95 cm`）：$\Delta z = 0.271\text{ m} \Rightarrow z_{\text{paper}} = 0.42 - 0.271 = 0.149\text{ m}$（与现场实测 `15 cm` 误差仅 `1 mm`），下俯偏置 `+3.23°`。导致 `attempt-02/03/05/06` 在 `4.0~5.9 m` 处虽已稳定看到蓝纸，却被连续拒掉 **112 / 196 / 162 / 172 帧**（延迟锁定 **4.9s～6.6s**） | 提前 **4～6.6 s** 锁定目标，且消除绕箱后偏离再回正 |
| 2 | **STM32 云台搜索慢（`31.25°/s`）、幅宽过宽（`±80°`）且右转抵消** | `gimbal_tracker.c` 中 `SEARCH_STEP_DEG=2.5° / 80ms`，扫幅 `160°` 单程 **5.12 s**、往返 **10.24 s**；`attempt-01` 右转时云台向左扫与车身相抵，盯死左墙 **3.5 s**；`attempt-04` 过箱后云台扫到 `+72.5°` 白白浪费 **~5 s**；`STAT` 仅 `~1.94 Hz` 导致转弯期间 `pan` 读数滞后达 `0.5 s` | 扫掠周期从 `10.2s` 压缩至 **`~2.7s`**（**3.7×**），右转不卡外墙，省 **3～5 s** |
| 3 | **隐藏联动陷阱：`APPROACH` 状态全程限速 `0.35 m/s`** | `visual_handoff_manager.cpp:139-145` 只要 `nav_state` 为 `APPROACH:` 即全局压低线速至 `0.35 m/s`、角速至 `0.40 rad/s`。修复根因 1 后会在 `4.5~5.9 m` 处提前锁定，若不改为按距离分段限速，直道会被拖慢 **3～4 s** | 避免提前锁定反减速，并使 `attempt-01` 等直道段省 **1.5～2.5 s** |
| 4 | **`CorridorExplorer` 在宽通道中选点偏左贴角** | `corridor_explorer.h:154` 将 `clearance` 截断在 `1.0 m`，在 `~2.9 m` 宽的末段区域出现平顶，评分退化为纯比 `depth`，6 轮测试全部把初始前沿点选在左下角 `(2.13, -4.88, -114.9°)`（比真实终点 `(2.55, -5.85)` 偏左 `0.45 m`、车头歪 `20°`） | 探索前沿点居中至 `(2.66, -4.45)`，未锁定前直指通道中线 |
| 5 | **末端 `TERMINAL_PARK` 的 `ALIGN ↔ APPROACH` 抖动与过早慢爬** | `attempt-04`（4.54s）与 `attempt-06`（4.99s）在末端出现 2～3 次 `ALIGN ↔ APPROACH` 往返（`turn_before_drive=0.44 rad` 与 `8°` 墙法向阈值均无迟滞），且最后 `0.65 m` 被 `(wall_depth - 0.48) * 0.8` 压成低速爬行 | 消除末端二次停顿，省 **0.8～1.5 s** |

### 0.3 不可破坏的硬约束与安全不变量
1. **ROS 环境规范**：宿主机 ROS1 通过 `bash -ic 'use_ros1 && <命令>'`；竞赛运行镜像通过 `bash tools/competition_terminal.sh prepare` 重建 `outputs/competition-noetic`。禁止混用 ROS1/ROS2，禁止修改系统 Python。
2. **`trigger_race_go.py` 发车前自检不变量**：`trigger_race_go.py:49-54, 210-217` 要求待机时 `/gimbal/joint_states` 中的 `camera_tilt_joint` 与 `tilt_to_ros_sign * default_tilt_deg`（`+10.0°`）偏差 `<= 4.0°`。**因此 `/gimbal/joint_states` 必须继续发布机械舵机角度，不得把 `camera_pitch_offset` 混入 `/gimbal/joint_states` 导致发车前自检误报或 `publishCameraTf()` 重复叠加偏移。**
3. **高处假蓝牌拦截不变量**：现场墙面上方存在蓝色标识牌（实测高度 `> 0.8 m`），`max_target_z = 0.52 m` 必须保留；修正俯仰偏置后，所有 6 轮录包中的远距误检仍须保持 **0 误通过**。
4. **整车安全与终端停车不变量**：不削弱 `serial_bridge` 碰撞预测、不放宽车尾盲区限制、不降低 `handoffCorridorClear` 激光扫掠校验与 `parkingStopReached` 整车入框判定；所有 ROS 侧新增参数在缺省值下保持向后兼容，不破坏现有单元测试。
5. **STM32 协议向后兼容不变量**：新增的任何控制命令均沿用 `0xAB` 帧头 + 1 字节命令码 + 校验和；`STAT` 文本格式字段名保持不变；保持俯仰轴等效比例速度增益不变，不破坏 ROS 侧 `GimbalTiltController`。

---

## 1. 工作包详细设计（WP1 ～ WP5）

---

### WP1：修正相机/云台俯仰零偏（`-0.056 rad` ≈ `-3.2°`），释放 3～6 m 远距锁定能力

#### 涉及文件
- `src/maze_bridge/launch/navigation_visual_handoff.launch`
- `src/maze_bridge/launch/navigation_stack.launch`（确认 `camera_pitch` 透传，已存在第 74、295 行）
- `src/maze_planner/src/blue_goal_navigator.cpp`
- `src/maze_planner/src/visual_handoff_manager.cpp`
- `src/maze_planner/src/direct_visual_controller.cpp`
- `src/maze_planner/test/visual_tracking_test.cpp`（补充高度与偏置单测）

#### 详细设计
1. **统一新增 `camera_pitch_offset` 参数（单位：弧度，ROS 约定正值=下俯，负值=上仰）**
   - 物理标定值：由 6 轮 1,855 帧回归得 `down_reported = down_true + 3.23°`，故 `optical_tilt = joint_tilt + camera_pitch_offset`，取 **`camera_pitch_offset = -0.056` rad（`-3.21°`）**。
   - 符号一致性核验：
     - `gimbal_bridge_node.cpp:266` 已写 `camera_down_deg = tilt_to_ros_sign_ * tilt_deg_ + camera_pitch_offset_ * 180.0 / M_PI;` → 当 `tilt_deg = -10°`、`tilt_to_ros_sign = -1`、`camera_pitch_offset = -0.056 rad` 时，`camera_down_deg = +10.0° - 3.21° = +6.79°`（正确！）。
     - `gimbal_bridge_node.cpp:369` 的 `publishCameraTf()` 已写 `camera_pitch_offset_ + tilt_to_ros_sign_ * degToRad(tilt_deg_)`（REP-103 绕 Y 正转即下俯，完全一致！）。
     - `gimbal_bridge_node.cpp:354` 的 `publishJointState()` **保持不变**（仍发机械舵机角 `+10°`），确保 `trigger_race_go.py` 发车前自检 `|tilt_deg - (+10°)| <= 4°` 完全不受影响，且不会重复扣除偏置。
2. **在三个视觉消费节点中读取并应用 `camera_pitch_offset`（默认值 `0.0` 保证旧单测 100% 兼容）**：
   - **`blue_goal_navigator.cpp`**：
     - 构造函数读取 `pnh_.param<double>("camera_pitch_offset", camera_pitch_offset_, 0.0);`；
     - 在 `targetBearingInBase()`（约 L366）与 `estimateTarget()`（约 L721–739）中，将参与 `visualBearing(...)` 与 `down_angle` 计算的俯仰角统一替换为：
       ```cpp
       const double optical_tilt = tilt + camera_pitch_offset_;
       ```
       即：
       ```cpp
       if (competition_mode_ &&
           !maze_planner::visualBearing(pan, optical_tilt, observation.center_x,
                                        observation.center_y, cx, cy, fx, fy, &bearing))
         return false;
       const double down_angle = optical_tilt + pixel_down_angle;
       ```
   - **`visual_handoff_manager.cpp`**：
     - 构造函数读取 `pnh_.param<double>("camera_pitch_offset", camera_pitch_offset_, 0.0);`；
     - 在 `targetGeometry()`（L210–213）中令 `const double optical_tilt = tilt + camera_pitch_offset_;`，并传给 `visualBearing(pan, optical_tilt, ...)` 与 `const double down = optical_tilt + std::atan2(...)`。
   - **`direct_visual_controller.cpp`**：
     - 构造函数读取 `pnh_.param<double>("camera_pitch_offset", camera_pitch_offset_, 0.0);`；
     - 在 `observationGeometry()`（L191–195）中同样使用 `const double optical_tilt = tilt + camera_pitch_offset_;`。
3. **Launch 参数统一透传（`navigation_visual_handoff.launch`）**：
   - 顶层新增 `<arg name="camera_pitch_offset" default="-0.056"/>`；
   - 统一传递给 **4 个消费者**（与 `fallback_hfov_deg` 一样一个都不能漏）：
     1. `<include file="$(find maze_bridge)/launch/navigation_stack.launch">` 中追加 `<arg name="camera_pitch" value="$(arg camera_pitch_offset)"/>`（传给 `gimbal_bridge`）；
     2. `handoff_blue_navigator`：`<param name="camera_pitch_offset" value="$(arg camera_pitch_offset)"/>`；
     3. `handoff_direct_visual`：`<param name="camera_pitch_offset" value="$(arg camera_pitch_offset)"/>`；
     4. `visual_handoff_manager`：`<param name="camera_pitch_offset" value="$(arg camera_pitch_offset)"/>`。
   - 同时检查 `handoff_blue_navigator` 参数：`camera_z=0.42`、`min_target_z=0.02`、`max_target_z=0.52` 保持不变（离线 6 轮录包全量回放已证实：应用 `-0.056 rad` 后真蓝纸在 `0.6~5.9 m` 全程落在 `0.06~0.25 m`，**6 轮均 0 误通过、提前 4.1~6.6 s 通过高度检查**）。
   - **顺带清理 `gimbal_min_target_down_deg` 检查**：在 `navigation_visual_handoff.launch` 中 `gimbal_min_target_down_deg` 为 `-4.0`，在 `camera_pitch_offset = -0.056` 下，5.9 m 处真蓝纸的 `target_down_deg ≈ +2.6° > -4.0°`，不会被云台俯仰保护拒绝。

---

### WP2：STM32 云台搜索 3.7× 提速、水平追踪 2.6× 提速、10 Hz 非阻塞状态回报 + 弯道内侧搜索引导

#### 涉及文件
- **STM32 固件**：
  - `/home/asuka/Desktop/college-level competition/firmware/STM32_1NEW_UART5_PC12_PD2/Core/Src/gimbal_tracker.c`
  - *(若现场烧录目录另有最新 `32端` Keil 工程，需将同一份 `gimbal_tracker.c` 改动同步过去；严禁使用旧 `CAN1` 工程)*
- **ROS 侧协同**：
  - `src/stm32_bridge/include/stm32_bridge/gimbal_status.hpp`
  - `src/stm32_bridge/src/gimbal_bridge_node.cpp`
  - `src/maze_bridge/launch/navigation_stack.launch`
  - `src/maze_bridge/launch/navigation_visual_handoff.launch`
  - `src/stm32_bridge/test/gimbal_bridge_parser_test.cpp`

#### 详细设计

##### 2.1 STM32 固件侧改动（`gimbal_tracker.c`）
1. **搜索提速与窄幅化（核心）**：
   ```c
   #define SEARCH_START_DELAY_MS      200U    /* 原 500U：过箱遮挡后 0.2s 即恢复搜索 */
   #define SEARCH_PERIOD_MS           50U     /* 原 80U：20 Hz 搜索步进 */
   #define SEARCH_STEP_DEG            3.5f    /* 原 2.5f：3.5°/50ms = 70°/s（原 31.25°/s） */
   #define SEARCH_PAN_MIN            -48.0f   /* 原 -80.0f：叠加 75° HFOV 覆盖 ±85.5° 前半球 */
   #define SEARCH_PAN_MAX             48.0f   /* 原 +80.0f：单程 1.37s，往返 2.74s（原 10.24s，快 3.7×） */
   ```
   - 在 30 fps 相机下，`70°/s` 每帧移动约 `2.3°`，蓝纸滞留 75° 视场超过 `1.07 s`（> 32 帧），远高于 `blue_detector` 的 `confirm_frames = 4`。
   - **安全保护**：若进入搜索时当前 `pan_target_angle` 在 `[-48°, +48°]` 之外（例如刚丢失大侧角目标），先朝 `[-48°, +48°]` 区间内部步进回拉，不要瞬跳。

2. **水平追踪 2.67× 提速，同时保持俯仰轴等效增益不变（防止破坏 ROS `GimbalTiltController`）**：
   ```c
   #define TRACK_PERIOD_MS            25U     /* 原 40U：40 Hz 追踪闭环 */
   #define TRACK_KP_X                 0.018f  /* 原 0.012f：水平跟踪更快收敛 */
   #define TRACK_KP_Y                 0.0075f /* 等效 0.0075/0.025 = 0.30 deg/(s·px)，与旧 0.012/0.040 完全一致！ */
   #define MAX_PAN_STEP_PER_UPDATE    2.5f    /* 2.5°/25ms = 100°/s（原 37.5°/s，轻松跟住车身转向） */
   #define MAX_TILT_STEP_PER_UPDATE   0.94f   /* 0.94°/25ms = 37.6°/s，与旧 1.5°/40ms 完全一致！ */
   #define SERVO_INTERVAL_MS          40U     /* 原 80U：缩短舵机插值滞后 */
   ```
   - 分别用 `ClampFloat(val, -MAX_PAN_STEP_PER_UPDATE, MAX_PAN_STEP_PER_UPDATE)` 与 `ClampFloat(val, -MAX_TILT_STEP_PER_UPDATE, MAX_TILT_STEP_PER_UPDATE)` 限制水平/俯仰步长。
   - **注意 `HAL_Delay(12U)`（原 L546）**：在 `TRACK_PERIOD_MS = 25U` 下，若两轴同时发指令，`12 ms` 阻塞过长；Fashion Star 舵机单帧 `12 bytes @ 115200 baud` 仅需 `1.04 ms`，无应答写指令间隙 **`3U` ms** 即可（或改为分拍交替发送：偶拍发 pan、奇拍发 tilt，或保留 `HAL_Delay(3U)` 避免半双工总线粘包）。推荐 `HAL_Delay(3U)`，总 UART5 耗时 `< 5 ms`，完全不挤占 50 Hz 底盘环。

3. **高频非阻塞 `STAT` 上报（10 Hz，消除 `0.5 s` 角度滞后与阻塞丢包）**：
   - 将 `STATUS_PERIOD_MS` 从 `500U` 改为 **`100U`**（10 Hz）。
   - 将 `SendStatus()` 中的 `HAL_UART_Transmit(&huart1, ..., 50U)` 改为**非阻塞中断发送**（使用文件级静态缓冲区 `static char stat_tx_buf[180]`，避免栈变量在 DMA/IT 期间失效）：
     ```c
     if (huart1.gState != HAL_UART_STATE_READY) return;
     HAL_UART_Transmit_IT(&huart1, (uint8_t *)stat_tx_buf, (uint16_t)length);
     ```
   - 同样检查 `SendText()`：若 `huart1.gState != HAL_UART_STATE_READY`，可短暂跳过非关键事件或使用小型队列，绝不在 `50 Hz` 主循环里阻塞 `8~50 ms`。
   - **校验和**：保持原有 `STAT,mode=...,bearing10=...,tilt10=...` 格式（也可在末尾追加 `*XX` XOR 校验，ROS 侧 `GimbalStatusParser` 已原生支持有无 `*XX` 自动识别）。

4. **新增轻量搜索方向引导命令（解决右转与云台反向抵消）**：
   - 新增两个 `0xAB` 控制命令码（旧固件会走 `default: break;` 无副作用）：
     ```c
     #define COMMAND_SEARCH_DIR_POS     0x04U   /* 强制 search_direction = +1.0f（舵机正角方向） */
     #define COMMAND_SEARCH_DIR_NEG     0x05U   /* 强制 search_direction = -1.0f（舵机负角方向） */
     ```
   - 在 `GimbalTracker_Task()` 的命令分发 `switch (command)` 中处理：
     - `COMMAND_SEARCH_DIR_POS`：`search_direction = 1.0f;`
     - `COMMAND_SEARCH_DIR_NEG`：`search_direction = -1.0f;`
   - **安全不变量**：仅修改下一次搜索步进方向，不改 `gimbal_mode`，不干扰已锁定跟踪（`target_found == 1` 时完全无影响），也不改变 `pending_command` 之外的任何状态。

##### 2.2 ROS 侧协同改动（`gimbal_bridge_node.cpp` + `gimbal_status.hpp` + Launch）
1. **弯道内侧搜索引导（`gimbal_bridge_node.cpp` + `gimbal_status.hpp`）**：
   - 抽出纯函数 `stm32_bridge::desiredSearchDirectionForTurn(...)` 放到 `gimbal_status.hpp` 便于单测：
     - 输入：`autonomy_enabled`、`searching`、`odom_wz`（ROS 约定左转 `+`、右转 `-`）、`pan_ros_deg`（`= pan_to_ros_sign * pan_deg`，ROS 约定左 `+`、右 `-`）、`pan_to_ros_sign`。
     - 规则：仅当 `autonomy_enabled && searching` 且车身明显转向（`|odom_wz| >= 0.22 rad/s`）时：
       - 若车身右转（`odom_wz <= -0.22`）且镜头偏向左侧（`pan_ros_deg >= 12.0°`），希望云台朝**右侧**（ROS 负角方向）回扫，对应舵机方向 `dir = (pan_to_ros_sign < 0 ? +1 : -1)`；
       - 若车身左转（`odom_wz >= +0.22`）且镜头偏向右侧（`pan_ros_deg <= -12.0°`），希望云台朝**左侧**（ROS 正角方向）回扫，对应舵机方向 `dir = (pan_to_ros_sign < 0 ? -1 : +1)`；
       - 否则返回 `0`（不干预）。
   - 在 `GimbalBridgeNode::timerCb()` 中新增参数 `turn_search_bias_enabled`（默认 `true`），且加 **`0.8 s` 冷却时间**与 `0.2 s` 命令保护窗，满足条件时发送 `0xAB 0x04 0xAF` 或 `0xAB 0x05 0xB0`（不覆盖 `Center/Enable` 命令）。
2. **配合 10 Hz `STAT` 更新 ROS 参数（`navigation_visual_handoff.launch` & `navigation_stack.launch`）**：
   - `gimbal_status_period_est_sec`: 保留参数，默认设为 `0.100`（若新固件已改非阻塞 IT 发送，同时将 `gimbal_avoid_status_tx_window` 默认设为 `false`；若尚未刷固件仍可一键切回 `true / 0.516`）；
   - `gimbal_max_skew`: 保持 `0.55`（向后兼容旧固件 `2 Hz`，在新固件 `10 Hz` 下 `GimbalHistory` 线性插值误差自动缩小 **5 倍**）；
   - 保持 `gimbal_yaw_feedforward_sec = 0.12`（可随实测微调至 `0.08~0.12`）。

---

### WP3：`visual_handoff_manager` 接近期按距离分段限速（解除提前锁定后的远距减速陷阱）

#### 问题根因
当前 `visual_handoff_manager.cpp:139-145` 在 `nav_state_.find("APPROACH:") == 0` 时无条件将底盘速度限制在 `approach_max_speed = 0.35 m/s`、`approach_max_turn = 0.40 rad/s`。WP1 生效后，小车在距终点 `4.0~5.9 m` 处就会进入 `APPROACH:`，若此处不改，整条 `3~4.5 m` 直道都会被压在 `0.35 m/s` 爬行。

#### 涉及文件
- `src/maze_planner/include/maze_planner/handoff_corridor.h`（或新建纯计算辅助函数供单测）
- `src/maze_planner/src/visual_handoff_manager.cpp`
- `src/maze_bridge/launch/navigation_visual_handoff.launch`
- `src/maze_planner/test/handoff_corridor_test.cpp`（补充距离分段限速单测）

#### 详细设计
1. **新增可调参数（`visual_handoff_manager.cpp`）**：
   - `cruise_max_speed`（默认 `0.60` m/s）
   - `cruise_max_turn`（默认 `0.60` rad/s）
   - `approach_max_speed`（保持默认 `0.35` m/s）
   - `approach_max_turn`（保持默认 `0.40` rad/s）
   - `approach_slowdown_distance`（默认 `2.20` m，距终点墙面大于此值时不压低巡航速度；设为 `<= 0` 时退化为旧行为全程限速）
   - `approach_full_slow_distance`（默认 `1.35` m，距终点墙面小于等于此值时完全降至 `approach_max_speed / approach_max_turn`）
2. **可靠的距离来源（视觉/雷达优先 + `move_base` 入口目标兜底）**：
   - 在 `VisualHandoffManager` 中新增两个轻量订阅：
     - `/move_base/feedback`（`move_base_msgs/MoveBaseActionFeedback`）记录最新 `robot_pose_` 与时间；
     - `/blue_target/approach_goal`（`geometry_msgs/PoseStamped`）记录最新入口目标 `approach_goal_` 与时间（入口距墙约 `0.80 m`）。
   - 同时每当 `corridorReady()` 成功拟合出 `wall.marker`（或 `targetGeometry()` 得到有效 `visual_range`）时，更新 `last_target_range_` 与 `last_target_range_time_`（新鲜阈值 `1.0 s`）。
   - 在 `navCmdCb()` 中估算当前距终点墙面距离 `est_dist`：
     - 优先取新鲜的 `last_target_range_`；
     - 若视觉短时被箱子遮挡（`> 1.0 s` 无新鲜测距），取 `hypot(approach_goal_ - robot_pose_) + 0.80`（前提是两个话题均在 `1.5 s` 内收到过）；
     - 若两者皆不可用（例如单测仅发了 `nav_state="APPROACH: ..."` 未发测距），`est_dist = 0.0` → **保守走原 `approach_max_speed / approach_max_turn` 限速（fail-safe，且兼容所有旧 mock 测试！）**。
3. **纯函数实现平滑保曲率限速（放入 `handoff_corridor.h` 供单元测试）**：
   ```cpp
   inline void approachVelocityLimits(double target_distance,
                                      double slowdown_distance,
                                      double full_slow_distance,
                                      double cruise_v, double cruise_w,
                                      double approach_v, double approach_w,
                                      double* max_v, double* max_w) {
     if (!(std::isfinite(target_distance) && slowdown_distance > full_slow_distance &&
           target_distance > full_slow_distance)) {
       *max_v = approach_v;
       *max_w = approach_w;
       return;
     }
     const double alpha = std::min(1.0, (target_distance - full_slow_distance) /
                                        (slowdown_distance - full_slow_distance));
     *max_v = approach_v + alpha * std::max(0.0, cruise_v - approach_v);
     *max_w = approach_w + alpha * std::max(0.0, cruise_w - approach_w);
   }
   ```
   - 在 `navCmdCb()` 中用动态算出的 `(*max_v, *max_w)` 执行原有保曲率缩放 `scale = std::min(vs, ws)`。
4. **Launch 透传（`navigation_visual_handoff.launch`）**：
   - 将 `cruise_max_speed`、`cruise_max_turn`、`approach_slowdown_distance`（`2.20`）、`approach_full_slow_distance`（`1.35`）传给 `visual_handoff_manager`。

---

### WP4：`CorridorExplorer` 宽通道探索前沿点居中化

#### 问题根因
`corridor_explorer.h:154` 中：
```cpp
const double candidate = depth + 1.6 * std::min(clearance_m, 1.0) -
                         .15 * from_robot[i] - (tracked ? .20 : 0.0);
```
因为 `distances(grid, obstacles, nullptr, 1.5)` 已算到 `1.5 m`，但打分把 `clearance_m` 硬截断在 `1.0 m`，在 `match-1127-1228` 末段宽 `~2.9 m` 的空旷区形成平顶，导致前沿点被拉到左下角 `(2.13, -4.88, -114.9°)`，比实际通道中线偏左 `0.45 m`。

#### 涉及文件
- `src/maze_planner/include/maze_planner/corridor_explorer.h`
- `src/maze_planner/test/corridor_explorer_test.cpp`

#### 详细设计
1. **调整打分项（`corridor_explorer.h:154-155`）**：
   - 将 `1.6 * std::min(clearance_m, 1.0)` 改为 **`2.5 * clearance_m`**（其中 `clearance_m = std::min(1.5, clearance[i])` 已在 L142 做好上限截断）。
   - **实图验证结果**（已在 `match-1127-1228/map.pgm` 上离线跑通）：
     - 修改前：`(2.13, -4.88)`（偏左贴角，距中线 `~0.45 m`）
     - 修改后：**`(2.66, -4.45)`**（`clr = 1.45 m`，紧贴通道正中线，距真终点横向仅 `~0.11 m`！）
   - **窄通道兼容性**：在现有 `corridor_explorer_test.cpp` 的 `1.30 m` 宽标准廊道（最大 `clearance = 0.65 m`）中，沿走廊轴向距离墙 ≥ `0.65 m` 处 `clearance` 恒为 `0.65 m`，仍精确选中走廊末端中线且全部既有断言（`goal.x > 6.3`、`goal.y ≈ 2.5`、T 型分叉双支路遍历）100% 保持通过。

---

### WP5：末端 `TERMINAL_PARK` 交接距离微调与 `ALIGN ↔ APPROACH` 迟滞防抖

#### 问题根因
1. `attempt-04`（4.54s）和 `attempt-06`（4.99s）在末端反复 `ALIGN ↔ APPROACH` 切换 2～3 次，源于 `turn_before_drive`（`0.44 rad`）与 `final_yaw_align`（`8°`）均无进入/退出迟滞。
2. `max_handoff_distance = 1.30 m` 略偏保守，且 `direct_reactive_policy.h:188` 的末端速度斜率 `(wall_depth - 0.48) * 0.8` 导致最后 `0.6 m` 耗时长达 `3.0~3.6 s`。

#### 涉及文件
- `src/maze_planner/include/maze_planner/direct_reactive_policy.h`
- `src/maze_planner/src/direct_visual_controller.cpp`
- `src/maze_bridge/launch/navigation_visual_handoff.launch`
- `src/maze_planner/test/direct_reactive_test.cpp`

#### 详细设计
1. **为 `ALIGN` 状态增加迟滞（`direct_reactive_policy.h` + `direct_visual_controller.cpp`）**：
   - 在 `reactiveVisualCommand(...)` 尾部增加带默认值的参数 `bool previously_aligning = false`（所有旧调用方和单测签名 100% 兼容）：
     - `steer_align` 阈值：未处于 `ALIGN` 时用 `p.turn_before_drive`（`0.44 rad ≈ 25.2°`）进入；已处于 `ALIGN` 时降至 `0.65 * p.turn_before_drive`（`0.286 rad ≈ 16.4°`）才退出；
     - `final_yaw_align` 阈值：未处于 `ALIGN` 时用 `8.0°` 进入；已处于 `ALIGN` 时降至 `4.0°` 才退出（`wall_depth <= 0.65` 条件不变）。
   - 在 `DirectVisualController` 中增加成员 `bool aligning_{false};`（在 `enableCb` 中复位为 `false`），调用 `reactiveVisualCommand(..., aligning_)` 后更新 `aligning_ = (std::string(command.state) == "ALIGN");`。
2. **平顺提升近距末端接近速度（保持安全停靠边界不变）**：
   - 在 `direct_reactive_policy.h:188` 将：
     ```cpp
     drive_speed = std::min(drive_speed, std::max(.04, (wall_depth - p.metric_stop_depth_min) * .8));
     ```
     调整为：
     ```cpp
     drive_speed = std::min(drive_speed, std::max(.06, (wall_depth - p.metric_stop_depth_min) * 1.1));
     ```
     - 下方 L240–247 的 `remaining = wall_depth - p.metric_stop_depth_min` 防越界截断与 `reactivePathClear(...)` 扫掠校验保持不变，绝不会冲过 `0.48 m` 安全深度下限。
3. **略微放宽行进间直道交接距离（`navigation_visual_handoff.launch:31`）**：
   - `max_handoff_distance`: `1.30` → **`1.50`** m（仍受 `max_handoff_lateral = 0.38 m`、`max_handoff_yaw_rad = 0.40 rad` 以及完整 `handoffCorridorClear` 3 帧激光清空校验严格把关）。

---

## 2. 验收标准与必须执行的测试矩阵

Codex 完成修改后，必须逐项执行以下全部验证并汇报真实输出，**0 errors、0 failures、不跳过**：

### 2.1 新增单元测试（必须补齐）
1. **`src/maze_planner/test/visual_tracking_test.cpp`**（WP1）：
   - 验证在模拟真实标定数据（`camera_z=0.42`, `joint_tilt=+6.2°`, `camera_pitch_offset=-0.056 rad`, `d=5.9 m`）下，修正后 `plausibleTargetHeight()` 通过且 $z \approx 0.15\text{ m}$；同时验证高处假蓝牌（`cy=160, d=5.0 m`）依然被 `max_target_z=0.52` 严格拒绝。
2. **`src/stm32_bridge/test/gimbal_bridge_parser_test.cpp`**（WP2）：
   - 验证 `desiredSearchDirectionForTurn()` 在右转+镜头偏左、左转+镜头偏右、直行、非搜索状态以及 `pan_to_ros_sign = ±1` 下的方向输出符号完全正确。
3. **`src/maze_planner/test/handoff_corridor_test.cpp`**（WP3）：
   - 验证 `approachVelocityLimits()`：`d >= 2.2 m` 返回全速 `(0.60, 0.60)`、`d <= 1.35 m` 返回接近限速 `(0.35, 0.40)`、中间距离线性平滑过渡、无效/未知距离（`NaN` 或 `0`）保守返回 `(0.35, 0.40)`。
4. **`src/maze_planner/test/corridor_explorer_test.cpp`**（WP4）：
   - 新增宽通道场景（宽 `2.8 m`）测试，验证 `CorridorExplorer::select()` 选出的前沿点位于通道中线附近（横向偏离中线 `< 0.25 m`），不再偏向边角；同时确认原有全部 3 条 `CorridorExplorer` 单测全过。
5. **`src/maze_planner/test/direct_reactive_test.cpp`**（WP5）：
   - 验证 `previously_aligning=true` 时，在 `normal_yaw = 6.0°`（介于 `4°` 与 `8°` 之间）仍保持 `ALIGN` 直到 `<= 4°` 才切回 `APPROACH`，消除边界抖动；且 `previously_aligning=false` 时 `6.0°` 直接 `APPROACH`。

### 2.2 全量构建、参数检查与离线回放验证
1. **宿主机 ROS1 全量编译与单元测试**：
   ```bash
   bash -ic 'use_ros1 && cd "/home/asuka/Desktop/college-level competition/xiaoche" && catkin_make -j4 && catkin_make run_tests && catkin_test_results build'
   ```
   要求：**0 errors, 0 failures**。
2. **竞赛 Docker 运行镜像重新编译**：
   ```bash
   cd "/home/asuka/Desktop/college-level competition/xiaoche" && bash tools/competition_terminal.sh prepare
   ```
   要求退出码 `0`。
3. **Launch 参数全量核对**：
   使用 `roslaunch --dump-params` 检查 `navigation_visual_handoff.launch`：
   - `camera_pitch_offset = -0.056`（`gimbal_bridge/camera_pitch_offset`、`handoff_blue_navigator/camera_pitch_offset`、`handoff_direct_visual/camera_pitch_offset`、`visual_handoff_manager/camera_pitch_offset` 四处完全一致）；
   - `fallback_hfov_deg = 75.0` 四处一致；
   - `visual_handoff_manager` 中 `approach_slowdown_distance = 2.20`、`approach_full_slow_distance = 1.35`、`cruise_max_speed = 0.60`、`max_handoff_distance = 1.50`；
   - `gimbal_bridge` 中 `turn_search_bias_enabled = true`。
4. **隔离仿真回归**（如当前环境可用）：跑通既有 ROS mock 测试（`visual_handoff_ros_mock.py`、`handoff_pipeline_ros_mock.py`、`visual_tracking_ros_mock.py`、`direct_visual_ros_mock.py`），确保 0 回归。

---

## 3. STM32 现场编译烧录与上车前 3 分钟台架核验清单

1. **确认烧录工程**：在现场 Keil 工程中定位当前车上在跑的 `gimbal_tracker.c`（确认使用 UART5 舵机总线、与底盘共用工程且非旧 `CAN1` 目录），合入 WP2.1 改动并编译烧录。
2. **架空/断电机使能开关台架核验（先不落地跑）**：
   - 运行 `bash tools/competition_terminal.sh ready match-01` 和 `bash tools/competition_terminal.sh go match-01`：
     - 确认待机状态条显示 **`云台:OK (enabled, 俯角=+10.0°)`**（验证 `trigger_race_go.py` 自检正常通过）；
     - 观察云台空扫幅宽约为 **`±48°`**、单程约 **`1.4 s`**（肉眼明显比之前快且不再扭头看车侧后方）；
     - 用 `rostopic hz /gimbal/joint_states` 确认状态频率升至 **`~10 Hz`**（原 `~1.9 Hz`）；
     - 手持蓝纸在 `1~5 m` 处左右移动，确认云台迅速跟上、不掉线，且 `rostopic echo /blue_target/map_pose` 在 **`4~5 m`** 处即开始持续输出（证明 WP1 远距高度过滤已彻底打通）。
3. **实地复测对比**：在 `match-01`（或新建场次复用 `match-1127-1228` 地图）按无箱 / 单箱 / 双箱各跑 1 轮，对比右转后锁定时间与总完赛时间；预计无箱从 `19.5s` 降至 **`14~16s`**，带箱场景从 `21~23s` 降至 **`16~18s`**，且右转后显著减少偏移与回转。
