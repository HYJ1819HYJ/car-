# 比赛场地：建图后搬移同两个障碍物

**建图时四个固定箱体加两个可搬移箱体，共六个；保存地图后，将同两个箱体搬到其他坐标，再开始导航。导航过程中六个箱体全部静止。** 不再使用往返运动、临近车辆让行或建图后新增箱体的场景。

黄色框按 0.8 × 0.8 m 标定，场地约 8.80 × 7.31 m，地图分辨率 0.025 m。新增模型与原图四个箱体的 Stage 实际尺寸一致：0.750 × 0.675 × 0.60 m。保留原小车 STL、24 点真实轮廓、未知区域限制及完整车身扫掠检查。尺寸来自示意图推算，现场仍需实测。

## 搬前／搬后地图

两个可搬移箱体的建图中心固定为 **(5.50, 6.30)、(8.15, 2.60) m**。每组导航使用这张搬前六箱地图，实际世界使用下表的搬后位置。

| 场景 | 箱体 1 搬后中心 m | 箱体 2 搬后中心 m | 预期 |
| --- | --- | --- | --- |
| `01_open` | (5.20, 5.55) | (8.15, 1.80) | 到达终点 |
| `02_upper_reroute` | (6.10, 4.65) | (8.10, 1.80) | 到达终点 |
| `03_lower_reroute` | (2.60, 1.50) | (5.80, 3.30) | 到达终点 |
| `04_divider_detour` | (7.35, 2.45) | (5.30, 0.55) | 到达终点 |
| `05_finish_approach` | (1.75, 2.35) | (6.15, 4.30) | 到达终点 |
| `06_blocked_exit` | (5.22, 5.75) | (6.25, 5.75) | 无路则安全停车 |

文件在 `relocated_scenarios/<场景>/`：

- `before.yaml/pgm`、`before.world`、`before.png`：搬前六箱地图、世界、预览。
- `environment.yaml/pgm`、`world.world`、`preview.png`：搬后六箱实景、世界、预览；仅作为环境和验收真值，不输入导航。
- `scenario.json`：同 ID 箱体的搬前／搬后坐标、统一尺寸、验收预期。
- `relocated_scenarios/overview.png`：六组搬前／搬后对照。红色轮廓为旧位置，橙色／紫色为同两个箱体的新位置。

先验地图由搬前场景几何栅格化生成；本次验证的是已有地图上的定位和导航，并未声称运行 SLAM 采集生成了它。`before.world` 可用于单独复现搬前场地。

## 终端启动

先在已有仿真的启动终端按 **Ctrl+C**，等待 Stage/RViz 退出。默认自动发往终点的目标：

```bash
"/home/asuka/Desktop/college-level competition/xiaoche/start-competition-sim.sh" scenario:=03_lower_reroute
```

只启动环境，自行在 RViz 使用 **2D Nav Goal**：

```bash
"/home/asuka/Desktop/college-level competition/xiaoche/start-competition-sim.sh" scenario:=03_lower_reroute demo:=false
```

换组时退出后替换 `scenario:=` 后的名字。默认 `01_open`。默认前进／转向上限 3.0 m/s、1.0 rad/s（DWA 与仿真安全层共用启动参数，Stage 模型支持同一上限）；需要低速可附加 `max_nav_v:=0.10 max_nav_w:=0.25`。2D Pose Estimate 只改变定位估计，不移动虚拟车辆。

如果原启动终端已丢失，可在另一个终端关闭本仿真：

```bash
"/home/asuka/Desktop/college-level competition/xiaoche-ros1.sh" \
  env ROS_MASTER_URI=http://127.0.0.1:11329 rosnode kill /competition_stage
```

## 如何清除搬走后的旧占据

原 ROS 障碍层会检测新位置，但默认最大值合并不能抹掉静态层中的旧箱体。因此仅启动原避障无法完整处理地图过期问题。

本包增加仿真专用 `reconcile_prior_map.py`：

1. `/map` 保留 `maps/mapping_before_move.yaml` 的搬前六箱地图，AMCL 使用它；文件不变。
2. 节点只读取该先验地图、建图时标注的两个可搬移区域和 `/scan_obstacles`，通过 TF 定位激光。**不读取搬后坐标、实景地图或 Stage 真值。**
3. 同一旧占据格必须有三个不同时间戳的连续扫描确认射线穿过、并越过至少 8 cm，才在 `/navigation_map` 清为空闲。相邻波束也必须通过；NaN、自身遮挡、近处障碍挡住的射线不作空闲证据。未观测处继续保留。
4. 仅允许清理这两个先验标注区域；固定墙、原四箱和未知区逐格保持不变。这个标注是本仿真的显式先验条件，实车也需要提供可搬移区域或另行验证通用地图更新方案。
5. 全局静态层使用 `/navigation_map` 接收更新，原障碍层继续用真实激光标记搬后箱体；固定墙角保护仍根据四固定箱底图生成。

RViz 主地图显示 `/navigation_map`；红框是旧位置显示标记，彩色实体是实际摆放显示标记，这些标记没有输入导航。原图可以查看 `/map`。诊断：

```bash
"/home/asuka/Desktop/college-level competition/xiaoche-ros1.sh" \
  env ROS_MASTER_URI=http://127.0.0.1:11329 rostopic echo /simulation/map_reconciliation
```

其中每箱的 `prior_cells/cleared_cells` 为旧占据格数／激光确认后清除数；封堵组无法看到远处旧箱位置时该箱可能保持未清除，不能用未观测区域冒充空闲。

## 批量复测和结果

先退出当前仿真并等待端口 11329 释放：

```bash
python3 "/home/asuka/Desktop/college-level competition/xiaoche/tools/test_relocated_obstacle_scenarios.py"
```

只测一组可在命令末尾加 `04_divider_detour`。各组独立启动／退出，Stage 保留 GUI 维持实时仿真，默认不打开 RViz。

报告中的兼容字段 `added_*` 现指同两个搬移箱体，并不表示新增。

本次结果写入 `xiaoche/outputs/relocated_obstacles_20261019/`：`summary.json/md`、每组 `result.json`、`trajectory.png`、`launch.log`。报告检查搬前 `/map` 一致性、激光确认清除旧位置且固定区域不变、新位置激光及致命代价观测、完整真实车体的碰撞采样、倒车指令及最终实际速度。可达组需到达终点；封堵组需实际停车。离线外接圆连通性只是预检查。

旧的 `scenarios/`、`tools/test_added_obstacle_scenarios.py` 与 `outputs/added_obstacles_20261019/` 是此前“建图后新增两个箱体”的历史数据，**不作为本次搬移验证的依据**；旧测试脚本已禁用，防止误运行。

修改 `config/relocated_obstacle_scenarios.yaml` 的 `mapped_centers`（搬前）或各场景 `centers`（搬后）后，重新生成：

```bash
python3 "/home/asuka/Desktop/college-level competition/xiaoche/tools/generate_relocated_obstacle_scenarios.py"
"/home/asuka/Desktop/college-level competition/xiaoche-ros1.sh" bash -c 'catkin_make -j4 --pkg competition_sim'
```

## 导航和安全链路

Docker ROS1 Noetic，独立 master 端口 11329：

`激光 → 自反射过滤 → /scan_obstacles → 全局／局部障碍层 → SmoothGlobalPlanner + DWAPlannerROS → /simulation/nav_cmd_vel → simulation_safety_gate → /simulation/cmd_vel → Stage`

安全节点复用 `stm32_bridge/safety_geometry.hpp` 全车体检查；前方 0.30 m 内减速，前缘 0.08 m 内或预测碰撞时停车，预测 0.8 s、余量 0.02 m，传感器／指令超时停车。没有通过改变障碍物运动来避让车辆。

不启动真实串口、MCU、相机或雷达。Stage 是差速运动学仿真，闭环结果不代表实车验收。世界、底盘和雷达周期 33 ms、`speedup=1`；此版本不要使用会过快推进仿真时间的 `stageros -g`。
