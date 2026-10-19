# 比赛场地：建图后新增两个固定障碍物

导航底图只包含原图的 **4 个箱体**。每次导航前，在实景中放入 **2 个同尺寸、固定不动的新箱体**。小车必须通过激光发现它们，再由原有全局/局部代价地图、SmoothGlobalPlanner 和 ForwardTeb/TEB 绕行；无法安全通过则停车。

已删除此前的往返运动控制器、25 cm 方块和运动跟踪配置。新增箱体没有让行、移动、消失或暂停逻辑。场景数据只提供给环境、显示及验收，不提前输入导航。

## 尺寸和地图

- 黄色区域按 0.8 × 0.8 m 标定；场地约 **8.80 × 7.31 m**，分辨率 **0.025 m**。
- 原图四个箱体在 Stage 实际占据栅格中都是 **0.750 × 0.675 m**，新增两个逐一复制该尺寸，高 0.60 m。原图未栅格化的推算约为 0.735 × 0.647 m，两者差异来自栅格取整。
- 墙厚按 0.04 m 建模，栅格化为 0.05 m；起终点地面标线不作为墙。
- 原小车 STL、24 点真实车体轮廓、禁止自主倒车和未知区域策略均保留。
- 起点 `(6.5622, 6.9205, −π/2)`；停车目标 `(0.5000, 2.5589, π)`，位于黄色终点框内，与左墙留有间隔。

`maps/final_competition.yaml` 是所有测试共用的 **四箱导航底图**，供 map_server、AMCL 和静态代价图加载。

`scenarios/<场景名>/environment.yaml` 是 **六箱实景/验收地图**，用于复现和查看该组摆放；不是导航先验。每组同目录还保存 `environment.pgm`、`world.world`、`scenario.json` 和 `preview.png`。启动文件自动选择实景世界，同时始终向导航加载四箱底图。`scenarios/overview.png` 为六组总览，橙色、紫色表示新增箱体。

这些尺寸来自比赛示意图，现场尺寸、墙厚及箱体位置仍应实测复核。

## 六组固定摆放

坐标为箱体中心，单位 m；每组都新增两箱，尺寸相同。

| 场景名 | 测试目的 | 箱体 1 中心 | 箱体 2 中心 | 期望 |
| --- | --- | --- | --- | --- |
| `01_open` | 开阔区新增箱体 | (5.50, 6.30) | (8.15, 2.60) | 到达终点 |
| `02_upper_reroute` | 上半场改道 | (6.10, 4.65) | (8.10, 1.80) | 到达终点 |
| `03_lower_reroute` | 下方横向通道改道 | (2.60, 1.50) | (5.80, 3.30) | 到达终点 |
| `04_divider_detour` | 中间隔板出口绕行 | (7.35, 2.45) | (5.30, 0.55) | 到达终点 |
| `05_finish_approach` | 终点前绕行 | (1.75, 2.35) | (6.15, 4.30) | 到达终点 |
| `06_blocked_exit` | 出口封堵 | (5.22, 5.75) | (6.25, 5.75) | 安全停车，报告无法规划 |

前五组通过完整车体外接半径 0.37 m 的离线连通性检查；第六组三个剩余缝隙都小于车体宽度，用于验证拒绝强行穿越。实际闭环测试结果另见输出记录，离线连通不代替导航验收。

## 终端 1：启动一组

如果已有仿真，先在原启动终端按 **Ctrl+C**。没有原前台终端时，可执行：

```bash
"/home/asuka/Desktop/college-level competition/xiaoche-ros1.sh" \
  env ROS_MASTER_URI=http://127.0.0.1:11329 rosnode kill /competition_stage
```

**等待 Stage 和 RViz 退出后**，选择一组启动，例如下方通道测试：

```bash
"/home/asuka/Desktop/college-level competition/xiaoche/start-competition-sim.sh" scenario:=03_lower_reroute
```

其余五组将 `scenario:=` 后的名字替换为上表场景名。省略参数默认启动 `01_open`。终端保持运行，默认自动从起点导航到终点，结束后可继续使用 RViz 的 **2D Nav Goal**。停止时按 **Ctrl+C**，停止后再切换下一组，不要重复启动。

若想自行指定目标，不自动跑圈：

```bash
"/home/asuka/Desktop/college-level competition/xiaoche/start-competition-sim.sh" scenario:=03_lower_reroute demo:=false
```

**2D Pose Estimate 仅改变定位估计，不会移动虚拟车辆。**

## 终端 2：检查是否发现新增障碍

原仿真保持运行，可查看最终安全状态或图层话题：

```bash
"/home/asuka/Desktop/college-level competition/xiaoche-ros1.sh" \
  env ROS_MASTER_URI=http://127.0.0.1:11329 rostopic echo /simulation/safety_reason
```

RViz 中彩色箱体是实景位置显示；激光和局部代价地图的障碍边缘才是导航实际感知来源。显示标记没有给导航提供障碍物真值。

## 批量复测

先停止当前仿真，等待端口 11329 释放，再执行：

```bash
python3 "/home/asuka/Desktop/college-level competition/xiaoche/tools/test_added_obstacle_scenarios.py"
```

会顺序启动六组，每组从相同起点独立运行，结束后关闭再启动下一组。Stage 保留 GUI 以维持实时仿真；批量测试默认不打开 RViz。只复测一组可以指定名字：

```bash
python3 "/home/asuka/Desktop/college-level competition/xiaoche/tools/test_added_obstacle_scenarios.py" 04_divider_detour
```

记录在 `xiaoche/outputs/added_obstacles_20261019/`：

- `summary.json` / `summary.md`：逐组结果。
- `<场景名>/result.json`：动作状态、实际里程、停车速度、静态/新增箱体碰撞采样、激光和代价地图观测次数、完整真值轨迹。
- `<场景名>/trajectory.png`：六箱实景上的实际行驶轨迹。
- `<场景名>/launch.log`：该次 ROS 启动与导航日志。

每组验收同时检查：`/map` 与四箱原图逐格一致；两个新箱都有激光回波和致命代价观测；原障碍与新增障碍的真实车体重叠计数为零；没有倒车指令。可通行组要求到达终点，封堵组要求车辆实际速度归零并报告无法规划，不将强行穿越算作通过。

## 修改摆放和重新生成

编辑 `config/added_obstacle_scenarios.yaml` 中的固定中心坐标，然后生成全部地图和世界：

```bash
python3 "/home/asuka/Desktop/college-level competition/xiaoche/tools/generate_added_obstacle_scenarios.py"
```

新增包或更改 CMake 后编译一次：

```bash
"/home/asuka/Desktop/college-level competition/xiaoche-ros1.sh" bash -c 'catkin_make -j4'
```

## 导航和安全链路

使用 Docker ROS1 Noetic，独立 ROS master 端口 **11329**。链路为：

`扫描 → 自反射过滤 → /scan_obstacles → 全局/局部障碍层 → 原规划器 → /simulation/nav_cmd_vel → simulation_safety_gate → /simulation/cmd_vel → Stage`

新增静止箱体按普通障碍处理，使用原 TEB 配置，不加载运动预测插件。AMCL 概率场与 beamskip 保留，用于减少底图中不存在的新箱体对定位的干扰。角点和薄墙末端保护仍按四箱底图生成。

安全节点复用 `stm32_bridge/safety_geometry.hpp` 的完整车体扫掠检查：前方 0.30 m 内减速，车体前缘 0.08 m 内停车，预测碰撞或传感器/指令超时停车；预测时长 0.8 s，余量 0.02 m。只有安全节点发布最终底盘速度。

这里没有启动真实串口、MCU、相机或雷达设备。Stage 是差速运动学仿真，未模拟编码器标定、轮胎打滑或固件健康状态；闭环结果不代表实车验收。世界、底盘与雷达周期均为 33 ms、`speedup=1`，不要直接改成无 GUI 的 `stageros -g`，该模式在此版本中会过快推进仿真时间。
