# 迷宫小车 ROS1 工程

版本变化见 [changelog](changelog)。

本仓库保存迷宫小车的 ROS Noetic 上位机代码，包含 STM32 串口桥、轮式里程计、激光雷达自反射过滤、SLAM 建图、AMCL 导航、动态避障、手柄控制、USB 摄像头、红色终点识别与云台通信。

`asuka-main` 分支保存 Asuka 基于队友原项目进行的调试和改进，包含完整车体轮廓、平滑全局规划、里程计与建图参数调整及外设调试工具。此仓库为 ROS1 上位机工程，不包含 STM32 下位机固件；LakiBeam 厂商驱动仍需单独准备。编译产物、现场日志和临时回滚备份不纳入版本控制；文档中的 `outputs/` 路径指本地调试记录。

2026-10-15 版本增加了静止定位更新、经过整车检查的原地转向、整车全局搜索，以及中间隔板端头/内侧拐角的定点余量。新地图为 `maps/competition_map_20261015_final.yaml`；启动、区域调整及回退说明见 [本版导航说明](docs/导航定位与定点余量_2026-10-15.md)。

## 目录

- `src/stm32_bridge`：底盘串口、里程计、雷达过滤、红色目标检测、云台桥接及测试。
- `src/maze_bridge`：硬件、建图、导航、代价地图和 RViz 的启动与参数文件。
- `src/maze_planner`：手柄控制、迷宫控制、轨迹记录/复现和红色终点导航。
- `src/maze_smooth_planner`：检查完整车体轮廓与路径扫掠碰撞的平滑全局规划器。
- `models`、`tools`：STL 车体模型、轮廓生成和调试工具。
- `maps`：当前比赛地图；换场地后应重新建图。
- `docs`：硬件连接、功能验证、建图和比赛流程说明。
- `build_map.sh`：slam_toolbox 建图入口。

## 环境

- Ubuntu 20.04
- ROS Noetic
- catkin
- 外部雷达驱动工作空间：`LakiBeam_ws`（其厂商源码不在本仓库中）

队友电脑建议直接执行一键安装：

```bash
sudo apt update && sudo apt install -y git
git clone --branch asuka-main https://github.com/HYJ1819HYJ/car-.git ~/xiaoche
cd ~/xiaoche
chmod +x setup_ros_noetic.sh
./setup_ros_noetic.sh
```

脚本仅支持 Ubuntu 20.04，会安装 ROS Noetic、导航/建图/相机/手柄依赖，编译工作空间并安装 VS Code 推荐扩展。Windows 用户只能在 WSL2 Ubuntu 20.04 中运行 `setup_ros_noetic.bat`；连接实车仍推荐原生 Ubuntu。

详细说明见 [`队友环境安装说明.md`](队友环境安装说明.md)。手动安装与编译命令如下：

```bash
cd ~/xiaoche
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

如 `rosdep` 提示找不到 `lakibeam1`，先单独编译并 source 雷达厂商工作空间，再编译本工程。

## 建图

确认雷达网卡、底盘串口和手柄均已连接，然后执行：

```bash
cd ~/xiaoche
./build_map.sh port:=/dev/serial/by-path/你的底盘串口
```

完成建图后，在另一个终端保存：

```bash
source ~/xiaoche/devel/setup.bash
rosrun map_server map_saver -f ~/xiaoche/maps/competition_map_v3 map:=/map
```

等待保存完成，再退出建图后启动导航。导航默认在代价地图就绪前生成或复用该地图的拐角及薄墙自由末端保护层，无需另行运行生成工具；地图图像、原点、分辨率或保护参数变化时自动重新生成，原图保持不变。

## 导航

```bash
source /opt/ros/noetic/setup.bash
source ~/LakiBeam_ws/devel/setup.bash
source ~/xiaoche/devel/setup.bash
roslaunch maze_bridge navigation_stack.launch \
  mode:=navigation \
  map_file:=$HOME/xiaoche/maps/competition_map_v3.yaml \
  port:=/dev/serial/by-path/你的底盘串口 \
  show_rviz:=true
```

设备路径、雷达 IP、车体尺寸和传感器外参均与实车相关，换电脑、USB 接口或机械结构后，请先修改 launch 参数并重新标定。当前导航安全逻辑默认禁用自主倒车，并按完整车体轮廓进行碰撞预测。

拐角保护默认开启：两侧墙段各至少 25 cm，方向变化至少 45°（排除大于 170° 的近折返），拐角附近额外保留 5 cm。默认根据新地图的主体连通墙体确定场地范围，包含其内部的独立墙段；不会沿用旧地图坐标。需要检测全图时加 `corner_scope:=all`；也可用 `corner_bounds:='[xmin, ymin, xmax, ymax]'` 指定本图范围。回退使用 `corner_clearance:=false`。旧命令若带有该关闭参数，应删除后再使用默认流程。详见 [拐角保护与自动加载](docs/全迷宫拐角余量_2026-10-18.md)。

单片薄墙的自由末端也默认保护：沿用长墙轮廓，识别两侧折返的薄端帽，在真实末端占据格周围生成半径 5 cm 的圆盘。墙段至少 25 cm、端帽横向厚度不超过 12 cm；普通直墙中段、闭合外墙和大箱体不按末端处理。旧版只有拐角的自动缓存会重新生成；显式指定旧 mask 时会拒绝启动并提示重新生成。`wall_endpoint_clearance:=false` 仅关闭末端保护，原有拐角保护保留；`wall_endpoint_radius:=0.05` 设置末端半径。详见 [墙壁末端保护](docs/墙壁末端5cm保护_2026-10-19.md)。

## 箱子换位避障

建图时仅保存固定墙与隔板，移开以后会变位置的箱子。保存地图、退出建图后再放入箱子，导航用 `/scan_obstacles` 实时标记和射线清除箱位。这个流把车身遮挡标为无效，避免误清盲区；自动恢复保留车周围 4 m 方形内的近处观测。固定墙角仍有 5 cm 保护。完整命令和箱子 A/B 换位测试见 [箱子换位避障测试](docs/箱子换位避障测试_2026-10-18.md)。已写进旧静态图的箱子不会因为开启实时避障自动消失。

## 测试

```bash
cd ~/xiaoche
source devel/setup.bash
catkin_make run_tests
catkin_test_results
```

详细操作见 [`docs`](docs) 和 [`src/maze_bridge/HARDWARE_SETUP.md`](src/maze_bridge/HARDWARE_SETUP.md)。

## 许可

MIT License，见 [`LICENSE`](LICENSE)。

摄像头默认识别红色 A4 目标（HSV H=170–179 或 0–10，S/V≥80）。为兼容现有导航和云台，节点、话题及参数仍保留 `blue_*` 名称。修改后需重新编译并重启识别节点；实际光照下的效果需现场确认。
