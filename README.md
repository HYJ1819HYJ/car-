# 迷宫小车 ROS1 工程

本仓库保存迷宫小车的 ROS Noetic 上位机代码，包含 STM32 串口桥、轮式里程计、激光雷达自反射过滤、SLAM 建图、AMCL 导航、动态避障、手柄控制、USB 摄像头、蓝色终点识别与云台通信。

## 目录

- `src/stm32_bridge`：底盘串口、里程计、雷达过滤、蓝色目标检测、云台桥接及测试。
- `src/maze_bridge`：硬件、建图、导航、代价地图和 RViz 的启动与参数文件。
- `src/maze_planner`：手柄控制、迷宫控制、轨迹记录/复现和蓝色终点导航。
- `maps`：当前比赛地图；换场地后应重新建图。
- `docs`：硬件连接、功能验证、建图和比赛流程说明。
- `build_map.sh`：slam_toolbox 建图入口。

## 环境

- Ubuntu 20.04
- ROS Noetic
- catkin
- 外部雷达驱动工作空间：`LakiBeam_ws`（其厂商源码不在本仓库中）

先安装工作空间声明的依赖：

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

## 导航

```bash
source /opt/ros/noetic/setup.bash
source ~/LakiBeam_ws/devel/setup.bash
source ~/xiaoche/devel/setup.bash
roslaunch maze_bridge navigation_stack.launch \
  mode:=navigation \
  map_file:=/home/hyj/xiaoche/maps/competition_map_v3.yaml \
  port:=/dev/serial/by-path/你的底盘串口 \
  show_rviz:=true
```

设备路径、雷达 IP、车体尺寸和传感器外参均与实车相关，换电脑、USB 接口或机械结构后，请先修改 launch 参数并重新标定。当前导航安全逻辑默认禁用自主倒车，并按完整车体轮廓进行碰撞预测。

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
