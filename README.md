# 迷宫小车 ROS1 工程

版本变化见 [changelog](changelog)。

本仓库保存迷宫小车的 ROS Noetic 上位机代码，包含 STM32 串口桥、轮式里程计、激光雷达自反射过滤、SLAM 建图、AMCL 导航、动态避障、手柄控制、USB 摄像头、蓝色终点识别与云台通信。

`asuka-main` 分支保存 Asuka 基于队友原项目进行的调试和改进，包含完整车体轮廓、平滑全局规划、里程计与建图参数调整及外设调试工具。此仓库为 ROS1 上位机工程，不包含 STM32 下位机固件；LakiBeam 厂商驱动仍需单独准备。编译产物、现场日志和临时回滚备份不纳入版本控制；文档中的 `outputs/` 路径指本地调试记录。

2026-10-15 版本增加了静止定位更新、经过整车检查的原地转向、整车全局搜索，以及中间隔板端头/内侧拐角的定点余量。新地图为 `maps/competition_map_20261015_final.yaml`；启动、区域调整及回退说明见 [本版导航说明](docs/导航定位与定点余量_2026-10-15.md)。

2026-10-25 保存了 TEB [v1/v2 参数版本](docs/TEB参数版本_2026-10-22.md)：两个导航入口默认加载 v2，v1 仅归档。Stage 仿真支持 3 m/s、1 rad/s 上限，实车入口默认速度保持 0.50 m/s、0.75 rad/s；操作见 [TEB 仿真测速](docs/TEB速度测试_2026-10-22.md)。DWA 实验版本单独保存在 `asuka_dwa` 分支。

2026-11-16 增加仅供仿真使用的 TEB v3 覆盖参数，用于平衡行进效率与转向指令变化；实车仍加载原 v2。仿真加 `optimize_teb:=false` 可回退到 v2 对照，配置与结果见 [TEB 仿真调参记录](docs/TEB仿真调参_2026-11-16.md)。

同日完成比赛场地六种箱位的 v2/v3 对照，12 次导航均符合预期；可达场景总用时下降约 2.0%、转向指令总变差下降约 26.5%，详见 [比赛场地对照报告](docs/TEB比赛场地对照_2026-11-16.md)。

## 目录

- `src/stm32_bridge`：底盘串口、里程计、雷达过滤、蓝色目标检测、云台桥接及测试。
- `src/maze_bridge`：硬件、建图、导航、代价地图和 RViz 的启动与参数文件。
- `src/maze_planner`：手柄控制、迷宫控制、轨迹记录/复现和蓝色终点导航。
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

比赛流程以现场规则为准：场地搭建完成后先建图，可推车或用手柄操控；此时终点位置未知。建图完成后小车返回起点，场地内的两个障碍物改变位置，裁判在终点墙随机放置竖直蓝色 A4 标志及其前方的 0.8 m × 0.8 m 终点区域。正式任务由控制台或脚本启动，小车需要自主避障、找到标志，并将整车停入终点区域；不能用 RViz 设置终点。

比赛运行阶段不使用大语言模型（LLM）或 AI 智能体：导航、避障、目标识别和停车决策由车载电脑上的程序本地完成，不依赖比赛期间的远程人工指导。

当前实车导航使用建图时保存的静态地图，并用 `/scan_obstacles` 实时标记新障碍。建图时已写入静态地图的障碍物不会因为开启实时避障自动消失，因此换位后的旧障碍可能继续影响规划；当前实车入口尚未接入旧障碍地图更新。先前的 [箱子换位避障测试](docs/箱子换位避障测试_2026-10-18.md) 使用建图前移开箱子的方式，不适合作为上述比赛流程的依据。

蓝纸任务已有地图搜索及目标接近逻辑，但尚未验证覆盖比赛场地的未知终点搜索。比赛入口的搜索点超时为 90 秒，普通导航入口默认为 30 秒；均可通过 `search_goal_timeout` 调整。固定起点的地图坐标需要在建图后确定并提供给定位程序，不能假定默认的 `(0, 0, 0)` 一定正确。

比赛入口默认启用 `visual_tracking_parking_enabled`：识别蓝纸后持续结合云台水平角和目标在画面中的偏移更新接近目标的朝向，路径仍由现有导航规划器生成。按现场要求，蓝纸轮廓面积达到整幅图像的 16%，且目标方向与车头夹角不超过 5° 时结束任务停车；可通过 `visual_stop_area_ratio`、`visual_facing_tolerance_deg` 调整。规划候选点初始距估计蓝纸位置 0.48 m；若导航器已到候选点但视觉条件尚未满足，会按 0.05 m 步长尝试规划更近的点，受既有地图可通行检查和急停约束，可能无法继续接近。视觉接近期间尝试把 TEB 的位置/方向到点容差调整为 0.04 m/0.08 rad，完成、禁用或重新搜索时恢复；动态配置服务不可用时会记录警告。前向急停仍为 0.12 m。此视觉方案尚未经过实车验证，占比受视角、云台、光照和遮挡影响，达到阈值不等同于已确认整车进框。

普通导航入口默认关闭视觉停车，可用 `visual_tracking_parking_enabled:=true` 开启。另一种待实车验证的 `wall_parking_enabled:=true` 方案利用蓝纸方向和雷达墙线估计方框与车体轮廓；两者同时开启时视觉停车优先。均不改变前向急停距离。

## 无地图蓝纸追踪测试

通过障碍物后，可以单独启动云台搜索、蓝纸追踪和近场雷达避障，不运行地图、AMCL、`move_base` 或 RViz：

```bash
cd '/home/asuka/Desktop/college-level competition/xiaoche'
./start-blue-direct.sh
```

脚本会打开识别画面；已有直接控制节点运行时，换好位置和角度后再次运行即可开始下一次。检查设备和任务状态而不启动小车：`./start-blue-direct.sh --check`。停止底盘自主追踪：`./start-blue-direct.sh --stop`；摄像头和云台仍可继续运行，关闭画面窗口不会停止小车。

控制器结合云台角和蓝纸在画面中的偏移调整车头，画面中的蓝纸轮廓占比达到 16% 且目标方向与车头相差不超过 5° 时锁停；目标暂时看不到时原地搜索。雷达用于选择近场绕行方向，12 cm 前向急停及原有整车碰撞预测仍由串口桥执行。局部反应式避障没有全局地图，无法保证在所有障碍布局中找到路线；纸张测距及占比会受视角、遮挡和光照影响。

2026-11-24 的一次实车测试中，开始直接控制时蓝纸已经可见，小车沿目标前进并自主停车，用户确认整车进入黄框。停止时记录到轮廓占比约 16.08%、方向误差约 0.84°、前向雷达距离约 19.3 cm；未知起始角度搜索和障碍绕行尚需分别实车验证。

## 导航到视觉的自动切换

已完成建图并把小车放回记录的起点后，可运行 `./start-competition.sh` 一键开始；默认使用 `maps/20261125/new_field` 的地图、起点和路线。脚本启动云台搜索、导航和视觉停车，并打开摄像头画面，无需在 RViz 指定终点。运行 `./start-competition.sh --check` 查看配置和设备，按 `Ctrl+C` 或运行 `./start-competition.sh --stop` 停止。更换地图时，默认从地图所在目录读取 `start_pose.json` 和 `course_waypoints.yaml`，也可设置 `COMPETITION_MAP_FILE`、`COMPETITION_START_POSE` 和 `COMPETITION_COURSE_FILE`；当前两只 CH340 根据设备 revision 区分，更换转换器时可用 `ROBOT_BASE_DEVICE`、`ROBOT_GIMBAL_DEVICE` 指定串口。

`src/maze_bridge/launch/navigation_visual_handoff.launch` 将前段地图导航和后段直接视觉控制接在一起：导航阶段按路线途经点依次经过赛道，规划器结合地图和实时雷达绕行障碍，云台同时搜索蓝纸。摄像头看到蓝纸后，用云台角度、画面偏移和雷达估计目标方向及墙面位置。雷达确认可观测的整车通道和起步转向空间没有障碍，连续三次检查通过后，程序停止转发导航速度、取消导航目标，并切换到无地图视觉追踪。末段仍使用雷达避障和 12 cm 前向急停；16% 占比及 5° 朝向同时满足时停车。

此次路线从该场地的建图轨迹中提取了外行程途经点，路线末点是蓝纸搜索的观测点。到达末点后，若相机和雷达稳定关联了同一目标墙但切换条件尚未满足，导航可尝试一次蓝纸正前方的观测位。默认沿墙面垂线选择距墙 0.90–1.30 m、最近且满足地图净空的点，再由 `move_base` 规划。视觉接管前还要求底盘基准点到蓝纸所在墙面垂线的横向距离不超过 0.20 m。候选点不可用时，车辆保持等待并继续搜索。更换场地后需根据新建图轨迹准备对应的路线文件。

切换检查约每 0.2 s 进行一次，默认需要三次通过。相机、云台和雷达的时间戳最大相差 0.5 s；雷达墙面距离与蓝纸视觉估距默认允许相差 `max(0.25 m, 视觉距离的 20%)`。检查通道时按 STL 车身轮廓计算，目标前 0.48 m 是通道检查的几何终点，并非停车条件。雷达无效线束不会直接被视为空地，但邻近有效线束可提供观测；盲区或无法关联目标墙时会继续导航。上述参数可在 launch 中调整。

此入口需要显式提供 `map_file` 以及建图后固定起点的 `initial_pose_x`、`initial_pose_y`、`initial_pose_a`；它们是地图和起点定位参数，不是蓝纸终点坐标。默认 `auto_start:=true`，启动后会在系统就绪时开始运动；现场调试可传 `auto_start:=false`，之后向 `/blue_autonomy/enable` 发布 true。不要与独立的 `direct_visual_tracking.launch` 同时启动，避免重复打开硬件或出现多个底盘指令来源。

该自动切换已在隔离 ROS 模拟中检查障碍遮挡、速度控制权切换和视觉停车锁定，并使用此前一次无障碍实车数据回放过通道判定；尚未完成自动切换的实车全程验证。前段静态地图可能保留已经移动的障碍，这个问题不会因切换功能自动解决。视觉目标关联和二维雷达对盲区、遮挡及误识别仍有局限，通道判定不等同于整段路径绝对无障碍。

直接追踪和自动切换入口启用蓝色长竖条过滤：利用附近蓝色区域的纵向延续关系，排除被反光切碎的墙面竖条，并暂时忽略图像上下边缘处过窄的色块。它保留 0.30 的最小宽高比，但被画面边缘截断的狭窄蓝纸可能需要完整进入视野后才能识别。

这两个入口会在摄像头节点退出后尝试重启，适用于摄像头重新连接后仍映射为同一视频设备的情况；USB 线缆或接口不稳定仍需处理，设备编号改变时需要重新绑定容器。

比赛终端脚本 `bash tools/competition_terminal.sh` 提供 `prepare`、`map`、`rviz`、`turn`、`save`、`ready`、`status`、`go` 和 `stop`。除 `prepare` 之外，操作时附上同一个场次名。到达建图折返点时执行 `turn`，回起点停稳后执行 `save`。导航待命后可以用 `status` 检查云台、定位和雷达，裁判发令时执行 `go`。

如已现场确认雷达近回波来自固定在车上的部件，可在 `outputs/competition-fields/<场次名>/lidar_self_echo.yaml` 写入标定值，`ready` 会加载。角度以车头方向为零、向左为正，距离从雷达中心量起。以下是本车固定线缆的实测值。

```yaml
fixed_self_echo_bearing_deg: 126.25
fixed_self_echo_range_m: 0.357
```

过滤窗口为标定角度的正负 0.5° 和标定距离的正负 0.01 m。命中光束在代价地图中保持未观测，以保留对遮挡区域的判断。

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

摄像头默认识别蓝色 A4 目标（OpenCV HSV：H=100–130，S/V≥80）。导航和比赛入口可用 `blue_h_min`、`blue_h_max`、`blue_s_min`、`blue_v_min` 调整；独立云台入口使用 `h_min`、`h_max`、`s_min`、`v_min`。实际光照和摄像头下的效果需现场确认。导航测距默认纸张竖直高度为 0.297 m，对应竖放的 A4 纸。

本车云台向右转时报告正角度，导航默认通过 `gimbal_pan_to_ros_sign:=-1.0` 转换为 ROS 右侧负角度。更换云台安装方向时应重新核对；该参数不改变底盘电机转向。

比赛自主入口默认前向雷达停车距离为 0.12 m，可用 `safety_stop_distance:=0.15` 回退至 0.15 m；现有整车碰撞预测仍独立工作。
