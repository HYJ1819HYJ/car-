# 迷宫小车硬件连接与功能检验指南

## 一、硬件清单

| 部件 | 连接方式 | 状态 |
|------|---------|------|
| 树莓派/Jetson Nano (上位机) | 运行 ROS | ✅ 你正在使用 |
| STM32 下位机 (电机驱动+编码器) | USB 串口 `/dev/ttyUSB0` | ✅ 已检测到 |
| Lakibeam1 激光雷达 | 有线网卡 (上位机 192.168.198.1，雷达 192.168.198.2) | 已确认 |
| USB 摄像头 | USB | ✅ 已检测到 (ASUS FHD webcam) |
| 蓝牙手柄 | 蓝牙适配器(内置) | ✅ 蓝牙可用 (Intel) |

---

## 二、第一步：安装缺失依赖

```bash
# 1. 安装 ROS joy 包（手柄驱动）
sudo apt install ros-noetic-joy

# 2. 安装 joystick 工具（检测手柄按键）
sudo apt install joystick

# 3. 安装蓝牙管理工具（如果缺失）
sudo apt install bluetooth bluez bluez-tools

# 4. 编译工作空间
cd ~/xiaoche
catkin_make
source devel/setup.bash
```

---

## 三、第二步：连接蓝牙手柄

### 方法 1：使用 bluetoothctl 配对

```bash
# 1. 打开蓝牙
sudo bluetoothctl
power on
agent on
default-agent
scan on

# 2. 看到手柄的 MAC 地址后，例如 XX:XX:XX:XX:XX:XX
#    复制 MAC 地址，执行配对
scan off
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
exit

# 3. 验证手柄设备是否出现
ls /dev/input/js*
# 应该看到 /dev/input/js0
```

### 方法 2：使用图形界面蓝牙设置

Ubuntu 桌面 → 设置 → 蓝牙 → 打开 → 搜索手柄 → 配对

### 方法 3：使用 PS4/PS5 手柄专用工具

```bash
# PS4 手柄
sudo apt install ds4drv
ds4drv

# Xbox 手柄（已支持内核驱动，直接配对即可）
```

### 验证手柄

连接后运行：
```bash
# 查看手柄输入
sudo jstest /dev/input/js0

# 摇动左摇杆、按按钮，看数值变化
# 记录对应的 axis/button 编号
```

---

## 四、第三步：分步检验各功能

### 检验 1：串口通信（STM32 桥）

```bash
# 终端 1：启动 ROS
roscore

# 终端 2：启动串口桥
rosrun stm32_bridge serial_bridge_node

# 终端 3：发送速度指令
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'

# 观察 STM32 是否响应（电机转动）
# 查看反馈：
rostopic echo /stm32_feedback_text
rostopic echo /wheel_encoder

# 如果要停止小车：
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

### 检验 2：激光雷达

```bash
# 启动雷达（需要先设置网络 IP）
roslaunch maze_bridge lakibeam1_scan_view_usb.launch

# 在另一个终端查看激光数据
rostopic echo /scan | head -20

# 如果能看到数据，说明雷达正常
# Rviz 会自动弹出显示点云
```

### 检验 3：USB 摄像头

```bash
roslaunch maze_bridge usb_cam.launch

# 查看图像话题
rosrun image_view image_view image:=/usb_cam/image_raw

# 或者用 rqt：
rqt_image_view /usb_cam/image_raw
```

### 检验 4：蓝牙手柄遥操作

```bash
# 先确保手柄已配对并出现 /dev/input/js0
ls /dev/input/js0

# 安装 joy 包（如果还没装）
sudo apt install ros-noetic-joy

# 测试手柄 Raw 输入
rosrun joy joy_node
rostopic echo /joy
# 摇动摇杆、按按钮，观察数据变化
# 记下 axis 和 button 的索引号

# 如果没有手柄，也可以用键盘模拟：
sudo apt install ros-noetic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 检验 5：手柄遥操作 + 教学记录（完整流程）

#### 场景 A：有小车硬件（完整测试）

```bash
# 一键启动：手柄控制 + 记录
roslaunch maze_bridge teach_teleop_complete.launch

# 按住 L1（左肩键），用左摇杆控制小车走迷宫
# 走完后轨迹保存在 ~/.ros/maze_teach.csv

# 然后启动复现：
roslaunch maze_bridge teach_teleop_replay.launch
```

#### 场景 B：没有小车硬件（纯软件测试）

```bash
# 终端 1：启动核心
roscore

# 终端 2：启动手柄遥操作（不依赖硬件）
rosrun maze_planner joy_teleop.py

# 终端 3：启动教学记录节点（不依赖硬件）
rosrun maze_planner teach_repeat_node _mode:=record

# 终端 4：模拟手柄输入
rostopic pub /joy sensor_msgs/Joy \
  '{axes: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
    buttons: [0,0,0,0,1,0,0,0,0,0,0,0,0]}' -r 20

# 终端 5：查看记录下发的速度
rostopic echo /cmd_vel_in
rostopic echo /cmd_vel

# 终端 6：查看 CSV 记录文件
cat ~/.ros/maze_teach.csv
```

### 检验 6：自主探索迷宫（不依赖手柄）

```bash
# 完整启动探索模式
roslaunch maze_bridge bringup.launch maze_mode:=explore

# 观察小车自主走迷宫
# 到终点后查看路径文件：
cat ~/.ros/maze_path.txt

# 然后启动复现模式：
roslaunch maze_bridge bringup.launch maze_mode:=replay
```

---

## 五、参数调优指南

### 常见调优参数

| 参数 | 默认值 | 说明 | 调优建议 |
|------|--------|------|---------|
| `front_stop_m` | 0.35 m | 前方安全距离 | 小车越大设越大 |
| `side_open_m` | 0.45 m | 侧方判定为"有空"的距离 | 路宽的一半 |
| `v_forward` | 0.18 m/s | 探索前进速度 | 先慢后快 |
| `w_turn` | 0.9 rad/s | 转弯角速度 | 约 50°/s |
| `turn_ticks_need` | 12 ticks | 转弯时长 (20Hz×12=0.6s) | 根据实际转角调整 |
| `goal_ratio_threshold` | 8% | 绿色终点检测灵敏度 | 环境光线影响大 |

### 转弯角度校准

```bash
# 让小车原地转一圈，数需要多少 ticks
roslaunch maze_bridge bringup.launch maze_mode:=explore

# 如果转得不够/转过了，调整 turn_ticks_need
# 每个 tick 约 50ms
# 90° 转弯 ≈ 需要 10-15 ticks
```

---

## 六、常见问题排查

### Q1: `/dev/ttyUSB0` 权限不够
```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### Q2: 雷达连不上
检查网络配置：
```bash
# 当前 Lakibeam1 IP: 192.168.198.2
# 当前上位机有线网卡 enp130s0: 192.168.198.1/24
# 接口名请先通过 ip -brief address 确认，不要假设为 eth0。
```

### Q3: 蓝牙手柄配对后没反应
```bash
# 查看蓝牙手柄连接状态
bluetoothctl info

# 如果显示已配对但没连接
bluetoothctl connect XX:XX:XX:XX:XX:XX

# 查看手柄设备
ls -la /dev/input/
# 应该看到 js0 或 event* 设备
```

### Q4: 绿色终点检测不准
调整 HSV 范围：
```cpp
// 在 maze_controller_node.cpp 的 imgCb() 中修改
cv::Scalar lo(35, 80, 80), hi(85, 255, 255);
// 先用 HSV 调试工具找到实际绿色的值
```

### Q5: 编译报错找不到 joy 消息
```bash
sudo apt install ros-noetic-joy
# joy 包提供 sensor_msgs/Joy 消息类型
```

---

## 七、快速启动备忘录

```bash
# 1. 编译
cd ~/xiaoche && catkin_make && source devel/setup.bash

# 2. 检查硬件
ls /dev/ttyUSB0 /dev/input/js0

# 3. 一键启动手柄遥操作教学
roslaunch maze_bridge teach_teleop_complete.launch

# 4. 复现
roslaunch maze_bridge teach_teleop_replay.launch

# 5. 或使用自主探索
roslaunch maze_bridge bringup.launch maze_mode:=explore
roslaunch maze_bridge bringup.launch maze_mode:=replay
```
