#!/bin/bash

# ============================================
# 小车位姿图建图快速启动脚本
# 用法: ./build_map.sh [port:=串口设备]
# ============================================

set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_BASE_PORT="/dev/serial/by-path/pci-0000:80:14.0-usb-0:9.2:1.0-port0"
BASE_PORT="$DEFAULT_BASE_PORT"
PORT_OVERRIDDEN=false
for launch_arg in "$@"; do
    case "$launch_arg" in
        port:=*)
            BASE_PORT="${launch_arg#port:=}"
            PORT_OVERRIDDEN=true
            ;;
    esac
done

# 同时启动两套建图系统会让同名 ROS 节点互相关闭，尤其会造成
# base_footprint->laser 静态 TF 消失，最终 /map 一直为 0x0。
# 用文件锁阻止双击/重复执行，并额外拦截绕过本脚本启动的同一 launch。
LOCK_FILE="/tmp/xiaoche_build_map.lock"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "错误：已有一套 build_map.sh 正在运行。请回到原终端使用，不要重复启动。" >&2
    exit 1
fi

if pgrep -af '[r]oslaunch maze_bridge (slam_toolbox_build_map|gmapping_build_map)\.launch' >/dev/null; then
    echo "错误：检测到建图系统已在运行。请先在原终端按 Ctrl+C。" >&2
    pgrep -af '[r]oslaunch maze_bridge (slam_toolbox_build_map|gmapping_build_map)\.launch' >&2
    exit 1
fi

if pgrep -af '[r]oslaunch maze_bridge navigation_stack.launch' >/dev/null; then
    echo "错误：导航系统仍在运行，不能同时启动建图。请先在导航终端按 Ctrl+C。" >&2
    pgrep -af '[r]oslaunch maze_bridge navigation_stack.launch' >&2
    exit 1
fi

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  小车位姿图SLAM建图快速启动${NC}"
echo -e "${GREEN}========================================${NC}"

# 1. 检查环境
echo -e "\n${YELLOW}[1/5] 检查环境...${NC}"
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi
if [ -f /home/hyj/LakiBeam_ws/devel/setup.bash ]; then
    source /home/hyj/LakiBeam_ws/devel/setup.bash
fi
if [ -z "${ROS_DISTRO:-}" ]; then
    echo -e "${RED}❌ ROS未初始化，请运行: source /opt/ros/noetic/setup.bash${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ROS已初始化 ($ROS_DISTRO)${NC}"

if ! rospack find slam_toolbox >/dev/null 2>&1; then
    echo -e "${RED}❌ 未安装 ROS Noetic slam_toolbox${NC}"
    echo -e "${YELLOW}请先运行: sudo apt update && sudo apt install ros-noetic-slam-toolbox${NC}"
    exit 1
fi
echo -e "${GREEN}✓ slam_toolbox 已安装${NC}"

if [ ! -e "$BASE_PORT" ]; then
    echo -e "${RED}❌ 未找到 STM32 底盘串口: $BASE_PORT${NC}"
    echo -e "${YELLOW}请接好底盘 USB 转 TTL；若 USB 插口改变，请用下面方式指定：${NC}"
    echo -e "${YELLOW}  ./build_map.sh port:=/dev/serial/by-path/你的底盘设备${NC}"
    if [ -d /dev/serial/by-path ]; then
        echo -e "${YELLOW}当前存在的串口：${NC}"
        find /dev/serial/by-path -maxdepth 1 -type l -printf '  %p -> %l\n' 2>/dev/null || true
    fi
    exit 1
fi
if [ ! -r "$BASE_PORT" ] || [ ! -w "$BASE_PORT" ]; then
    echo -e "${RED}❌ 当前用户无权访问 $BASE_PORT，请确认用户属于 dialout 组${NC}"
    exit 1
fi
echo -e "${GREEN}✓ STM32 底盘串口: $BASE_PORT${NC}"
if command -v ip >/dev/null 2>&1 && ! ip -4 addr show | grep -q '192\.168\.198\.1'; then
    echo -e "${YELLOW}⚠ 未检测到雷达网卡地址 192.168.198.1，/scan 可能无数据${NC}"
fi

# 2. 创建必要目录
echo -e "\n${YELLOW}[2/5] 创建地图保存目录...${NC}"
mkdir -p "$WORKSPACE_DIR/maps"
echo -e "${GREEN}✓ 目录已创建: $WORKSPACE_DIR/maps${NC}"

# 3. 编译项目
echo -e "\n${YELLOW}[3/5] 编译ROS包...${NC}"
cd "$WORKSPACE_DIR"
catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)"
echo -e "${GREEN}✓ 编译成功${NC}"

# 4. 初始化ROS环境
echo -e "\n${YELLOW}[4/5] 初始化ROS环境...${NC}"
source "$WORKSPACE_DIR/devel/setup.bash"
echo -e "${GREEN}✓ ROS环境已设置${NC}"

# 5. 启动建图
echo -e "\n${YELLOW}[5/5] 启动建图系统...${NC}"
echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}建图已启动！说明：${NC}"
echo -e "  • 雷达数据在 /scan 话题"
echo -e "  • 建立的地图在 /map 话题"
echo -e "  • 当前算法为 slam_toolbox 位姿图优化和回环修正"
echo -e "  • 地图完成后请在另一终端手动保存"
echo -e "  • 保存命令: rosrun map_server map_saver -f $WORKSPACE_DIR/maps/competition_map_v3 map:=/map"
echo -e "  • RViz已打开显示实时建图结果"
echo -e "  • 如出现RViz保存错误，请忽略"
echo -e "  • 按 Ctrl+C 停止建图"
echo -e "${GREEN}================================${NC}\n"

if [ "$PORT_OVERRIDDEN" = true ]; then
    roslaunch maze_bridge slam_toolbox_build_map.launch "$@"
else
    roslaunch maze_bridge slam_toolbox_build_map.launch port:="$BASE_PORT" "$@"
fi
