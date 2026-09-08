#!/bin/bash

# 小车迷宫竞赛 - 现在就测试脚本
# 这个脚本会引导你完成所有必要的测试

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║         小车迷宫竞赛 - 系统测试指南                                      ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════════════════╝${NC}"

echo -e "\n${YELLOW}[准备工作]${NC}"
echo "1. 确保小车和蓝牙手柄都已打开"
echo "2. 确保小车放在平坦的地面上"
echo "3. 确保蓝牙手柄已配对"
echo ""
read -p "准备好了吗? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${RED}放弃测试${NC}"
    exit 1
fi

echo -e "\n${YELLOW}[第1步] 检查手柄设备${NC}"
if [ -e /dev/input/js0 ]; then
    echo -e "${GREEN}✓ 手柄设备存在: /dev/input/js0${NC}"
else
    echo -e "${RED}✗ 手柄设备不存在!${NC}"
    echo "  运行: ls /dev/input/js*"
    echo "  然后重新配对蓝牙手柄"
    exit 1
fi

echo -e "\n${YELLOW}[第2步] 检查串口${NC}"
if [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}✓ 串口存在: /dev/ttyUSB0${NC}"
else
    echo -e "${RED}✗ 串口不存在!${NC}"
    echo "  检查USB连接和权限"
    exit 1
fi

echo -e "\n${YELLOW}[第3步] 进入项目目录并编译${NC}"
cd "$WORKSPACE_DIR"
if [ -d "devel" ]; then
    echo -e "${GREEN}✓ 已编译过${NC}"
else
    echo -e "${YELLOW}首次编译，请稍候...${NC}"
    catkin_make -j4 2>&1 | tail -3
fi

echo -e "\n${YELLOW}[第4步] 初始化ROS环境${NC}"
source devel/setup.bash
echo -e "${GREEN}✓ ROS环境已初始化${NC}"

echo -e "\n${BLUE}════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}现在可以开始测试了!${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════════════════${NC}"

echo -e "\n${YELLOW}【测试1️⃣ - 手柄记录模式】${NC}"
echo "用途: 用蓝牙手柄遥控小车，记录轨迹"
echo "命令:"
echo "  $ roslaunch maze_bridge teach_teleop_complete.launch"
echo ""
echo "操作步骤:"
echo "  1. RViz窗口打开后，确保能看到激光扫描数据（黄色点）"
echo "  2. 按住蓝牙手柄的 L1 键"
echo "  3. 用左摇杆控制小车:"
echo "     - 摇杆前推 → 前进"
echo "     - 摇杆后拉 → 后退"
echo "     - 摇杆左右 → 转向"
echo "  4. 手动牵引小车走完整个迷宫 (至少30秒)"
echo "  5. 按 Ctrl+C 停止程序"
echo "  6. 验证: ls -lh ~/.ros/maze_teach.csv"
echo "     应该显示一个有实际大小的文件"
echo ""
read -p "要现在启动测试1吗? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}启动手柄记录模式...${NC}"
    echo "提示: 用手柄遥控小车，然后按 Ctrl+C 停止"
    read -p "按 Enter 继续..."
    
    roslaunch maze_bridge teach_teleop_complete.launch
    
    echo -e "\n${YELLOW}检查轨迹文件...${NC}"
    if [ -f ~/.ros/maze_teach.csv ]; then
        SIZE=$(stat -c%s ~/.ros/maze_teach.csv)
        if [ $SIZE -gt 10000 ]; then
            echo -e "${GREEN}✓ 轨迹文件成功生成 (大小: $SIZE 字节)${NC}"
            LINES=$(cat ~/.ros/maze_teach.csv | wc -l)
            echo -e "${GREEN}✓ 记录了 $LINES 行数据${NC}"
        else
            echo -e "${RED}⚠ 轨迹文件太小，请重新记录${NC}"
        fi
    else
        echo -e "${RED}✗ 轨迹文件未生成${NC}"
    fi
fi

echo -e "\n${YELLOW}【测试2️⃣ - 自动复现模式】${NC}"
echo "用途: 小车自动按照刚才记录的轨迹走动"
echo "命令:"
echo "  $ roslaunch maze_bridge teach_teleop_replay.launch"
echo ""
echo "操作步骤:"
echo "  1. 将小车放在起始位置（刚才开始的位置）"
echo "  2. 运行上述命令"
echo "  3. 观察小车是否按照相同的轨迹走动"
echo "  4. 检查激光雷达数据是否实时更新"
echo "  5. 按 Ctrl+C 停止程序"
echo ""
read -p "要现在启动测试2吗? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    if [ -f ~/.ros/maze_teach.csv ]; then
        echo -e "${YELLOW}启动自动复现模式...${NC}"
        echo "提示: 观察小车的运动，然后按 Ctrl+C 停止"
        read -p "按 Enter 继续..."
        
        roslaunch maze_bridge teach_teleop_replay.launch
        
        echo -e "\n${GREEN}✓ 复现测试完成${NC}"
    else
        echo -e "${RED}✗ 没有找到轨迹文件，请先运行测试1${NC}"
    fi
fi

echo -e "\n${BLUE}════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}测试完成！${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════════════════${NC}"

echo -e "\n${YELLOW}【测试结果评估】${NC}"
echo ""
echo "如果以上测试都成功了:"
echo -e "  ${GREEN}✓ 你的系统已经准备好参加比赛了！${NC}"
echo ""
echo "如果有测试失败:"
echo "  查看 建图问题诊断和解决.md 文件中的故障排查部分"
echo ""
echo -e "\n${YELLOW}【下一步】${NC}"
echo "1. 在实际迷宫环境中测试 3-5 次"
echo "2. 优化手柄控制，找到最快的路线"
echo "3. 调整参数以改进复现精度"
echo "4. 准备比赛日的设备和代码"
echo ""
echo -e "${GREEN}祝你比赛顺利！🚀${NC}"
