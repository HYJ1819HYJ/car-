#!/usr/bin/env bash

# 迷宫小车 ROS 端一键环境安装脚本
# 支持：Ubuntu 20.04 (Focal) + ROS Noetic

set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="noetic"
LAKIBEAM_WS="${LAKIBEAM_WS:-$HOME/LakiBeam_ws}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info() { echo -e "${GREEN}[安装]${NC} $*"; }
warn() { echo -e "${YELLOW}[提示]${NC} $*"; }
die() { echo -e "${RED}[失败]${NC} $*" >&2; exit 1; }

trap 'die "第 $LINENO 行执行失败，请保留上方错误信息。"' ERR

if [ ! -r /etc/os-release ]; then
    die "无法识别操作系统。请在 Ubuntu 20.04 中运行本脚本。"
fi

# shellcheck disable=SC1091
source /etc/os-release
if [ "${ID:-}" != "ubuntu" ] || [ "${VERSION_CODENAME:-}" != "focal" ]; then
    die "ROS Noetic 原生安装要求 Ubuntu 20.04 (focal)，当前是 ${PRETTY_NAME:-未知系统}。"
fi

if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
else
    command -v sudo >/dev/null 2>&1 || die "系统缺少 sudo。"
    SUDO="sudo"
    sudo -v
fi

info "安装基础工具并配置 ROS 软件源"
$SUDO apt-get update
$SUDO apt-get install -y \
    ca-certificates curl gnupg2 lsb-release software-properties-common \
    build-essential cmake git python3-pip python3-rosdep python3-vcstool

$SUDO install -d -m 0755 /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | $SUDO gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg

ARCH="$(dpkg --print-architecture)"
ROS_SOURCE="deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main"
echo "$ROS_SOURCE" | $SUDO tee /etc/apt/sources.list.d/ros1-latest.list >/dev/null

info "安装 ROS Noetic、导航、建图、相机、串口和手柄依赖"
$SUDO apt-get update
$SUDO apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-navigation \
    ros-noetic-slam-toolbox \
    ros-noetic-gmapping \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-dwa-local-planner \
    ros-noetic-navfn \
    ros-noetic-joy \
    ros-noetic-serial \
    ros-noetic-usb-cam \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-image-view \
    ros-noetic-tf2-ros \
    ros-noetic-teleop-twist-keyboard \
    joystick v4l-utils bluez bluez-tools iproute2 libopencv-dev

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    info "初始化 rosdep"
    $SUDO rosdep init
fi
rosdep update --rosdistro="$ROS_DISTRO_NAME"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [ -f "$LAKIBEAM_WS/devel/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "$LAKIBEAM_WS/devel/setup.bash"
    info "已载入雷达工作空间：$LAKIBEAM_WS"
else
    warn "未找到 $LAKIBEAM_WS/devel/setup.bash。"
    warn "ROS 主工程可以编译，但运行雷达前必须把厂商 lakibeam1 驱动复制并编译到该目录。"
fi

info "安装 package.xml 中可自动解析的依赖"
rosdep install --from-paths "$PROJECT_DIR/src" --ignore-src -r -y \
    --rosdistro="$ROS_DISTRO_NAME" --skip-keys="lakibeam1"

info "编译工作空间"
cd "$PROJECT_DIR"
catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)"

if getent group dialout >/dev/null 2>&1 && [ "$(id -u)" -ne 0 ]; then
    $SUDO usermod -aG dialout "$USER"
fi
if getent group input >/dev/null 2>&1 && [ "$(id -u)" -ne 0 ]; then
    $SUDO usermod -aG input "$USER"
fi

add_source_line() {
    local line="$1"
    grep -Fqx "$line" "$HOME/.bashrc" 2>/dev/null || echo "$line" >> "$HOME/.bashrc"
}
add_source_line "source /opt/ros/noetic/setup.bash"
add_source_line "[ -f '$LAKIBEAM_WS/devel/setup.bash' ] && source '$LAKIBEAM_WS/devel/setup.bash'"
add_source_line "[ -f '$PROJECT_DIR/devel/setup.bash' ] && source '$PROJECT_DIR/devel/setup.bash'"

if command -v code >/dev/null 2>&1; then
    info "安装 VS Code 推荐扩展"
    for extension in \
        ms-iot.vscode-ros \
        ms-vscode.cpptools \
        ms-vscode.cmake-tools \
        ms-python.python \
        redhat.vscode-xml \
        redhat.vscode-yaml \
        timonwong.shellcheck; do
        code --install-extension "$extension" --force || warn "扩展安装失败：$extension"
    done
else
    warn "没有检测到 code 命令；安装 VS Code 后打开本仓库并选择‘安装推荐的扩展’。"
fi

echo
info "ROS 环境和本工程已安装完成。"
echo "项目目录：$PROJECT_DIR"
echo "下一步：注销并重新登录，使串口/手柄用户组权限生效。"
echo "重新登录后运行：source '$PROJECT_DIR/devel/setup.bash'"
echo "硬件连接说明：$PROJECT_DIR/src/maze_bridge/HARDWARE_SETUP.md"
