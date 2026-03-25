#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEMD_USER_DIR="/etc/systemd/system"
CURRENT_USER=$(whoami)

if [ "$CURRENT_USER" = "root" ]; then
    echo "错误: 请不要使用 root 用户运行此脚本"
    exit 1
fi

echo "=== RoboMaster 开机自启动安装脚本 ==="

# 创建 systemd 服务目录（用户级服务）
mkdir -p ~/.config/systemd/user

# 复制服务文件到用户目录
cp "$SCRIPT_DIR/robot_vision.service" ~/.config/systemd/user/
cp "$SCRIPT_DIR/robot_nav.service" ~/.config/systemd/user/
cp "$SCRIPT_DIR/robot_system.target" ~/.config/systemd/user/
cp "$SCRIPT_DIR/robot_nav_healthcheck.service" ~/.config/systemd/user/
cp "$SCRIPT_DIR/robot_nav_healthcheck.timer" ~/.config/systemd/user/

echo "服务文件已复制到 ~/.config/systemd/user/"

# 重新加载 systemd
systemctl --user daemon-reload

# 启用开机自启动
systemctl --user enable robot_system.target
systemctl --user enable robot_nav_healthcheck.timer
echo "已启用 robot_system.target 开机自启动"

# 立即启动服务（可选）
read -p "是否立即启动服务进行测试? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    systemctl --user start robot_system.target
    echo "服务已启动"
    echo ""
    echo "查看服务状态:"
    systemctl --user status robot_vision.service
    systemctl --user status robot_nav.service
fi

echo ""
echo "=== 安装完成 ==="
echo "常用命令:"
echo "  查看状态: systemctl --user status robot_vision.service"
echo "  查看日志: journalctl --user -u robot_vision.service -f"
echo "  停止服务: systemctl --user stop robot_system.target"
echo "  禁用自启: systemctl --user disable robot_system.target"
