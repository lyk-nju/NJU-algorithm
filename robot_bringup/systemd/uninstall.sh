#!/bin/bash
set -e

echo "=== RoboMaster 开机自启动卸载脚本 ==="

# 停止并禁用服务
systemctl --user stop robot_system.target 2>/dev/null || true
systemctl --user disable robot_system.target 2>/dev/null || true
systemctl --user stop robot_nav_healthcheck.timer 2>/dev/null || true
systemctl --user disable robot_nav_healthcheck.timer 2>/dev/null || true

# 删除服务文件
rm -f ~/.config/systemd/user/robot_vision.service
rm -f ~/.config/systemd/user/robot_nav.service
rm -f ~/.config/systemd/user/robot_system.target
rm -f ~/.config/systemd/user/robot_nav_healthcheck.service
rm -f ~/.config/systemd/user/robot_nav_healthcheck.timer

# 重新加载 systemd
systemctl --user daemon-reload

echo "已卸载 robot_system 自启动服务"
echo "注意: 源代码文件仍保留在 ~/.config/systemd/user/ 备份目录"
