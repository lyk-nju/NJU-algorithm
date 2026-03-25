# RoboMaster 开机自启动配置

## 已创建的文件

```
robot_bringup/systemd/
├── robot_vision.service   # 自瞄服务 (auto_aimer_test)
├── robot_nav.service      # 导航+决策服务 (robot_bringup.launch.py)
├── robot_system.target     # 统一目标
├── robot_nav_healthcheck.service  # 导航健康检查（oneshot）
├── robot_nav_healthcheck.timer    # 定时巡检并自动拉起导航服务
├── install.sh             # 安装脚本
└── uninstall.sh           # 卸载脚本
```

## 安装步骤

```bash
cd /home/nvidia/NJU-algorithm/robot_bringup/systemd
sudo loginctl enable-linger nvidia  # 允许用户服务开机自启
bash install.sh
```

## 常用命令

```bash
# 查看服务状态
systemctl --user status robot_vision.service
systemctl --user status robot_nav.service

# 查看日志
journalctl --user -u robot_vision.service -f
journalctl --user -u robot_nav.service -f
journalctl --user -u robot_nav_healthcheck.service -f

# 手动启动/停止
systemctl --user start robot_system.target
systemctl --user stop robot_system.target

# 健康检查状态
systemctl --user status robot_nav_healthcheck.timer

# 卸载自启动
bash uninstall.sh
```

## 服务启动顺序

1. `robot_system.target` 拉起 `robot_vision.service` 与 `robot_nav.service`
2. `robot_vision.service` 托管 `auto_aimer_test`（异常自动重启）
3. `robot_nav.service` 依赖 `robot_vision.service`，启动 `robot_bringup.launch.py`

## 注意事项

- 服务使用 journal 日志，可通过 `journalctl --user` 查看
- 串口 `/dev/ttyACM0` 需确保有权限: `sudo usermod -a -G dialout nvidia`
- 确保 ROS2 环境变量在 `~/.bashrc` 中正确配置
