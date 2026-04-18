## 1. 项目文件树
vision_task/
├── assets/
│   ├── armor/
│   │   └── armor_video.avi
│   └── buff/
│       └── buff_video.avi
├── config/
│   ├── hero.yaml
│   ├── sentry.yaml
│   ├── infantry.yaml
│   └── uav.yaml
├── cuda/
├── io/
│   ├── camera/
│   │   ├── usbcamera/
│   │   └── hiki/
│   ├── serial/
│   ├── structs/
│   ├── interface/
│   ├── algorithm/
│   ├── driver/        # cboard / camera(+direct_image_source) / ros2(+ros2_image_source)
│   └── serial/
├── models/
├── tasks/
│   ├── auto_aim/
│   │   ├── armor.hpp
│   │   ├── aimer.cpp/hpp
│   │   ├── detector.cpp/hpp
│   │   ├── tracker.cpp/hpp
│   │   ├── target.cpp/hpp
│   │   ├── pnp_solver.cpp/hpp
│   │   ├── ekf.cpp/hpp
│   │   ├── trajectory.cpp/hpp
│   │   └── decider.cpp/hpp
│   └── buff/
│       ├── buff.hpp
│       ├── aimer.cpp/hpp
│       ├── detector.cpp/hpp
│       ├── tracker.cpp/hpp
│       ├── target.cpp/hpp
│       └── pnp_solver.cpp/hpp
├── test/
│   └── ekf.cpp/hpp
├── deploy/
│   ├── auto_aim/
│   └── buff/
├── video/
│   ├── auto_aim/
│   └── buff/
└── tools/
    ├── logger/
    ├── draw/
    └── math/

---

## 2. 模块具体介绍
资源与配置
assets/：存储装甲板（armor）和能量机关（buff）相关的测试视频资源。

config/：包含英雄（hero）、哨兵（sentry）、步兵（infantry）及无人机（uav）等不同兵种的配置文件。

通信与输入输出 (io/)
camera/：支持驱动 USB 相机及海康（hiki）相机。

serial/ & usb/：处理串口与 USB 通信，其中包含针对 ROS 2 的封装。

数据定义（io/structs/）
通信协议与上层决策所需的基础数据结构（命令、反馈、裁判系统、图像帧、相机内外参等）。结构体零依赖，可被任意层复用。

数据转换（io/algorithm/ros2_transfer.hpp）
完成不同接口（例如 USB 串口与 ROS2 消息）之间的数据转换方法；不依赖具体实现类（如 `Cboard` / `ROS2Manager`）。

核心任务 (tasks/)
auto_aim/：包含基于第三方库封装的通信层 aimer.cpp/hpp。核心逻辑涵盖目标检测（detector）、追踪（tracker）、位姿解算（pnp_solver）、卡尔曼滤波（ekf）及弹道轨迹计算（trajectory）。

decider.cpp/hpp：全向感知决策模块。

buff/：能量机关任务模块，包含专门的检测、追踪及位姿解算逻辑。

开发辅助与部署
test/：针对关键算法（如 EKF）的独立测试模块。

deploy/：存放自动瞄准及能量机关的部署相关文件。

video/：存储任务运行过程中的自瞄或能量机关视频数据。

tools/：通用工具类，包括日志（logger）、绘图（draw）及数学（math）库。

1. 待办与优化建议 (Notes)
数据逻辑封装：考虑将 tools/transfer 的数据转换逻辑封装到 io 模块中，以统一处理通信数据。

模块拆分：考虑将 auto_aim_system 继续拆分为更小的功能模块以优化结构。