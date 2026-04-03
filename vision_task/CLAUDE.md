# RoboMaster NJU 视觉自瞄系统 - 项目文档

## 项目概述

本项目是南京大学（NJU）RoboMaster 战队的视觉自动瞄准系统，运行于哨兵机器人（Sentry）。系统通过相机实时检测敌方装甲板，结合 IMU 数据进行三维位姿估计，使用扩展卡尔曼滤波器（EKF）跟踪目标，最终输出炮台控制命令。

**当前分支**：`sentry_leo`（哨兵机器人版本）

---

## 系统架构

整体数据流为单线程流水线：

```
Camera → Detector → IMU Sync → PnP Solver → Tracker (EKF) → Aimer → Serial Output
```

```
┌─────────────┐
│   相机输入   │  ROS2 话题 或 视频文件
└──────┬──────┘
       ▼
┌──────────────────┐
│    Detector      │  YOLO (TensorRT/OpenCV-DNN) + CUDA 预处理
│  装甲板检测       │  输出：ArmorArray（2D 检测结果 + 关键点）
└──────┬───────────┘
       ▼
┌──────────────────┐
│   IMU 同步        │  查询历史 IMU 缓冲，匹配图像时间戳
│  AutoAimSystem   │  输出：R_gimbal2world（云台到世界系旋转矩阵）
└──────┬───────────┘
       ▼
┌──────────────────┐
│   PnP Solver     │  solvePnP (IPPE) + 坐标系链式变换
│  位姿估计         │  输出：每块装甲板的 3D 位置和姿态
└──────┬───────────┘
       ▼
┌──────────────────┐
│    Tracker       │  数据关联 + 状态机
│  多目标跟踪       │  输出：当前跟踪目标
└──────┬───────────┘
       ▼
┌──────────────────┐
│  Target + EKF    │  11 维状态向量，运动预测
│  目标状态估计     │  输出：更新后的状态（位置、速度、偏航、角速度等）
└──────┬───────────┘
       ▼
┌──────────────────┐
│     Aimer        │  落点预测（迭代弹道解算）+ 瞄准点选择
│  弹道解算         │  输出：云台目标 yaw/pitch 角度
└──────┬───────────┘
       ▼
┌──────────────────┐
│  Serial Manager  │  USB/UART 通信，发送控制命令到 STM32
└──────────────────┘
```

---

## 目录结构

```
vision_task/
├── armor_task/               # 主自瞄系统（核心）
│   ├── CMakeLists.txt
│   ├── config/
│   │   ├── camera1.yaml      # 主标定与策略参数
│   │   ├── deploy_test.yaml  # 部署配置（端口、模型路径）
│   │   └── video_test.yaml   # 视频测试配置
│   ├── cuda/                 # CUDA 内核
│   │   ├── yolo_decode.cu    # GPU 解码 + NMS
│   │   ├── yolo_decode.cuh
│   │   └── yolo_types.hpp    # GPU 数据结构
│   ├── include/
│   │   ├── armor.hpp         # Armor, LightBar 结构体
│   │   ├── car.hpp           # Robot 类
│   │   └── structures.hpp    # 统一头文件
│   ├── io/
│   │   ├── serial_manager.*  # STM32 串口通信
│   │   ├── ros2_manager.*    # ROS2 相机接口
│   │   └── keyboard_manager.* # 键盘调试输入
│   ├── models/
│   │   └── best.engine       # TensorRT 序列化模型
│   ├── src/
│   │   └── main.cpp          # 程序入口
│   └── tasks/
│       ├── detector.*        # YOLO 检测器
│       ├── pnp_solver.*      # PnP 位姿估计
│       ├── tracker.*         # 目标跟踪状态机
│       ├── target.*          # EKF 目标状态估计
│       ├── ekf.*             # 通用 EKF 实现
│       ├── aimer.*           # 弹道解算与瞄准
│       ├── auto_aim_system.* # 系统总调度 + IMU 同步
│       ├── trajectory_normal.* # 简单抛物线弹道模型
│       ├── trajectory_rk4.*    # RK4 阻力弹道模型
│       └── tools/
│           ├── math_tools.*  # 数学工具（旋转、坐标变换）
│           └── draw.*        # OpenCV 可视化
├── hiki_ros2/                # 相机 ROS2 驱动
└── camera_calibration/       # 相机标定工具
```

---

## 核心算法详解

### 1. 装甲板检测（Detector）

**文件**：`armor_task/tasks/detector.cpp|hpp`

#### 推理后端
系统支持两种推理后端：
- **TensorRT 引擎**（`.engine` 文件）：GPU 加速推理，用于部署
- **OpenCV DNN**（`.onnx`）：CPU 回退，用于调试

#### CUDA 预处理流水线
```
原图 → Letterbox 缩放（GPU） → 归一化 → NHWC→NCHW → TensorRT 推理
```
- 目标尺寸：640×640
- 缩放因子：`scale = min(640/W, 640/H)`
- 填充量：`pad_x = (640 - W*scale)/2`

#### 模型输出解析
- 8400 个锚点 × 40 通道（坐标 + 置信度 + 36 类 + 8 关键点）
- 置信度阈值：0.55
- NMS 阈值：0.5

#### CUDA 内核（`yolo_decode.cu`）
三阶段 GPU 处理：
1. **decode_kernel**：并行解析每个锚点，原子操作累计有效框
2. **nms_kernel**：IoU 过滤抑制重复框
3. **compact_kernel**：压缩输出最终结果

```cpp
struct GPUBoudingBox {
    float cx, cy, w, h;  // 中心坐标 + 宽高
    float score;          // 置信度
    int class_id;         // 装甲板类别（0-35，对应颜色+数字）
    float kpts[8];        // 4 个角点关键点 (x1,y1,...,x4,y4)
};
```

---

### 2. 位姿估计（PnP Solver）

**文件**：`armor_task/tasks/pnp_solver.cpp|hpp`

#### 3D 物理尺寸
```
小装甲板：135mm × 56mm
大装甲板：230mm × 56mm
```
物体坐标系原点在装甲板中心，Z 轴指向外法线方向。

#### 坐标系变换链
```
图像坐标系 → [solvePnP, IPPE] → 相机坐标系
        → [R_camera2gimbal, t_camera2gimbal] → 云台坐标系
        → [R_gimbal2world (IMU 四元数)] → 世界坐标系（大地系）
```

关键矩阵（来自 `camera1.yaml`）：
- `camera_matrix`：3×3 相机内参矩阵
- `distort_coeffs`：畸变系数 [k1, k2, p1, p2, k3]
- `R_camera2gimbal`：相机到云台的旋转（[0,0,1,-1,0,0,0,-1,0]，相机竖装）
- `t_camera2gimbal`：相机到云台的平移 [0.1012, 0.0523, 0.0] m

**特殊处理**：NJU 哨兵 IMU 横置，pitch/roll 轴对调，在 `R_gimbal2imubody` 中修正。

---

### 3. 目标跟踪状态机（Tracker）

**文件**：`armor_task/tasks/tracker.cpp|hpp`

```
LOST ──(连续检测 min_detect_count 帧)──> DETECTING
DETECTING ──(稳定)──> TRACKING
TRACKING ──(目标消失)──> TEMP_LOST
TEMP_LOST ──(超过 max_temp_lost_count 帧)──> LOST
TEMP_LOST ──(重新检测到)──> TRACKING
```

每帧处理逻辑：
1. 按颜色过滤（去除非敌方装甲板）
2. 按到图像中心距离排序
3. 数据关联（角度匹配最近装甲板）
4. EKF 预测 + 更新
5. 发散检测（半径估计超出物理范围则重置）

---

### 4. EKF 目标状态估计（Target + EKF）

**文件**：`armor_task/tasks/target.cpp|hpp`, `ekf.cpp|hpp`

#### 状态向量（11 维）
```
x = [xc, vxc, yc, vyc, zc, vzc, yaw, ω, r, Δl, Δh]

xc, yc, zc   — 旋转中心位置（世界系，m）
vxc, vyc, vzc — 中心速度（m/s）
yaw          — 装甲板偏航角（rad）
ω            — 角速度（rad/s）
r            — 旋转半径（m）
Δl           — 长短轴半径差（多装甲板机器人，m）
Δh           — 高度差（前后两块装甲板高度差，m）
```

#### 运动模型（匀速转动）
```
xc(k+1) = xc(k) + vxc * dt
vxc(k+1) = vxc(k)
yaw(k+1) = yaw(k) + ω * dt
ω(k+1) = ω(k)
r(k+1) = r(k)
...（依此类推）
```

#### 观测模型
观测量 z = [yaw_球面, pitch_球面, 距离, armor_yaw]

装甲板 i 的预测位置（从旋转中心推算）：
```
angle_i = yaw + i * (2π / num_armors)
armor_x = center_x - r_i * cos(angle_i)
armor_y = center_y - r_i * sin(angle_i)
armor_z = center_z + (long_radius ? Δh : 0)
```

#### 过程噪声（Q 矩阵，对角）
```cpp
v1 = 100        // 位置加速度方差
v1_y = 100      // y 轴加速度（装甲板切换时较大）
v2 = 400        // 角加速度方差
```

#### EKF 更新方程
```
创新：    y = z - h(x)
卡尔曼增益：K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹
状态更新：  x = x + K·y
协方差更新（Joseph 形式，保证正定性）：
           P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ
```

**滤波器一致性检验**：
- **NIS**（归一化新息平方）：χ² 检验，检测滤波器是否一致
- **NEES**（归一化估计误差平方）：一致性统计量

---

### 5. 弹道解算与瞄准（Aimer）

**文件**：`armor_task/tasks/aimer.cpp|hpp`

#### 总体流程
```
延迟补偿 → 瞄准点选择 → 迭代弹道解算 → 输出角度
```

#### 延迟补偿
- **检测延迟**：算法处理时间
- **发射延迟**：`low_speed_delay_time = 0.1275s`（ω < decision_speed）
              `high_speed_delay_time = 0.1275s`（ω ≥ decision_speed）
- 使用 EKF 预测目标在 `t + delay` 时刻的状态

#### 瞄准点选择策略
- **静止/低速旋转**：瞄准当前朝向装甲板
- **高速旋转**（ω > 2 rad/s）：选择正前方（60° 范围内）的装甲板
- 根据 `comming_angle`/`leaving_angle` 区分来向/去向装甲板

#### 迭代弹道解算（最多 10 次迭代）
```python
for i in range(10):
    trajectory.solve(aim_point)      # 计算弹道（飞行时间 fly_time）
    predicted = ekf.predict(fly_time) # 预测目标移动
    new_aim = predicted.get_armor_pos()
    if |fly_time_new - fly_time| < 0.001:
        break
```

#### 两种弹道模型

**简单抛物线模型**（`trajectory_normal`）：
```
y = x·tan(θ) - g·x²/(2·v₀²·cos²(θ))
解二次方程得 θ，取飞行时间最短的解
```

**RK4 阻力模型**（`trajectory_rk4`）：
```
加速度 = -k·|v|·v - g·ẑ
使用 4 阶 Runge-Kutta 数值积分，步长 dt=0.01s
二分/牛顿法迭代求初始仰角
```
- 空气阻力系数 k = 0.005 kg/m

#### 最终角度输出
```cpp
yaw   = atan2(target_y, target_x) + yaw_offset    // yaw_offset=3.7°（手动标定）
pitch = -(trajectory.pitch + pitch_offset)          // 负号：世界系向上为正
```

---

### 6. IMU 同步（AutoAimSystem）

**文件**：`armor_task/tasks/auto_aim_system.cpp|hpp`

相机曝光时刻与 IMU 上报时刻不同步，需根据图像时间戳从历史缓冲中查找最近的 IMU 数据：

```cpp
class IMUHistory {
    std::deque<IMUTimestamp> buffer_;  // 最多保存 100 帧
    // 返回最接近 target_time 的四元数
    bool query(target_time, &quat, &yaw, &pitch);
};
```

缓冲用互斥锁保护，支持异步 IMU 更新线程。

---

## IO 接口

### 串口通信（Serial Manager）
- **波特率**：115200
- **端口**：通常 `/dev/ttyACM0`
- **接收**（来自 STM32）：IMU 四元数、云台 yaw/pitch、比赛时间、血量、机器人 ID
- **发送**（到 STM32）：云台目标 yaw、pitch、是否开火

### ROS2 接口（ROS2 Manager）
- **订阅** `/image_raw`（sensor_msgs/Image）：相机图像
- **发布** `/autoaim_data`（std_msgs/Float32MultiArray）：自瞄结果遥测

### 键盘调试（Keyboard Manager）
测试模式下非阻塞键盘控制：
```
W/S — pitch ±5°    A/D — yaw ±5°
P   — 单次射击       Q   — 退出
```

---

## 配置参数

### camera1.yaml（主配置）

| 参数 | 值 | 说明 |
|------|-----|------|
| `camera_matrix` | [1711.31, 0, 732.48, 0, 1714.61, 546.93, 0, 0, 1] | 相机内参 |
| `distort_coeffs` | [-0.1199, -0.0786, 0.0075, -0.0280, 0] | 畸变系数 |
| `R_camera2gimbal` | [0,0,1,-1,0,0,0,-1,0] | 相机竖装旋转 |
| `t_camera2gimbal` | [0.1012, 0.0523, 0.0] | 相机偏移量(m) |
| `enemy_color` | blue | 敌方颜色 |
| `min_detect_count` | 5 | 进入跟踪所需帧数 |
| `max_temp_lost_count` | 15 | 允许丢失帧数 |
| `yaw_offset` | 3.7° | yaw 手动标定偏差 |
| `comming_angle` | 60.0° | 来向装甲板选择角 |
| `high_speed_delay_time` | 0.1275s | 高速旋转发射延迟 |

### deploy_test.yaml（部署配置）

| 参数 | 说明 |
|------|------|
| `send_port` / `receive_port` | 串口设备路径 |
| `yolo_model_path` | TensorRT 模型路径 |
| `bullet_speed` | 子弹初速（m/s），默认 22 |
| `config_path` | 指向 camera1.yaml |

---

## 构建系统

**要求**：
- C++17
- CUDA（用于 TensorRT 推理和预处理）
- OpenCV 4.x（含 CUDA 支持）
- Eigen3
- yaml-cpp
- ROS2 Humble（rclcpp, sensor_msgs, geometry_msgs）
- TensorRT
- spdlog

**构建**：
```bash
cd armor_task
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

**可执行目标**：
- `video_test`：单机测试（无 ROS2，读取视频文件）
- `deploy_test`：完整部署版（ROS2 + 串口）

---

## 关键数据结构

```cpp
// armor.hpp
struct LightBar {
    cv::Point2f center, top, bottom;  // 灯条几何
    Color color;                       // red / blue / purple
};

struct Armor {
    cv::Rect box;                       // YOLO 检测框
    int car_num;                        // 数字识别结果
    float confidence;                   // 置信度
    cv::Point2f center;                 // 图像中心
    std::vector<cv::Point2f> corners;   // 4 个角点
    LightBar left_lightbar, right_lightbar;

    // 三维信息（PnP 求解后填充）
    Eigen::Vector3d p_camera, p_gimbal, p_world;
    Eigen::Vector3d ypr_in_gimbal, ypr_in_world;
    Eigen::Vector3d ypd_in_world;  // 球面坐标 (yaw, pitch, dist)
    double r = 0.3;                // 旋转半径估计
    bool islarge;                  // 大/小装甲板
};

// io/serial_manager.hpp
struct Command {
    bool valid, shoot;
    float yaw, pitch;  // 单位：弧度
};

struct JudgerData {
    int game_time, self_hp, self_id;
};
```

---

## 调试与可视化

`armor_task/tasks/tools/draw.*` 提供：
- 绘制检测到的装甲板（包括 ID、置信度）
- 绘制 EKF 预测轨迹
- 显示 FPS、各模块耗时
- 3D 点投影到图像

关键数学工具（`math_tools.*`）：
- `limit_rad(angle)` — 角度归一化到 (-π, π]
- `xyz2ypd(pos)` — 笛卡尔转球面坐标
- `ypd2xyz(sph)` — 球面转笛卡尔坐标
- `xyz2ypd_jacobian()` — EKF 用雅可比矩阵

---

## 开发注意事项

1. **坐标系约定**：世界系 x 向前、y 向左、z 向上；云台 yaw 正方向为逆时针
2. **IMU 特殊处理**：哨兵 IMU 横置，`R_gimbal2imubody` 需修正 pitch/roll 轴
3. **大小装甲板**：`car_num == 1` 视为大装甲板（英雄机器人）
4. **子弹速度**：若从串口读取值 < 14 m/s 则默认使用 23 m/s
5. **EKF 发散保护**：当半径估计超出 [0.1, 0.7] m 范围时重置滤波器
6. **TensorRT 加速**：首次运行需用 trtexec 将 ONNX 转为 `.engine`（与目标硬件绑定）
