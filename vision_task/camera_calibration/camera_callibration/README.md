# 相机标定工具

用于ROS2相机内参标定的Python脚本。

## 功能特点

- 从ROS2的`/image_raw`话题读取实时图像
- 使用12×7棋盘格标定板进行标定
- 可视化界面显示角点检测结果
- 交互式图像采集：按C键采集，按Q键完成
- 自动计算相机内参矩阵和畸变系数
- 支持多种格式输出（NPZ、YAML）
- 显示去畸变效果对比

## 依赖项

```bash
# ROS2依赖
sudo apt install ros-${ROS_DISTRO}-cv-bridge

# Python依赖
pip install opencv-python numpy
```

## 使用方法

### 1. 准备标定板

使用12×7的棋盘格标定板，每个方格宽度为10mm。

**注意**: 12×7指的是内角点的数量，实际标定板应该是13×8的方格。

### 2. 启动相机节点

首先启动你的相机节点，确保发布图像到`/image_raw`话题：

```bash
# 示例：启动USB相机
ros2 run usb_cam usb_cam_node_exe
```

### 3. 运行标定脚本

**重要**: 必须使用系统 Python（不是 Anaconda Python），因为 ROS2 Humble 是为 Python 3.10 编译的。

```bash
# 推荐方法: 使用启动脚本（会自动使用正确的Python版本）
./run_calibration.sh

# 或者直接指定系统Python运行
/usr/bin/python3 camera_calibration_node.py --ros-args --params-file config.yaml
```

**注意**: 不要直接使用 `python3` 或 `python`，因为可能会调用 Anaconda 的 Python，导致版本不匹配错误。

### 4. 采集图像

- 将标定板放在相机前，确保能看到完整的棋盘格
- 当检测到棋盘格时，图像边框会变成**绿色**，并显示角点
- 按**C键**采集当前图像
- 从不同角度、不同位置、不同距离采集至少15-20张图片
- 采集完成后按**Q键**退出并开始标定

### 5. 标定建议

为了获得更好的标定效果，建议：

- 采集20-30张图片
- 覆盖图像的不同区域（中心、边缘、角落）
- 包含不同的倾斜角度
- 包含不同的距离（远、中、近）
- 确保标定板完全在图像内

## 输出文件

标定完成后，会在`calibration_images`目录下生成：

1. **采集的图像**: `calib_YYYYMMDD_HHMMSS.png`
2. **标定结果(NPZ)**: `camera_calibration.npz` - 包含完整的标定数据
3. **标定结果(YAML)**: `camera_calibration.yaml` - ROS兼容格式

### 标定结果说明

- **camera_matrix**: 相机内参矩阵 (3×3)
  - fx, fy: 焦距
  - cx, cy: 光心坐标
- **dist_coeffs**: 畸变系数
  - k1, k2, k3: 径向畸变
  - p1, p2: 切向畸变
- **mean_error**: 平均重投影误差（越小越好，通常小于1像素为优秀）

## 自定义参数

可以通过修改`config.yaml`文件来自定义参数：

```yaml
camera_calibration_node:
  ros__parameters:
    image_topic: '/camera/image_raw'  # 修改话题名称
    chessboard_width: 9               # 修改棋盘格列数
    chessboard_height: 6              # 修改棋盘格行数
    square_size: 25.0                 # 修改方格大小(mm)
    min_images: 20                    # 修改最少图片数量
```

## 键盘操作

- **C**: 采集当前图像（仅当检测到棋盘格时有效）
- **Q**: 完成采集并开始标定

## 常见问题

### 1. Python版本错误

如果看到 `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'` 错误：

- **原因**: 使用了错误的Python版本（如Anaconda的Python 3.12），而ROS2 Humble需要Python 3.10
- **解决**: 使用 `./run_calibration.sh` 脚本，或直接用 `/usr/bin/python3` 运行
- **详情**: 查看 `PYTHON_VERSION_ISSUE.md` 文件

### 2. 段错误(Segmentation fault)

如果程序崩溃并显示"段错误"或"核心已转储"：

- **原因**: OpenCV库版本冲突（系统库vs本地编译库）
- **解决**: 已在代码中修复，使用`passthrough`编码避免cv_bridge的颜色转换
- **验证**: 确保使用最新版本的脚本

### 3. 无法检测到棋盘格

- 确保棋盘格完全在图像内
- 确保光照充足且均匀
- 确保棋盘格是平整的
- 调整棋盘格与相机的距离
- 检查棋盘格尺寸配置是否正确（内角点数量）

### 4. 重投影误差较大

- 增加采集的图片数量（建议20-30张）
- 确保图片质量良好（不模糊）
- 确保覆盖了图像的不同区域
- 检查棋盘格的实际尺寸是否准确

### 5. 话题名称不匹配

修改`config.yaml`中的`image_topic`参数，或使用命令行参数：

```bash
/usr/bin/python3 camera_calibration_node.py --ros-args -p image_topic:=/your/topic/name
```

### 6. 无法创建OpenCV窗口

- 检查DISPLAY环境变量：`echo $DISPLAY`
- 确保在图形界面环境中运行，不是纯终端

