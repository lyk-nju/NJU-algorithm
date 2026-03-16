# 海康相机参数调试完整指南

## 📖 目录
1. [参数说明](#参数说明)
2. [三种调整方法](#三种调整方法)
3. [快速调试工具](#快速调试工具)
4. [参数优化指南](#参数优化指南)
5. [常见问题解决](#常见问题解决)

---

## 参数说明

### 📷 曝光时间 (Exposure Time)
- **参数名**: `exposure_time`
- **单位**: 微秒 (μs)
- **典型范围**: 100 - 50000 μs
- **作用**: 控制相机传感器的曝光时长

**效果**:
- ⬆️ **增大曝光时间** → 图像更亮，但动态物体可能模糊
- ⬇️ **减小曝光时间** → 图像更暗，但能捕捉快速运动

### 🔆 增益 (Gain)
- **参数名**: `gain`
- **单位**: 分贝 (dB)
- **典型范围**: 0 - 24 dB (取决于相机型号)
- **作用**: 放大传感器信号

**效果**:
- ⬆️ **增大增益** → 图像更亮，但噪点增加
- ⬇️ **减小增益** → 噪点减少，但图像更暗

### ⚖️ 白平衡 (Balance White Auto)
- **参数名**: `balance_white_auto`
- **值**: 0=关闭, 1=连续, 2=一次
- **作用**: 自动调整颜色平衡

---

## 三种调整方法

### 方法 1️⃣: 修改配置文件（推荐用于永久设置）

**步骤**:
1. 编辑配置文件:
   ```bash
   cd ~/NJU_RMVision/hiki_ros2
   nano config/camera_params.yaml
   ```

2. 修改参数:
   ```yaml
   /hik_camera:
     ros__parameters:
       camera_name: narrow_stereo
       exposure_time: 15000    # 调整曝光时间
       gain: 20.0              # 调整增益
       balance_white_auto: 0
   ```

3. 重新启动节点:
   ```bash
   ros2 launch hik_camera hik_camera.launch.py
   ```

**优点**: 永久保存，每次启动自动应用
**缺点**: 需要重启节点才能生效

---

### 方法 2️⃣: 启动时覆盖参数

**直接指定参数启动**:
```bash
ros2 run hik_camera hik_camera_node --ros-args \
  -p exposure_time:=15000 \
  -p gain:=20.0
```

**使用自定义配置文件**:
```bash
ros2 launch hik_camera hik_camera.launch.py \
  params_file:=/path/to/custom_params.yaml
```

**优点**: 快速测试不同配置
**缺点**: 仅在当前运行有效

---

### 方法 3️⃣: 运行时动态调整（推荐用于调试）

**命令行方式**:
```bash
# 查看当前值
ros2 param get /hik_camera exposure_time
ros2 param get /hik_camera gain

# 设置新值
ros2 param set /hik_camera exposure_time 15000
ros2 param set /hik_camera gain 20.0

# 列出所有参数
ros2 param list /hik_camera
```

**使用我们的调试脚本** (最简单):
```bash
./adjust_camera_params.sh
```

**优点**: 实时调整，立即生效，无需重启
**缺点**: 重启节点后会恢复默认值

---

## 快速调试工具

### 🚀 使用交互式调试脚本

```bash
cd ~/NJU_RMVision/hiki_ros2
./adjust_camera_params.sh
```

**功能**:
- ✅ 查看当前参数值
- ✅ 交互式调整曝光和增益
- ✅ 预设配置快速应用
- ✅ 查看参数有效范围
- ✅ 实时反馈调整结果

**预设配置**:
1. **暗环境配置**: 曝光 20000μs, 增益 18dB
2. **正常环境配置**: 曝光 8000μs, 增益 12dB
3. **高速抓拍配置**: 曝光 1000μs, 增益 10dB
4. **低噪声配置**: 曝光 5000μs, 增益 5dB

### 📊 实时查看图像效果

调整参数时，同时打开图像查看器：

```bash
# 方法1: 使用 rqt_image_view
ros2 run rqt_image_view rqt_image_view

# 方法2: 使用 rviz2
rviz2
```

在 rqt_image_view 中选择话题 `/hik_camera/image_raw` 查看效果。

---

## 参数优化指南

### 🎯 优化目标

根据您的使用场景选择优化目标：

| 场景             | 曝光时间      | 增益    | 说明                   |
| ---------------- | ------------- | ------- | ---------------------- |
| **暗环境标定**   | 15000-30000μs | 16-20dB | 需要足够亮度看清标定板 |
| **正常室内**     | 5000-10000μs  | 10-15dB | 平衡亮度和噪点         |
| **运动物体追踪** | 500-2000μs    | 8-12dB  | 减少运动模糊           |
| **高速抓拍**     | 100-1000μs    | 5-10dB  | 捕捉快速运动           |
| **低噪声成像**   | 3000-8000μs   | 0-8dB   | 需要良好光照           |

### 📝 调试步骤

#### Step 1: 确定基准

```bash
# 使用默认值启动
ros2 launch hik_camera hik_camera.launch.py

# 在另一个终端查看图像
ros2 run rqt_image_view rqt_image_view
```

#### Step 2: 调整曝光时间

```bash
# 如果图像太暗，增大曝光时间
ros2 param set /hik_camera exposure_time 15000

# 如果有运动模糊，减小曝光时间
ros2 param set /hik_camera exposure_time 2000
```

**经验法则**:
- 每次调整幅度: ±5000μs
- 观察图像亮度变化
- 注意运动模糊程度

#### Step 3: 微调增益

```bash
# 如果曝光已最大但仍太暗，增加增益
ros2 param set /hik_camera gain 18.0

# 如果噪点过多，减小增益
ros2 param set /hik_camera gain 8.0
```

**经验法则**:
- 每次调整幅度: ±2dB
- 优先调整曝光，增益作为辅助
- 增益越小噪点越少

#### Step 4: 平衡优化

**原则**: 
1. ⏱️ **优先调整曝光时间** 来控制亮度
2. 🔆 **增益作为补充** 当曝光时间受限时使用
3. 🎯 **保持增益尽可能低** 以减少噪点

**示例优化流程**:
```bash
# 1. 先用较低增益
ros2 param set /hik_camera gain 8.0

# 2. 调整曝光到合适亮度
ros2 param set /hik_camera exposure_time 12000

# 3. 如果还不够亮，稍微提高增益
ros2 param set /hik_camera gain 12.0

# 4. 微调到最佳效果
ros2 param set /hik_camera exposure_time 10000
```

#### Step 5: 保存配置

满意后，将参数写入配置文件：

```bash
nano config/camera_params.yaml
```

```yaml
/hik_camera:
  ros__parameters:
    exposure_time: 12000  # 您调试的值
    gain: 12.0            # 您调试的值
```

---

## 常见问题解决

### ❓ 图像太暗

**解决方案**:
1. 先增大曝光时间:
   ```bash
   ros2 param set /hik_camera exposure_time 20000
   ```

2. 如果还不够亮，增加增益:
   ```bash
   ros2 param set /hik_camera gain 18.0
   ```

3. 检查环境光照是否充足

### ❓ 图像噪点太多

**解决方案**:
1. 降低增益:
   ```bash
   ros2 param set /hik_camera gain 8.0
   ```

2. 适当增加曝光补偿亮度:
   ```bash
   ros2 param set /hik_camera exposure_time 15000
   ```

3. 改善环境照明

### ❓ 运动物体模糊

**解决方案**:
1. 减小曝光时间:
   ```bash
   ros2 param set /hik_camera exposure_time 1000
   ```

2. 增加增益补偿亮度:
   ```bash
   ros2 param set /hik_camera gain 12.0
   ```

3. 增强场景照明

### ❓ 参数设置不生效

**检查步骤**:
1. 确认节点正在运行:
   ```bash
   ros2 node list | grep hik_camera
   ```

2. 检查参数是否设置成功:
   ```bash
   ros2 param get /hik_camera exposure_time
   ```

3. 查看节点日志是否有错误:
   ```bash
   ros2 node info /hik_camera
   ```

### ❓ 不知道有效范围

**查询方法**:
```bash
# 查看参数描述和范围
ros2 param describe /hik_camera exposure_time
ros2 param describe /hik_camera gain

# 或使用调试脚本的选项5
./adjust_camera_params.sh
# 选择: 5) 查看参数范围
```

节点启动时也会在日志中打印范围:
```
[INFO] [hik_camera_node]: Exposure time: 5000 (range: 100.0 to 50000.0)
[INFO] [hik_camera_node]: Gain: 10.5 (range: 0.0 to 24.0)
```

---

## 🎓 进阶技巧

### 自动曝光模式

海康相机支持自动曝光，但当前节点使用手动模式。如需自动曝光，可以修改源码。

### 创建场景配置文件

为不同场景创建多个配置文件：

```bash
# 暗环境配置
config/camera_params_dark.yaml

# 正常光照配置
config/camera_params_normal.yaml

# 高速抓拍配置
config/camera_params_highspeed.yaml
```

启动时指定:
```bash
ros2 launch hik_camera hik_camera.launch.py \
  params_file:=config/camera_params_dark.yaml
```

### 使用 rqt_reconfigure

ROS2 也支持动态参数调整GUI (如果节点实现了相应接口):
```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

---

## 📞 获取帮助

如果遇到问题:
1. 查看节点日志: `ros2 topic echo /rosout`
2. 检查参数文档: `README.md`
3. 使用调试脚本: `./adjust_camera_params.sh`
4. 查看相机手册了解具体参数范围

---

**提示**: 建议在调试时使用方法3️⃣（动态调整），找到最佳参数后再写入配置文件（方法1️⃣）永久保存。