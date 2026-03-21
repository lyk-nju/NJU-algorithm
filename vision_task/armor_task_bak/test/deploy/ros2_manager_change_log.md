# ROS2Manager 变更日志

日期：2026-03-17  
文件：`io/ros2_manager.cpp`、`io/ros2_manager.hpp`

## 变更背景

在 `auto_aimer_test` 运行过程中，出现帧率波动明显（例如 60~100 FPS 之间抖动）。  
分析链路后发现，`ROS2Manager::image_callback` 每帧都执行 `frame.clone()`，会带来额外的 CPU 内存拷贝开销与带宽抖动。

## 具体改动

### 1. `FramePacket` 增加图像引用持有字段

- 文件：`io/ros2_manager.hpp`
- 修改：
  - 在 `FramePacket` 中新增：
    - `cv_bridge::CvImageConstPtr image_ref;`

目的：
- 显式持有 `cv_bridge` 返回对象，确保 `cv::Mat` 的底层数据生命周期安全。

---

### 2. 图像接收路径从深拷贝改为引用传递

- 文件：`io/ros2_manager.cpp`
- 位置：`ROS2Manager::image_callback`
- 旧逻辑：
  - `packet->frame = frame.clone();`
- 新逻辑：
  - `packet->image_ref = cv_ptr;`
  - `packet->frame = frame;`

目的：
- 去掉每帧深拷贝，降低内存带宽开销。
- 在高帧率场景下减少处理时延抖动，提升系统稳定性。

## 影响评估

- 功能行为：不改变上层接口语义（`get_frame_packet` 仍返回最新帧）。
- 性能影响：减少一次 `clone`，通常可降低 CPU 压力并减少 FPS 波动。
- 生命周期安全：通过 `image_ref` 持有消息引用，避免悬空内存风险。

## 验证结果

- 目标已成功编译：`auto_aimer_test`
- 编译命令：
  - `cmake --build build --target auto_aimer_test -j6`

## 后续建议

- 若仍存在波动，可继续从以下方向优化：
  - 对 `detector.detect()` 做限频/分帧策略（稳定优先）。
  - 相机侧设置固定 `acquisition_frame_rate`（如 90~100）换取更平滑输出。
