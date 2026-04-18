#pragma once

// Driver 层聚合头：上层（tasks/）想拿到任何具体驱动时，只需要 include 这一个文件。
// 每一路 driver 都位于 io/driver/<name>/ 子目录，和结构体/接口/算法解耦。
//
// 依赖方向：driver → interface + algorithm + structs （永远单向）。
//
// 现有 driver：
//   - cboard/ : Cboard         实现 io::ICboard
//   - camera/ : Camera / Hik   物理相机 SDK 封装；
//               DirectImageSource 作为它对外的 io::IFrameSource 适配器
//   - ros2/   : ROS2Manager    ROS2 节点 + 图像/cmd_vel 订阅 + autoaim 发布；
//               Ros2ImageSource 作为它对外的 io::IFrameSource 适配器
//
// 未来若接入 MultiCameraRig（组合多个 ICamera）等组合式 driver，在 camera/ 下追加即可。

#include "camera/camera_base.hpp"
#include "camera/direct_image_source.hpp"
#include "cboard/cboard.hpp"
#include "ros2/ros2_image_source.hpp"
#include "ros2/ros2_manager.hpp"
