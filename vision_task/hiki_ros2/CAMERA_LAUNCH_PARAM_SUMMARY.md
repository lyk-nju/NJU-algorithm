# 相机 Launch 时参数设置与 hikSDK 对应关系

## 1. 参数入口
- Launch 文件通过 `params_file` 加载参数文件，并注入少量运行参数：`launch/hik_camera.launch.py:23-49`
- 相机参数主配置来自：`config/camera_params.yaml:1-12`

## 2. Launch 启动阶段实际设置/修改的相机参数

| 相机参数 | 来源（launch/yaml/默认） | 节点中调用位置 | 调用的 hikSDK API | hikSDK 声明位置 |
|---|---|---|---|---|
| `TriggerMode`（固定设为关闭触发=0） | 代码固定值（非 yaml） | `src/hik_camera_node.cpp:50` | `MV_CC_SetTriggerMode(..., 0)` | `hikSDK/include/MvCameraControl.h:2554` |
| `ADCBitDepth` / `ADCBitsDepth`（尝试设为 8bit） | 代码固定策略（非 yaml） | `src/hik_camera_node.cpp:268-290` | `MV_CC_GetEnumValue` + `MV_CC_SetEnumValueByString` | `744`、`782` |
| `PixelFormat`（优先 BGR8，否则 RGB8） | 代码自动探测（非 yaml） | `src/hik_camera_node.cpp:61-79` | `MV_CC_GetPixelFormat` + `MV_CC_SetPixelFormat` | `2401`、`2418` |
| `sdk_image_node_num` | `config/camera_params.yaml:9`（默认 3） | `src/hik_camera_node.cpp:243-250` | `MV_CC_SetImageNodeNum` | `599` |
| `acquisition_frame_rate`（>0 才启用） | `config/camera_params.yaml:10`（默认 0.0） | `src/hik_camera_node.cpp:252-265` | `MV_CC_SetBoolValue("AcquisitionFrameRateEnable", true)` + `MV_CC_SetFloatValue("AcquisitionFrameRate", ...)` | `858`、`820` |
| `binning_horizontal` | `config/camera_params.yaml:11`（默认 2） | `src/hik_camera_node.cpp:230,233,296-327` | `MV_CC_GetIntValue` + `MV_CC_SetIntValue` | `665`、`706` |
| `binning_vertical` | `config/camera_params.yaml:12`（默认 2） | `src/hik_camera_node.cpp:231,234,296-327` | `MV_CC_GetIntValue` + `MV_CC_SetIntValue` | `665`、`706` |
| `exposure_auto` | `config/camera_params.yaml:6`（默认 0） | `src/hik_camera_node.cpp:341-347` | `MV_CC_SetExposureAutoMode` | `2520` |
| `exposure_time` | `config/camera_params.yaml:5`（默认 5000） | `src/hik_camera_node.cpp:351-356` | `MV_CC_GetFloatValue("ExposureTime")` + `MV_CC_SetFloatValue("ExposureTime", ...)` | `801`、`820` |
| `gain` | `config/camera_params.yaml:7`（默认当前值） | `src/hik_camera_node.cpp:360-365` | `MV_CC_GetFloatValue("Gain")` + `MV_CC_SetFloatValue("Gain", ...)` | `801`、`820` |
| `balance_white_auto` | `config/camera_params.yaml:8`（默认 0） | `src/hik_camera_node.cpp:372-374` | `MV_CC_SetBalanceWhiteAuto` | `2841` |

> 说明：`camera_name`、`camera_info_url`、`use_sensor_data_qos`、`publish_camera_info`、`enable_timing_log` 是 ROS 发布/标定相关参数，不直接写入相机硬件寄存器。

## 3. Launch 后可运行时动态修改的相机参数

参数回调：`src/hik_camera_node.cpp:377-424`

| 可动态修改参数 | 回调中的 SDK 调用 | 节点位置 | hikSDK 声明位置 |
|---|---|---|---|
| `exposure_time` | `MV_CC_SetFloatValue("ExposureTime", ...)` | `384` | `820` |
| `exposure_auto` | `MV_CC_SetExposureAutoMode(...)` | `390` | `2520` |
| `acquisition_frame_rate` | `MV_CC_SetBoolValue("AcquisitionFrameRateEnable", ...)` + `MV_CC_SetFloatValue("AcquisitionFrameRate", ...)` | `397`, `404` | `858`、`820` |
| `gain` | `MV_CC_SetFloatValue("Gain", ...)` | `411` | `820` |
| `balance_white_auto` | `MV_CC_SetBalanceWhiteAuto(...)` | `417` | `2841` |

补充：`adjust_camera_params.sh` 通过 `ros2 param set /hik_camera ...` 实际调用上述动态回调，当前脚本主要修改 `exposure_time` 与 `gain`。

## 4. 结论
- Launch 启动时，相机硬件侧至少会被设置的关键项包括：触发模式、ADC 位深、像素格式、SDK 图像节点数、帧率限制（可选）、binning、曝光自动/曝光时间、增益、白平衡自动模式。
- 这些设置均可在 `src/hik_camera_node.cpp` 中定位到具体调用点，并在 `hikSDK/include/MvCameraControl.h` 中定位到对应 API 声明。
