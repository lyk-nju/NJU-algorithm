# 视觉自瞄系统整改 TODO 清单

> 目标：消除已确认的性能瓶颈、清理冗余代码、统一架构组织。  
> 优先级：🔴 高（直接影响帧率）→ 🟡 中（影响可维护性）→ 🟢 低（代码风格）  
> 每条任务均标注了**文件路径和行号**，方便直接定位。

---

## 一、性能优化

### P-01 🔴【构建类型强制 Debug，全程零优化】
**文件**：`armor_task/CMakeLists.txt`，第 284 行  
**问题**：`set(CMAKE_BUILD_TYPE Debug)` 被**硬编码**在 CMakeLists 最末尾，导致所有目标均以 `-O0 -g` 编译，这是当前 100fps 上限的最主要原因之一。  
**修改方法**：
1. 删除第 284 行的 `set(CMAKE_BUILD_TYPE Debug)`
2. 在该行位置替换为以下内容，实现默认 Release、可在命令行覆盖：
```cmake
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
```
3. 将第 287-295 行的所有 `if(CMAKE_BUILD_TYPE STREQUAL "Debug")` 条件块保留，它们会在用户传 `-DCMAKE_BUILD_TYPE=Debug` 时自动生效。
4. 在 `Release` 模式下补充优化选项（在文件末尾添加）：
```cmake
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -march=native)
    add_compile_definitions(NDEBUG)
endif()
```
5. 重新编译验证：`cmake -DCMAKE_BUILD_TYPE=Release ..` + `make -j$(nproc)`，运行 `video_test` 观察 FPS 变化。

---

### P-02 🔴【detector 热路径大量 std::cout 输出】
**文件**：`armor_task/tasks/detector.cpp`，第 538-547 行  
**问题**：每帧推理完成后，对每个检测到的装甲板都调用 `std::cout`，包含字符串拼接和 I/O 刷新。在 100fps 下每秒输出数百行，严重拖慢主线程。  
**修改方法**：
1. 将第 538-547 行整块 `if (!detected_armors.empty())` 打印代码删除（或用 `#ifdef DEBUG_DETECTOR` 包裹）：
```cpp
// 删除这段代码：
if (!detected_armors.empty()) {
    std::cout << "[Detector] Detected " << ...
    for (const auto& armor : detected_armors) {
        std::cout << "  - Detect ID: " ...
    }
}
```
2. 如果需要调试输出，改为条件编译：在文件顶部加 `// #define DEBUG_DETECTOR`，用 `#ifdef DEBUG_DETECTOR ... #endif` 包裹打印代码。

---

### P-03 🔴【CUDA 流水线中有两次阻塞同步（cudaStreamSynchronize）】
**文件**：`armor_task/cuda/yolo_decode.cu`，第 169 行 和 第 183 行  
**问题**：`launch_decode_kernel` 函数在 decode_kernel 之后立即做了一次 `cudaStreamSynchronize`（第169行）来读取 `count`，这会强制 GPU 停下等待 CPU 取值，打断 GPU 流水线。每帧多一次停顿约 0.3-1ms。  
**修改方法**：
1. 利用"上一帧 count 作为本帧上界"的方式，消除第一次 sync：
```cpp
// 在函数外（或 Detector 成员中）维护一个 last_count
static int last_count = kMaxBoxes;  // 首帧保守估计

// 直接用 last_count 启动 NMS，不需要先 sync 读 count
const int nms_blocks = (last_count + threads - 1) / threads;
nms_kernel<<<nms_blocks, threads, 0, stream>>>(d_boxes, last_count, nms_thresh, d_keep_flags);
compact_kernel<<<...>>>(d_boxes, d_keep_flags, last_count, ...);

// 只在最后做一次 sync，读取本帧实际 kept_count，更新 last_count
cudaMemcpyAsync(&kept_count, d_kept_count, sizeof(int), cudaMemcpyDeviceToHost, stream);
cudaMemcpyAsync(&last_count, d_count, sizeof(int), cudaMemcpyDeviceToHost, stream);
cudaStreamSynchronize(stream);
last_count = std::min(last_count, kMaxBoxes);
```
2. 删除第 167-169 行的第一次 `cudaMemcpyAsync count` + `cudaStreamSynchronize`。
3. 验证：添加断言 `assert(kept_count <= kMaxBoxes)` 确保无越界。

---

### P-04 🔴【target.cpp 每帧动态构造 11×11 Eigen 矩阵】
**文件**：`armor_task/tasks/target.cpp`，第 48-106 行（`Target::predict(double dt)` 函数）  
**问题**：每次调用 `predict(dt)` 都在**栈/堆**上新建 `Eigen::MatrixXd F(11,11)` 和 `Eigen::MatrixXd Q(11,11)` 两个动态大小矩阵并赋值。100fps 下每秒构造/析构 200 次矩阵，产生不必要的内存分配和 cache 污染。  
**修改方法**：
1. 在 `Target` 类的私有成员区（`target.hpp` 第 45 行附近）添加缓存矩阵：
```cpp
// target.hpp 私有成员区新增：
Eigen::Matrix<double, 11, 11> F_;  // 用固定大小类型，栈分配
Eigen::Matrix<double, 11, 11> Q_;
```
2. 在 `Target::predict(double dt)` 函数中，将每帧都构造的 `Eigen::MatrixXd F` 和 `Q` 改为修改 `F_` 和 `Q_` 的对应元素（只更新含 `dt` 的位置，其余位置在构造函数中初始化一次）：
```cpp
// 构造函数中初始化 F_ 为单位阵，Q_ 为零
// predict 中只更新变化的元素：
F_(0,1) = dt; F_(2,3) = dt; F_(4,5) = dt; F_(6,7) = dt;
// Q 同理，只更新含 dt 的项
```
3. 将函数调用改为 `ekf_.predict(F_, Q_, f);`

---

### P-05 🔴【aimer.cpp 每帧创建 10 个 Target 对象副本】
**文件**：`armor_task/tasks/aimer.cpp`，第 72 行  
**问题**：`std::vector<Target> iteration_target(10, target)` 一次性复制 10 个 Target 对象（每个含 Eigen 矩阵），但实际迭代通常 3-5 次就收敛。  
**修改方法**：
1. 删除第 72 行 `std::vector<Target> iteration_target(10, target);`
2. 在循环外声明一个副本：`Target iter_target = target;`
3. 将第 85 行的 `iteration_target[iter].predict(predict_time)` 改为：
```cpp
iter_target = target;          // 每次迭代重置为初始状态
iter_target.predict(predict_time);
auto aim_point = choose_aim_point(iter_target);
```
4. 相关 `aim_point` 变量引用从 `iteration_target[iter]` 改为 `iter_target`。

---

### P-06 🟡【tracker 的 update_target 对所有候选装甲板调用 PnP】
**文件**：`armor_task/tasks/tracker.cpp`，第 278-320 行（`Tracker::update_target`）  
**问题**：对所有 `car_num` 匹配的装甲板都调用 `solver_._solve_pnp(armor)`（第 290 行）。PnP 包含矩阵求解和坐标变换，是重型操作。若视野中同时有 3 块同编号装甲板，每帧多算 2 次 PnP。

**⚠️ 重要约束**：`update_target` 的打分逻辑（第 295-308 行）依赖 PnP 得到的 `p_world` 和 `ypd_in_world`，与 EKF 预测的 4 个装甲板位置做 **3D 空间匹配**，用于在多块可见装甲板中找出正确的那块。`target_.update()` 随后再用这块装甲板确定它对应哪个槽位（id=0/1/2/3）。因此**不能**用 2D 图像距离替代整个 3D 打分流程——在小陀螺场景下（同时可见 2 块装甲板），2D 最近不等于 3D 最优，选错装甲板会导致 EKF 槽位匹配错误，产生跳变。

**正确的优化方向——2D 阈值剔除（非单选）**：
1. 在 `Tracker` 中添加一个从 EKF 状态反投影到图像坐标的近似预测函数（或复用已有的 `pnp_solver_.fast_project_armor()`）：
```cpp
// 利用 EKF 预测的3D位置反投影，得到各槽位的近似2D图像坐标
cv::Point2f predict_2d_center = project_ekf_center_to_image(target_, solver_);
```
2. 在第 288 行（`car_num` 过滤）之后、第 290 行（PnP）之前，增加 2D 距离**阈值过滤**，距离超过阈值的直接跳过：
```cpp
if (armor.car_num != target_.car_num) continue;

// 新增：2D 粗筛，拒绝明显偏离的候选（不是单选，所有通过阈值的都保留）
constexpr float kMax2dDistPx = 300.0f;  // 根据实际分辨率调整
if (cv::norm(armor.center - predict_2d_center) > kMax2dDistPx) continue;

if (!solver_._solve_pnp(armor)) continue;
```
3. 这样保留了所有合理候选（可能有 1-2 块），仍走完整的 3D 打分流程，但拒绝了因检测误报或同场景其他机器人装甲板导致的无效 PnP 调用。
4. 验证：打印每帧进入 PnP 的候选数量，确认从最坏情况 N 降到 1-2。

---

### P-07 🟡【CUDA decode 后 CPU 再次排序（重复工作）】
**文件**：`armor_task/tasks/detector.cpp`，第 467-474 行  
**问题**：GPU NMS 已经完成了置信度排序和去重，但 CPU 在取回结果后又执行了 `nth_element`（第 468-472 行）和 `sort`（第 474 行），对 `decoded_boxes` 再次按置信度排序，属于重复工作。  
**修改方法**：
1. 删除第 467-474 行的以下代码：
```cpp
// 删除这段：
constexpr size_t kTopKDetections = 128;
if (decoded_boxes.size() > kTopKDetections) {
    std::nth_element(...);
    decoded_boxes.resize(kTopKDetections);
}
std::sort(decoded_boxes.begin(), decoded_boxes.end(), [...]);
```
2. GPU NMS 已保证输出为不重叠的框，直接进入第 478 行开始的封装逻辑即可。

---

### P-08 🟡【detect() 函数丢弃 preprocess() 返回值】
**文件**：`armor_task/tasks/detector.cpp`，第 138-145 行  
**问题**：
```cpp
ArmorArray Detector::detect(const cv::Mat &frame) {
    cv::Mat input_blob = preprocess(frame, ...);  // 计算了 letterbox 参数
    return search_armors(frame, cv::Mat());        // 传入空 Mat！
}
```
`preprocess()` 对 TRT 路径有效（CUDA kernel 直接写入 `device_input_buffer_`，side-effect），但传给 `search_armors` 的是 `cv::Mat()`。对于 OpenCV-DNN 路径，`input_blob` 根本没被用到，意味着 DNN 路径从未能正常工作。代码意图不清晰。  
**修改方法**：
1. 将第 144 行改为正确传入 blob：
```cpp
return search_armors(frame, input_blob);
```
2. 在 `search_armors` 的 OpenCV-DNN 分支（第 451-460 行）使用传入的 `input_blob`：
```cpp
yolo_net_.setInput(input_blob);  // 使用传入的 blob 而非重新生成
```
3. 删除第 450 行注释 `// 保持原有的 OpenCV 推理逻辑` 前面的冗余逻辑。

---

### P-09 🟡【CUDA 静态局部指针无法释放，存在资源泄漏】
**文件**：`armor_task/cuda/yolo_decode.cu`，第 141-156 行  
**问题**：`launch_decode_kernel` 内使用 `static` 局部变量保存 CUDA 设备指针，程序退出时无法释放（没有对应的 `cudaFree`）。虽然 OS 会回收，但在长时间运行中无法手动控制生命周期。  
**修改方法**：
1. 将这些 `static` 变量（`d_boxes`, `d_kept_boxes`, `d_count`, `d_kept_count`, `d_keep_flags`, `h_boxes`）移动到 `Detector` 类的私有成员（`detector.hpp` 第 100 行附近新增）：
```cpp
// decoder.hpp 新增成员：
DecodedBBox* d_decode_boxes_ = nullptr;
DecodedBBox* d_kept_boxes_ = nullptr;
int* d_decode_count_ = nullptr;
int* d_kept_count_ = nullptr;
uint8_t* d_keep_flags_ = nullptr;
DecodedBBox* h_kept_boxes_ = nullptr;
bool decode_buffers_init_ = false;
```
2. 在 `Detector` 构造函数中初始化这些缓冲区（在现有 TRT 初始化代码块末尾添加）。
3. 在 `Detector::~Detector()` 析构函数（第 124-136 行）中添加对应的 `cudaFree` 和 `cudaFreeHost`。
4. 修改 `launch_decode_kernel` 函数签名，接收这些指针作为参数，去掉 `static` 声明。

---

## 二、代码风格

### S-01 🔴【CMakeLists 中大量重复的 target_link_libraries】
**文件**：`armor_task/CMakeLists.txt`，第 154-254 行  
**问题**：7 个可执行目标各自重复写了相同的链接库列表（`OpenCV_LIBS`, `EIGEN3_LIBRARIES`, `yaml-cpp`, `spdlog`, `pthread`, `nvinfer`, `cudart`），共约 100 行重复代码。  
**修改方法**：
1. 在第 148 行（第一个 `target_link_libraries` 之前）定义公共链接库变量：
```cmake
set(COMMON_LIBS
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
    spdlog::spdlog
    pthread
    nvinfer
    cudart
    -L/usr/local/cuda/lib64
)
set(ROS2_LIBS
    rclcpp::rclcpp
    cv_bridge::cv_bridge
    visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
)
```
2. 将每个 `target_link_libraries` 中的重复部分替换为 `${COMMON_LIBS}` 和 `${ROS2_LIBS}`，例如：
```cmake
target_link_libraries(video_test ${COMMON_LIBS} rclcpp::rclcpp
    visualization_msgs::visualization_msgs__rosidl_typesupport_cpp)
target_link_libraries(camera_sub_test ${COMMON_LIBS} ${ROS2_LIBS})
target_link_libraries(detector_test ${COMMON_LIBS} ${ROS2_LIBS})
# ...以此类推
```
3. 同理，`target_compile_options` 和 `set_target_properties` 的重复部分也可以用循环简化：
```cmake
set(ALL_TARGETS video_test camera_sub_test detector_test pnp_solver_test
                target_test auto_aimer_test communication_test fake_publish)
foreach(T ${ALL_TARGETS})
    target_compile_options(${T} PRIVATE -Wall -Wextra)
endforeach()
```

---

### S-02 🔴【删除 detector.cpp 中数百行注释掉的旧代码】
**文件**：`armor_task/tasks/detector.cpp`  
**问题**：文件中存在三大段被注释掉的旧实现：
- 第 147-186 行：旧的 CPU `preprocess()` 函数（`// cv::Mat Detector::preprocess...`）
- 第 238-428 行：旧的 CPU `search_armors()` 函数（`// ArmorArray Detector::search_armors...`）  
- 第 551-644 行：另一版本的旧 `search_armors()` 实现

这些代码已被新实现替代，靠 git 历史保存，无需留在源文件中。  
**修改方法**：
1. 删除第 147-186 行的注释掉的旧 `preprocess()` 函数。
2. 删除第 238-428 行的注释掉的旧 `search_armors()` 函数（含约 190 行）。
3. 删除第 551-644 行的另一版本旧代码。
4. 操作前执行 `git diff` 确认当前无未提交修改，以便回滚。

---

### S-03 🟡【删除 tracker.cpp 中的注释旧代码和大量注释调试输出】
**文件**：`armor_task/tasks/tracker.cpp`  
**问题**：
- 第 51-94 行：大量被注释的排序逻辑、过滤逻辑。
- 第 113-146 行：完整注释掉的状态机调试输出和发散检测代码。
- 第 322-363 行：旧版 `update_target` 的注释实现。
- 第 52-70 行间散落多个 `//std::cout` 单行注释调试打印。

**修改方法**：
1. 删除第 322-363 行的旧版注释 `update_target` 函数。
2. 删除第 113-146 行的注释调试输出块（保留活跃逻辑不变）。
3. 删除第 51 行的 `// if (state_ != "lost" && dt > 0.1)` 注释块。
4. 删除所有形如 `// std::cout` 的单行注释（用 IDE 正则替换 `^\s*//.*std::cout.*$`）。

---

### S-04 🟡【删除 pnp_solver.cpp 中注释掉的旧 set_R_gimbal2world 实现】
**文件**：`armor_task/tasks/pnp_solver.cpp`，第 87-103 行  
**问题**：旧版 `set_R_gimbal2world` 实现被整体注释，共 17 行，已无用。  
**修改方法**：删除第 87-103 行的完整注释块（从 `// void PnpSolver::set_R_gimbal2world` 到 `// }`）。

---

### S-05 🟡【用 enum class 替换 Tracker 的字符串状态机】
**文件**：`armor_task/tasks/tracker.hpp` 和 `tracker.cpp`  
**问题**：状态机用 `std::string` 表示状态（"lost", "tracking" 等），每次状态判断都是字符串比较（如 `if (state_ == "lost")`），有额外开销且易出现拼写错误。  
**修改方法**：
1. 在 `tracker.hpp` 中（`Tracker` 类定义之前）添加枚举：
```cpp
enum class TrackerState { LOST, DETECTING, TRACKING, TEMP_LOST, SWITCHING };
```
2. 将 `tracker.hpp` 中的成员 `std::string state_` 和 `std::string pre_state_` 改为 `TrackerState state_` 和 `TrackerState pre_state_`。
3. 在 `tracker.hpp` 添加辅助函数：
```cpp
static const char* state_name(TrackerState s) {
    switch(s) {
        case TrackerState::LOST:      return "lost";
        case TrackerState::DETECTING: return "detecting";
        case TrackerState::TRACKING:  return "tracking";
        case TrackerState::TEMP_LOST: return "temp_lost";
        case TrackerState::SWITCHING: return "switching";
    }
    return "unknown";
}
```
4. 在 `tracker.cpp` 中将所有 `state_ == "lost"` 形式替换为 `state_ == TrackerState::LOST`，以此类推（共约 15 处）。
5. 将 `Tracker::state()` 的返回类型改为 `const char*`，用 `state_name(state_)` 返回；或新增 `TrackerState state_enum() const` 接口。
6. 修改 `tracker.hpp` 中 `std::string state() const;` 的声明。

---

### S-06 🟡【armor.hpp 中同时有 #ifndef 和 #pragma once（冗余）】
**文件**：`armor_task/include/armor.hpp`，第 1-4 行  
**问题**：文件开头同时使用了 `#ifndef ARMOR_HPP` + `#define ARMOR_HPP` 和 `#pragma once` 两种 include guard，重复。  
**修改方法**：删除第 1 行 `#ifndef ARMOR_HPP`、第 2 行 `#define ARMOR_HPP`、最后的 `#endif`，只保留 `#pragma once`。

---

### S-07 🟡【PnpSolver::_solve_pnp 使用保留命名规范（下划线开头）】
**文件**：`armor_task/tasks/pnp_solver.hpp` 第 22 行，以及所有调用处  
**问题**：C++ 中单下划线开头（`_solve_pnp`）在全局和命名空间作用域是保留名，虽然在类内使用风险较低，但不符合规范且名字含义模糊。  
**修改方法**：
1. 将 `pnp_solver.hpp` 第 22 行的 `bool _solve_pnp(Armor &armor);` 改为 `bool solvePnP(Armor &armor);`
2. 在 `pnp_solver.cpp` 中同步修改函数定义名。
3. 用全局搜索（`grep -rn "_solve_pnp"` 在 `armor_task/` 目录下）找到所有调用处，统一改为 `solvePnP`（主要在 `tracker.cpp` 第 245、290 行）。

---

---

### S-09 🟡【aimer.cpp 使用魔数 57.3 作为角度转换】
**文件**：`armor_task/tasks/aimer.cpp`，第 17-20 行  
**问题**：`yaml["yaw_offset"].as<double>() / 57.3` 用 57.3 近似 180/π，精度不足（实际应为 57.29577...）且含义不直观。  
**修改方法**：
1. 在 `aimer.cpp` 文件顶部（`#include` 区域之后）添加：
```cpp
static constexpr double kDeg2Rad = M_PI / 180.0;
```
2. 将第 17-20 行的所有 `/ 57.3` 替换为 `* kDeg2Rad`：
```cpp
yaw_offset_    = yaml["yaw_offset"].as<double>()    * kDeg2Rad;
pitch_offset_  = yaml["pitch_offset"].as<double>()  * kDeg2Rad;
comming_angle_ = yaml["comming_angle"].as<double>()  * kDeg2Rad;
leaving_angle_ = yaml["leaving_angle"].as<double>()  * kDeg2Rad;
```
3. 同样检查 `aimer.cpp` 第 148 行 `60 / 57.3`，替换为 `60.0 * kDeg2Rad`。

---

### S-10 🟡【Target::convergened() 存在拼写错误】
**文件**：`armor_task/tasks/target.hpp`，第 38 行；`target.cpp` 对应定义处  
**问题**：函数名 `convergened()` 是 `converged()` 的拼写错误（多了一个 `ne`）。  
**修改方法**：
1. 在 `target.hpp` 第 38 行将 `bool convergened();` 改为 `bool converged();`
2. 在 `target.cpp` 中找到对应的函数定义（通过 `grep -n "convergened" target.cpp`），同步修改为 `converged`。
3. 全局搜索所有调用处（`grep -rn "convergened"` 在整个 `armor_task/` 目录），统一修改。

---

### S-11 🟡【tools/pharser.cpp 存在拼写错误，应为 parser】
**文件**：`armor_task/tools/pharser.cpp`，`armor_task/tools/pharser.hpp`  
**问题**：`pharser` 是 `parser` 的拼写错误，文件名和内容均需修正。  
**修改方法**：
1. 将文件 `pharser.cpp` 和 `pharser.hpp` 重命名为 `parser.cpp` 和 `parser.hpp`：
   ```bash
   cd armor_task/tools
   git mv pharser.cpp parser.cpp
   git mv pharser.hpp parser.hpp
   ```
2. 修改 `parser.hpp` 中的 include guard（如有）：将 `PHARSER_HPP` 改为 `PARSER_HPP`。
3. 修改所有 `#include "pharser.hpp"` 为 `#include "parser.hpp"`（搜索：`grep -rn "pharser" armor_task/`）。
4. 修改 `CMakeLists.txt` 第 103 行 `tools/pharser.cpp` 为 `tools/parser.cpp`，以及第 124 行同样修改。

---

### S-12 🟢【Ekf 类公有数据成员应封装】
**文件**：`armor_task/tasks/ekf.hpp`，第 13-14 行  
**问题**：`Ekf` 类的 `x`（状态向量）和 `P`（协方差矩阵）是 `public` 成员，任何代码都可以直接修改 EKF 内部状态，破坏封装性。  
**修改方法**：
1. 将 `x` 和 `P` 改为 `private`（放到现有 `private` 区域）。
2. 添加 `const` 访问器：
```cpp
const Eigen::VectorXd& state() const { return x; }
const Eigen::MatrixXd& covariance() const { return P; }
```
3. 搜索所有直接访问 `ekf_.x` 和 `ekf_.P` 的地方（主要在 `target.cpp`），改为通过 `ekf_.state()` 访问。
4. 注意 `target.cpp` 第 206 行 `ekf_.x` 直接访问，需改为 `ekf_.state()`。

---

### S-13 🟢【PnpSolver 的 R_gimbal2world_ 不应为 public】
**文件**：`armor_task/tasks/pnp_solver.hpp`，第 41 行  
**问题**：`R_gimbal2world_` 是 `public` 成员，在 `video_test.cpp` 第 235 行被直接引用（`pnp_solver.R_gimbal2world_`）。公共成员变量破坏了封装。  
**修改方法**：
1. 将 `pnp_solver.hpp` 第 41 行的 `R_gimbal2world_` 改为 `private`。
2. 添加 `const` 访问器：
```cpp
const Eigen::Matrix3d& gimbal2world() const { return R_gimbal2world_; }
```
3. 修改 `video_test.cpp` 第 235 行：将 `pnp_solver.R_gimbal2world_` 改为 `pnp_solver.gimbal2world()`。
4. 同样检查 `draw.cpp`、`draw.hpp` 等是否有对 `R_gimbal2world_` 的直接访问，一并修改。

---

### S-14 🟢【删除 src/main.cpp（无法编译的死代码）】
**文件**：`armor_task/src/main.cpp`  
**问题**：`main.cpp` 引用了已不存在的旧 API：`auto_aim::Armor`、`Tracker::evaluate()`、`Tracker::update_tracker()`、`Tracker::get_current_target()`、`tools::plotSingleArmor()` 等。该文件无法编译，是完全的死代码，而且与当前 `test/` 目录下的测试文件功能重叠。  
**修改方法**：
1. 确认 `CMakeLists.txt` 中没有将 `src/main.cpp` 加入任何编译目标（第 91-93 行已注释掉 `tasks/main.cpp`，确认 `src/main.cpp` 也未被使用）。
2. 执行删除：`git rm armor_task/src/main.cpp`
3. 如果 `src/` 目录因此变空，也一并删除：`git rm -r armor_task/src/`

---

## 三、代码架构组织

### A-01 🔴【YAML 配置被多次重复解析】
**文件**：`tracker.cpp` 第 15 行，`aimer.cpp` 第 15 行，`pnp_solver.cpp` 第 24 行，`auto_aim_system.cpp` 第 68-74 行  
**问题**：`AutoAimSystem` 构造函数内 `Tracker`、`Aimer`、`PnpSolver` 三个组件各自独立调用 `YAML::LoadFile(config_path)`，同一个文件被磁盘读取 3 次，每次构造都有 I/O 开销。  
**修改方法**：
1. 修改 `Tracker`、`Aimer`、`PnpSolver` 的构造函数，增加接受已解析 `YAML::Node` 的重载版本：
```cpp
// tracker.hpp 新增构造函数：
Tracker(const YAML::Node &config, PnpSolver &solver);

// pnp_solver.hpp 新增：
PnpSolver(const YAML::Node &config);

// aimer.hpp 新增：
Aimer(const YAML::Node &config);
```
2. 原来接受 `const std::string &config_path` 的构造函数保留，但内部改为：
```cpp
Tracker::Tracker(const std::string& config_path, PnpSolver& solver)
    : Tracker(YAML::LoadFile(config_path), solver) {}
```
3. 在 `AutoAimSystem` 构造函数中（`auto_aim_system.cpp` 第 68 行），一次性解析 YAML 后传给各组件：
```cpp
AutoAimSystem::AutoAimSystem(const string& model, const string& cfg, double bs) {
    auto yaml = YAML::LoadFile(cfg);
    detector_ = Detector(model);
    pnp_solver_ = PnpSolver(yaml);
    tracker_ = Tracker(yaml, pnp_solver_);
    aimer_ = Aimer(yaml);
    bullet_speed_ = bs;
    ...
}
```
4. 注意 `auto_aim_system.hpp` 目前用成员初始化列表初始化，需改为两阶段初始化（先默认构造，后赋值），或使用指针/`std::optional`。

---

### A-02 🔴【video_test.cpp 绕过 AutoAimSystem 直接使用各组件】
**文件**：`armor_task/test/video/video_test.cpp`，第 104-222 行  
**问题**：测试文件直接 `new Detector / PnpSolver / Tracker / Aimer`，而 `AutoAimSystem` 类封装了相同的流水线（含 IMU 同步）。导致两套并行的流程，任何对流程的修改必须改两处。  
**修改方法**：
1. 修改 `AutoAimSystem::processFrame()` 增加一个不需要 IMU 同步的重载（或添加一个 flag 参数），供 video_test 使用：
```cpp
// auto_aim_system.hpp 新增：
ProcessResult processFrame(const cv::Mat &img);  // 不使用 IMU，IMU 用单位四元数
```
2. 在 `video_test.cpp` 中将第 104-108 行的 4 个独立组件实例化替换为：
```cpp
AutoAimSystem sys(model_path, config_path, bullet_speed);
```
3. 将第 206-224 行的 detect→track→aim 三步调用替换为：
```cpp
auto result = sys.processFrame(display_frame);
auto& detected_armors = result.armors;
auto& targets = result.targets;
auto& latest_autoaim_cmd = result.cmd;
stats.detect_ms = result.detect_time_ms;
stats.track_ms = result.track_time_ms;
stats.aim_ms = result.aim_time_ms;
```
4. 删除第 211 行 `pnp_solver.set_R_gimbal2world(Eigen::Quaterniond(1,0,0,0))`（由 AutoAimSystem 内部处理）。

---

### A-03 🟡【aimer.aim() 接受 std::list<Target> 但只用 front()，类型不合适】
**文件**：`armor_task/tasks/aimer.hpp` 第 22 行，`aimer.cpp` 第 26-29 行  
**问题**：`aim(std::list<Target> targets, ...)` 接受一个 list，但函数内第 29 行立即 `auto target = targets.front()` 只取第一个元素，完全不使用 list 的迭代特性。调用方（`video_test.cpp` 第 220 行）还需要 `std::list<Target>(targets.begin(), targets.end())` 进行一次不必要的容器转换。  
**修改方法**：
1. 将 `aimer.hpp` 中的函数签名改为接受单个 Target：
```cpp
io::Command aim(const Target& target, std::chrono::steady_clock::time_point timestamp,
                double bullet_speed, bool to_now = false);
```
2. 在 `aimer.cpp` 第 26-29 行删除 `if (targets.empty())` 检查和 `targets.front()` 取值，直接使用传入的 `target`。
3. 修改所有调用处：
   - `video_test.cpp` 第 220-221 行：删除 `std::list<Target>` 转换，直接传 `targets[0]`
   - `auto_aim_system.cpp` 第 137-138 行：同样修改

---

### A-04 🟡【Ekf 类不在 armor_task 命名空间中】
**文件**：`armor_task/tasks/ekf.hpp`，`armor_task/tasks/ekf.cpp`  
**问题**：`Ekf` 类定义在全局命名空间，但使用它的 `Target` 在 `armor_task` 命名空间，造成命名空间不一致，增加理解成本。  
**修改方法**：
1. 在 `ekf.hpp` 中，将 `class Ekf` 的定义包裹在 `namespace armor_task { ... }`。
2. 在 `ekf.cpp` 中，将所有 `Ekf::` 实现包裹在 `namespace armor_task { ... }`（或在函数名前加 `armor_task::Ekf::`）。
3. 在所有使用 `Ekf` 的地方（`target.hpp` 的 include 和类型声明）检查是否需要加 `armor_task::` 前缀（由于 `target.hpp` 已在 `armor_task` 命名空间内，应可直接使用）。

---

### A-05 🟡【tasks/postprocess.cu 用途不明，疑似遗留文件】
**文件**：`armor_task/tasks/postprocess.cu`  
**问题**：`postprocess.cu` 同时出现在 `SOURCES` 和 `SOURCES_NO_VISUALIZER` 中（`CMakeLists.txt` 第 93 行和第 128 行），但其内容与 `cuda/yolo_decode.cu` 的功能重叠，未在代码中看到实际调用。  
**修改方法**：
1. 查看 `postprocess.cu` 的内容：`cat armor_task/tasks/postprocess.cu`
2. 如果内容是空文件或与 `yolo_decode.cu` 功能完全重叠，执行 `git rm armor_task/tasks/postprocess.cu`
3. 从 `CMakeLists.txt` 的 `SOURCES`（第 93 行）和 `SOURCES_NO_VISUALIZER`（第 128 行）中删除 `tasks/postprocess.cu`

---

### A-06 🟡【CMakeLists.txt 中包含路径包含无效的绝对路径】
**文件**：`armor_task/CMakeLists.txt`，第 54-56 行  
**问题**：
```cmake
include_directories(/opt/ros/humble/include/cv_bridge)
include_directories(/usr/local/include/cv_bridge)
include_directories(/home/user/vision_opencv/cv_bridge/include)  # 不存在的路径！
```
第 56 行 `/home/user/vision_opencv/...` 是某个开发者环境的绝对路径，在其他机器上不存在。  
**修改方法**：
1. 删除第 54-56 行这三个 `include_directories` 硬编码行。
2. cv_bridge 的头文件已通过 `find_package(cv_bridge REQUIRED)` 和后面的 `${cv_bridge_INCLUDE_DIRS}` 正确引入（第 68 行），无需手动指定路径。
3. 验证删除后 `cmake ..` 仍能找到 cv_bridge 头文件（通过 `message(STATUS ...)` 确认）。

---

### A-07 🟡【SOURCES 和 SOURCES_NO_VISUALIZER 几乎完全相同，维护双份列表】
**文件**：`armor_task/CMakeLists.txt`，第 91-130 行  
**问题**：`SOURCES` 和 `SOURCES_NO_VISUALIZER` 两个列表几乎一样，唯一区别是后者不含 `tools/visualizer.cpp`。每次新增源文件都要在两处添加，容易遗漏。  
**修改方法**：
1. 只保留 `SOURCES_NO_VISUALIZER`（改名为 `CORE_SOURCES`）。
2. 将 `SOURCES` 改为在 `CORE_SOURCES` 基础上追加：
```cmake
set(CORE_SOURCES
    tasks/auto_aim_system.cpp
    ...  # 共同部分
    cuda/yolo_decode.cu
)
set(SOURCES ${CORE_SOURCES} tools/visualizer.cpp)
```
3. 需要 visualizer 的目标用 `${SOURCES}`，不需要的用 `${CORE_SOURCES}`。

---

### A-08 🟢【ProcessResult.tracker_state 应用 TrackerState 枚举而非 string】
**文件**：`armor_task/tasks/auto_aim_system.hpp`，第 52 行  
**问题**：`ProcessResult` 中的 `tracker_state` 字段类型是 `std::string`（第 52 行），和 S-05 中同样的问题——string 比较开销高且不安全。  
**修改方法**：在完成 S-05（引入 `TrackerState` 枚举）之后：
1. 将第 52 行 `std::string tracker_state;` 改为 `TrackerState tracker_state;`
2. 修改 `auto_aim_system.cpp` 第 92 行和第 127 行对应的赋值，改为传 `tracker_.state_enum()`（需在 S-05 中同步添加该接口）。
3. 修改所有使用 `result.tracker_state` 的调用方，从字符串比较改为枚举比较。

---

### A-09 🟢【config 下两个 yaml 中部分参数硬编码在代码中，缺乏文档说明】
**文件**：`armor_task/config/camera1.yaml`  
**问题**：一些关键调试参数分散在 C++ 代码中（`target.cpp` 第 70-72 行的 `v1=100, v1_y=100, v2=400`；`target.cpp` 第 272 行的 P0 对角线值），无法通过配置文件调整，每次调参都要重新编译。  
**修改方法**：
1. 在 `camera1.yaml` 中添加 EKF 过程噪声配置节：
```yaml
ekf:
  process_noise_v1: 100.0    # x/z 方向加速度方差
  process_noise_v1y: 100.0   # y 方向加速度方差
  process_noise_v2: 400.0    # 角加速度方差
  p0_diag: [10, 64, 10, 64, 10, 64, 0.4, 100, 0.1, 0.1, 0.1]
```
2. 在 `Target` 构造函数（`target.cpp` 第 6 行）增加从 YAML 读取这些参数的能力，或在 `Tracker::set_target()` 中读取后传入 `Target` 构造函数。
3. 这样调参时直接修改 YAML 文件，无需重编译。

---

## 四、test/deploy 目录专项整改

> `test/deploy/` 是实际上机部署的入口，与核心库耦合最深，问题最集中。

---

### D-01 🔴【auto_aimer.cpp 完整复刻了 AutoAimSystem 和 IMUHistory，形成双份维护】
**文件**：`armor_task/test/deploy/auto_aimer.cpp`，第 66-153 行、第 213-222 行、第 299-416 行  
**问题**：
1. 第 66-153 行定义了局部 `IMUHistory` 类——与 `auto_aim_system.hpp` 第 27-39 行的 `armor_task::IMUHistory` 实现**完全相同**（含 `push`, `query`, `size`，以及同样的 100 条上限和二分查找逻辑）。
2. 第 213-222 行手动实例化 `Detector`、`PnpSolver`、`Tracker`、`Aimer` 四个组件，第 494-634 行手动复现了 detect→IMU sync→track→aim 的完整流水线——这与 `AutoAimSystem::processFrame()` 的逻辑完全重复。  
   任何对核心算法流程的修改，都必须在 `auto_aim_system.cpp` 和 `auto_aimer.cpp` 两处同步，极易遗漏。

**修改方法**：
1. 删除 `auto_aimer.cpp` 第 66-153 行的局部 `IMUHistory` 类定义，改为直接使用 `#include "../../tasks/auto_aim_system.hpp"` 引入的 `armor_task::IMUHistory`。
2. 删除第 213-222 行的四组件手动初始化，替换为：
```cpp
AutoAimSystem auto_aim_sys(yolo_model_path, config_path, bullet_speed);
```
3. 将 `AutoAimSystem::updateImu()` 暴露给外部调用（已有该接口），在 IMU 接收线程（第 339 行）处改为：
```cpp
auto_aim_sys.updateImu(quat, imu_yaw, imu_pitch);
```
4. 将主循环第 494-634 行的手动 detect/track/aim 替换为：
```cpp
auto result = auto_aim_sys.processFrame(img, estimated_exposure_time);
// 从 result.cmd、result.armors、result.targets 取数
```
5. 根据 result 结果处理 `is_switching`（`result.is_switching`）、开火逻辑、绘图等，不再手动访问 tracker/aimer 内部状态。

---

### D-02 🔴【auto_aimer.cpp 热路径存在多处 std::cout 打印】
**文件**：`armor_task/test/deploy/auto_aimer.cpp`  
**问题**：以下几处 `std::cout` 位于每帧必经的热路径，100fps 下每秒打印数百行，严重拖慢主线程：
- 第 496 行：`std::cout << "armorsize detect"<< armors.size()<<std::endl;`
- 第 500-502 行：对每个检测装甲板打印置信度
- 第 572 行：`std::cout << "is red: " << self_is_red << std::endl;`
- 第 402 行（send_thread）：`std::cout << "valid:" << cmd_to_send.valid ...`
- 第 410 行（send_thread）：另一处发送日志

**修改方法**：
1. 删除第 496 行的打印（或改为 `if (frame_count % 100 == 0)` 每 100 帧打印一次）。
2. 删除第 500-502 行对每个装甲板的置信度打印。
3. 删除第 572 行 `is red` 打印（颜色初始化只需一次，可在首次变化时打印）。
4. 将第 402 和 410 行 send_thread 内的 `std::cout` 替换为频率受控的打印（每秒最多打印一次）。

---

### D-03 🔴【detector.cpp 使用裸指针 new io::USB，资源不安全】
**文件**：`armor_task/test/deploy/detector.cpp`，第 58-67 行  
**问题**：
```cpp
io::USB *usb = nullptr;
try {
    usb = new io::USB(send_port, receive_port);  // 裸指针
    ...
}
```
若程序在 `new` 之后、`delete usb` 之前抛出异常退出，内存和文件描述符将泄漏。`auto_aimer.cpp` 已经正确使用了 `std::unique_ptr<io::USB>`，但 `detector.cpp` 没有。  
**修改方法**：
1. 将第 58 行的 `io::USB *usb = nullptr;` 改为：
```cpp
std::unique_ptr<io::USB> usb;
```
2. 将第 61 行的 `usb = new io::USB(send_port, receive_port);` 改为：
```cpp
usb = std::make_unique<io::USB>(send_port, receive_port);
```
3. 将所有 `usb->` 调用保持不变（unique_ptr 支持 `->` 操作符）。
4. 删除第 205-208 行的手动 `delete usb;`（unique_ptr 析构时自动释放）。

---

### D-04 🟡【auto_aimer.cpp 第 645 行仍在用字符串比较 tracker 状态】
**文件**：`armor_task/test/deploy/auto_aimer.cpp`，第 645 行  
**问题**：`tracker.state() == "tracking"` 是字符串比较，与 S-05 中的枚举化改造目标冲突。若 S-05 完成后 `state()` 返回 `const char*`，字符串字面量比较仍然能工作，但语义上应该用枚举或 `state_name()` 函数。  
**修改方法**：
1. 完成 S-05（引入 `TrackerState` 枚举）之后，在 `Tracker` 类添加 `state_enum()` 接口。
2. 将第 645 行改为：
```cpp
tracker.state_enum() == TrackerState::TRACKING
```
3. 同样检查 `auto_aimer.cpp` 中其他所有 `tracker.state()` 字符串比较，统一替换。

---

### D-05 🟡【auto_aimer.cpp 第 631 行不必要地将 vector 转为 list】
**文件**：`armor_task/test/deploy/auto_aimer.cpp`，第 631 行  
**问题**：`std::list<Target> target_list(targets.begin(), targets.end())` 对每帧有目标时都执行一次 vector→list 的深拷贝，而 `aimer.aim()` 只用 `targets.front()`。  
**修改方法**：在完成 A-03（修改 `aimer.aim()` 接受 `const Target&`）之后：
1. 删除第 631 行的 `std::list<Target> target_list(...)` 转换。
2. 将第 634 行改为：
```cpp
auto_cmd = aimer.aim(targets[0], frame_start, bullet_speed);
```

---

### D-06 🟡【communication.cpp 和 auto_aimer.cpp 使用 3.14159265 而非 M_PI】
**文件**：`armor_task/test/deploy/communication.cpp`，第 131、134、169、174 行；`auto_aimer.cpp` 多处  
**问题**：直接使用 `3.14159265` 字面量（精度不足，正确值应为 `3.14159265358979...`）进行弧度转角度换算，且含义不直观。  
**修改方法**：
1. 在 `communication.cpp` 文件顶部 `#include` 之后添加：
```cpp
#include <cmath>
static constexpr double kRad2Deg = 180.0 / M_PI;
```
2. 将所有 `* 180.0 / 3.14159265` 替换为 `* kRad2Deg`（共约 4 处）。
3. 在 `auto_aimer.cpp` 中同样处理（搜索 `3.14159`）。

---

### D-07 🟡【auto_aimer.cpp 中 IMU/串口/图像三个线程逻辑与其他测试文件重复】
**文件**：`armor_task/test/deploy/auto_aimer.cpp`（第 299-449 行），`armor_task/test/deploy/target.cpp`（类似结构）  
**问题**：`auto_aimer.cpp` 和 `target.cpp` 都有几乎一样的三线程结构：
1. IMU 接收线程（`while(running)` → `usb->receive_all()` → push to history）
2. 串口命令发送线程（`while(running)` → 读 mutex → `usb->send_command()`）
3. 图像接收线程（`while(running)` → `ros_node->get_frame_packet()` → notify_one）

这段逻辑在每个测试文件中手写一遍，任何修改（如频率控制、错误处理）都要改多处。  
**修改方法**：
1. 在 `armor_task/io/` 目录下新建 `deploy_runner.hpp`，定义 `DeployRunner` 类，封装以上三个线程和对应的 mutex/condition_variable：
```cpp
// io/deploy_runner.hpp
class DeployRunner {
public:
    DeployRunner(std::shared_ptr<ROS2Manager> ros_node,
                 std::unique_ptr<io::USB> usb,
                 AutoAimSystem& sys);
    void start();
    void stop();
    // 回调：每帧图像就绪时调用
    void setFrameCallback(std::function<void(const cv::Mat&,
                          std::chrono::steady_clock::time_point)> cb);
};
```
2. 将 `auto_aimer.cpp` 和 `target.cpp` 中的三线程逻辑迁移到 `DeployRunner` 实现。
3. 各测试文件 `main()` 只负责：初始化 + `runner.start()` + 等待退出 + `runner.stop()`。

---

### D-08 🟢【auto_aimer.cpp 第 365 行裸 catch(...) 吞掉所有异常信息】
**文件**：`armor_task/test/deploy/auto_aimer.cpp`，第 365-367 行  
**问题**：IMU 接收线程中：
```cpp
catch (...) {
    // 空 catch，所有异常被静默吞掉
}
```
串口读取出现任何错误（断线、格式错误、设备异常）都会被静默忽略，给调试带来极大困难。  
**修改方法**：
1. 将 `catch (...)` 改为先捕获具体异常再兜底：
```cpp
catch (const std::exception& e) {
    // 控制打印频率，避免刷屏
    static int err_count = 0;
    if (++err_count % 100 == 0)
        std::cerr << "[IMU Thread] Exception: " << e.what() << std::endl;
}
catch (...) {
    static int unk_count = 0;
    if (++unk_count % 100 == 0)
        std::cerr << "[IMU Thread] Unknown exception caught" << std::endl;
}
```
2. 若频繁出现异常，增加退出条件（如连续 N 次异常则 `running = false`）。

---

## 五、AutoAimSystem 封装 Bug（auto_aimer.cpp 可用但 AutoAimSystem 不可用）

> 以下三个问题共同导致 `AutoAimSystem` 在部署场景下无法正常工作，而手写版 `auto_aimer.cpp` 可以。

---

### B-01 🔴【AutoAimSystem 缺少 updateJudgerData 接口，enemy_color 从不更新】
**文件**：`armor_task/tasks/auto_aim_system.hpp/.cpp`

**根因**：在 `auto_aimer.cpp` 中，IMU 线程通过 `usb->receive_all()` 同时收到 **IMU 四元数** 和 **裁判数据（JudgerData）**，然后主线程用裁判数据中的 `self_id` 推断己方颜色，调用 `tracker.get_enemy_color()` 更新敌方颜色：

```cpp
// auto_aimer.cpp 第 568-574 行（正常工作）
bool self_is_red = tools::get_color_from_self_id(latest_judger_data);
tracker.get_enemy_color(self_is_red);
```

而 `AutoAimSystem::updateImu()` 只接受四元数，**没有 JudgerData 参数**，`processFrame()` 也从未调用 `tracker_.get_enemy_color()`。导致 `tracker_` 的 `enemy_color_` 从 YAML 读取后**永远不会更新**。

**后果**：如果 YAML 中 `enemy_color: blue` 但实际应该打红色（或反过来），`tracker_` 会过滤掉所有装甲板，跟踪结果永远为空。

**修改方法**：

1. 在 `auto_aim_system.hpp` 的 `AutoAimSystem` 类中新增接口：
```cpp
// 新增：接收下位机完整数据（IMU + 裁判）
void updateImuAndJudger(const Eigen::Quaterniond &quat, double yaw, double pitch,
                        const io::JudgerData &judger_data);
```

2. 在 `auto_aim_system.cpp` 实现：
```cpp
void AutoAimSystem::updateImuAndJudger(const Eigen::Quaterniond &quat, double yaw,
                                        double pitch, const io::JudgerData &judger_data)
{
    imu_history_.push(quat, yaw, pitch);
    // 更新敌方颜色（工具函数需要 non-const 引用，故拷贝一份）
    io::JudgerData jd = judger_data;
    bool self_is_red = tools::get_color_from_self_id(jd);
    tracker_.get_enemy_color(self_is_red);
}
```

3. 同时保留原有 `updateImu()` 作为兼容接口（内部调用 `imu_history_.push()`）。

4. 调用方（IMU 线程）改为调用 `updateImuAndJudger()`：
```cpp
// 原来：
auto_aim_sys.updateImu(quat, yaw, pitch);
// 改为：
auto_aim_sys.updateImuAndJudger(quat, yaw, pitch, judger_data);
```

---

### B-02 🔴【AutoAimSystem 返回的 cmd.shoot 永远为 false】
**文件**：`armor_task/tasks/auto_aim_system.cpp`，第 138 行；`armor_task/tasks/aimer.cpp`，第 119 行

**根因**：`aimer_.aim()` 的返回值硬编码 `shoot = false`（`aimer.cpp` 第 119 行）：
```cpp
return {true, false, static_cast<float>(yaw), static_cast<float>(pitch)};
//           ^^^^^ shoot 永远 false
```

`auto_aimer.cpp` 第 670 行显式覆盖了这个值：
```cpp
auto_cmd.shoot = auto_cmd.valid;   // 有效命令时允许开火
```

但 `auto_aim_system.cpp` 第 138 行没有这个覆盖：
```cpp
result.cmd = aimer_.aim(target_list, track_end, bullet_speed_);
// result.cmd.shoot 永远是 false → 机器人永远不开火
```

**修改方法**：在 `auto_aim_system.cpp` 第 138 行之后立即添加：
```cpp
result.cmd = aimer_.aim(target_list, track_end, bullet_speed_);
result.cmd.shoot = result.cmd.valid;   // 补上这一行
```

---

### B-03 🟡【AutoAimSystem 未发布 update_aimer_data，底盘运动判断失效】
**文件**：`armor_task/test/deploy/auto_aimer.cpp`，第 687-692 行（有）；`auto_aim_system.cpp`（缺失）

**根因**：`auto_aimer.cpp` 每帧向 ROS2 话题 `/autoaim_data` 发布：
```cpp
const io::AimerData aim_result_pub = tools::from_vis_dec(cmd_valid, latest_judger_data);
ros_node->update_aimer_data(aim_result_pub);
```

底盘运动控制节点订阅该话题来判断是否有目标（决定是否转圈巡逻）。`AutoAimSystem` 是 IO 无关设计，不包含此发布——属于**预期行为**，但调用方**必须**手动处理：

**修改方法**：在使用 `AutoAimSystem` 的调用方代码中，`processFrame` 之后补上：
```cpp
auto result = auto_aim_sys.processFrame(img, ts);

// 需要手动发布（AutoAimSystem 不含 IO，需调用方负责）
{
    std::lock_guard<std::mutex> lock(imu_mutex);
    auto aim_data = tools::from_vis_dec(result.cmd.valid, latest_judger_data);
    ros_node->update_aimer_data(aim_data);
}
```

---

### B-04 🟡【ros2_manager.cpp 图像时间戳转换依赖 system_clock 与 steady_clock 同步假设】
**文件**：`armor_task/io/ros2_manager.cpp`，第 52-62 行

**根因**：图像时间戳从 `system_clock`（ROS2 header，Unix 纪元）转换为 `steady_clock`：
```cpp
auto time_diff = std::chrono::duration<double>(ros_time_sec - now_system_sec);
auto frame_timestamp = now_steady + time_diff;   // 假设两个时钟的"现在"是同一物理时刻
```

此转换在同一台机器上通常正确，但存在隐患：
- **`time_diff` 是负数**（图像曝光时刻早于 callback 触发时刻），正常。
- **若相机驱动 ROS 时间与本机 system_clock 存在偏差**（如 NTP 未同步、跨机器通信），`frame_timestamp` 会与 `IMUHistory` 中的 `steady_clock` 时间戳对不上，导致 `query()` 总是返回最早或最晚的 IMU 数据，而不是曝光时刻的数据。

**排查方法**：添加临时调试打印，确认时间戳差值在合理范围（< 100ms）：
```cpp
// 在 processFrame 开始处临时添加（排查后删除）：
auto now = std::chrono::steady_clock::now();
auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - image_timestamp).count();
std::cout << "[Timestamp diff] " << diff_ms << " ms" << std::endl;
// 正常值应为 10-80ms（图像处理延迟）
// 若超过 500ms 或为负数，说明时间戳转换有问题
```

**根本修复**（如时间戳偏差较大）：改为在 callback 收到图像时就记录 `steady_clock::now()` 作为时间戳，放弃使用 ROS2 header 时间：
```cpp
// ros2_manager.cpp image_callback 中替换：
// 原来：基于 ROS header 时间计算的 frame_timestamp
// 改为：直接使用 callback 触发时刻
auto frame_timestamp = std::chrono::steady_clock::now();
packet->timestamp = frame_timestamp;
```
代价是丢失精确曝光时刻信息，但实践中 callback 触发延迟（< 5ms）通常可接受。

---

## 快速验证顺序建议

以下任务按实施难度和收益排序，建议按顺序进行：

| 顺序 | 任务                                    | 预计 FPS 增益                | 实施难度 |
| ---- | --------------------------------------- | ---------------------------- | -------- |
| 1    | P-01 修改 CMakeLists 构建类型           | 高（最大，Release vs Debug） | 5分钟    |
| 2    | P-02 删除 hot path std::cout（含 D-02） | 中                           | 10分钟   |
| 3    | S-02 删除大量注释旧代码                 | 无直接，但减少干扰           | 10分钟   |
| 4    | D-03 detector.cpp 改 unique_ptr         | 无直接，安全性               | 5分钟    |
| 5    | S-01 简化 CMakeLists 重复链接           | 无                           | 20分钟   |
| 6    | P-03 减少 CUDA sync                     | 小（每帧约 0.5ms）           | 30分钟   |
| 7    | P-05 aimer 减少 Target 副本             | 小                           | 15分钟   |
| 8    | S-05 状态机改 enum（D-04 联动）         | 无直接，代码质量             | 30分钟   |
| 9    | A-01 YAML 单次解析                      | 微小（仅影响启动）           | 45分钟   |
| 10   | P-04 矩阵缓存                           | 小                           | 45分钟   |
| 11   | D-01 auto_aimer 用 AutoAimSystem        | 消除双份维护                 | 90分钟   |
| 12   | D-07 提取 DeployRunner                  | 无直接                       | 120分钟  |
