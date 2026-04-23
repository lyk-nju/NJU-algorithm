#pragma once

#include "structures.hpp"

#include "../../cuda/yolo_decode.cuh"

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>

namespace armor_task
{
void launch_preprocess_kernel(
    const uint8_t *src, int src_w, int src_h, int src_step, // 输入：原始图??
    float *dst, int dst_w, int dst_h,                       // 输出：模型输??buffer
    cudaStream_t stream);

class Logger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity != Severity::kINFO) std::cout << "[TRT] " << msg << std::endl;
    }
};

class Detector
{
  public:
    // 默认加载 video_test 使用??TensorRT 引擎
    Detector();

    explicit Detector(const std::string &yolo_model_path);

    ~Detector();

    /*
    主要检测函数：
        输入：带装甲板的图片  类型：cv::Mat
        输出：装甲板列表     类型：ArmorArray
    */
    ArmorArray detect(const cv::Mat &frame);
    bool usingTensorRt() const { return is_trt_; }
    const char *backendName() const { return is_trt_ ? "TensorRT" : "OpenCV-DNN"; }

  private:
    // 模块一：预处理（letterbox + 归一化）
    cv::Mat preprocess(const cv::Mat &input_image, int target_width, int target_height);

    // 模块二：推理 + 后处理，返回装甲板列??
    ArmorArray search_armors(const cv::Mat &frame, const cv::Mat &input_blob);

    // ??4 个角点提取左右灯??
    std::pair<LightBar, LightBar> extract_lightbars_from_corners(const std::vector<cv::Point2f> &corners);

    // 根据类别 ID 推断装甲板颜??
    Color get_armor_color(int class_id);

    cv::dnn::Net yolo_net_; // YOLO 检测网络（OpenCV DNN 路径使用??

    // 模型参数
    int input_width_;
    int input_height_;
    float confidence_threshold_;
    float nms_threshold_;

    // letterbox 预处理参数（与训练时??zero-padding 对齐??
    float scale_; // 缩放比例
    float pad_x_; // ??640x640 上的 x 方向 padding
    float pad_y_; // ??640x640 上的 y 方向 padding

    std::vector<std::string> class_names_;

    // TensorRT 资源
    bool is_trt_ = false;
    Logger logger_;
    nvinfer1::IRuntime *runtime_ = nullptr;
    nvinfer1::ICudaEngine *engine_ = nullptr;
    nvinfer1::IExecutionContext *context_ = nullptr;

    cudaStream_t stream_ = nullptr;
    std::string input_name_;
    std::string output_name_;
    void *device_input_buffer_ = nullptr;
    void *device_output_buffer_ = nullptr;

    size_t output_size_ = 0;
    std::vector<float> cpu_output_buffer_;

    uint8_t *device_image_buffer_ = nullptr;
    size_t device_image_buffer_size_ = 0;
};
} // namespace armor_task
