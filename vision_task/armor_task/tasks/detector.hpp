#pragma once

#include "structures.hpp"
//#include "preprocess.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#if ARMOR_TASK_WITH_TENSORRT_CUDA
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include "../cuda/yolo_decode.cuh"
#endif

namespace armor_task {
#if ARMOR_TASK_WITH_TENSORRT_CUDA
void launch_preprocess_kernel(
    const uint8_t* src, int src_w, int src_h, int src_step, // 输入：原始图像信息
    float* dst, int dst_w, int dst_h,                       // 输出：模型输入 buffer
    cudaStream_t stream
);
class Logger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char* msg) noexcept override
    {
        if (severity != Severity::kINFO)
            std::cout << "[TRT] " << msg << std::endl;
    }
};
#endif

class Detector
{
  public:
    // 构造函数
    Detector(const std::string &yolo_model_path);

    // 析构函数
    ~Detector();

    /*
    主要检测函数:
        输入：带装甲板的图片     类型：cv::Mat
        输出：装甲板列表        类型：ArmorArray
    */
    ArmorArray detect(const cv::Mat &frame);
    bool usingTensorRt() const { return is_trt_; }
    const char *backendName() const { return is_trt_ ? "TensorRT" : "OpenCV-DNN"; }

  private:
    // 模块一：预处理
    cv::Mat preprocess(const cv::Mat &input_image, int target_width, int target_height);

    // 模块二：使用YOLO进行装甲板搜索
    ArmorArray search_armors(const cv::Mat &frame, const cv::Mat &input_blob);

    // 模块三：后处理
    void postprocess(ArmorArray &armors, const cv::Mat &frame);

    // 模块3.1：数字分类
    void classify(Armor &armor, const cv::Mat &frame);

    // 在指定区域提取灯条
    std::pair<LightBar, LightBar> extract_lightbars_from_corners(const std::vector<cv::Point2f> &corners);

    // 辅助函数
    Color get_armor_color(int class_id);

    // 用于强化图像特征（二值化）
    cv::Mat preprocessImage(const cv::Mat &img);

    cv::dnn::Net yolo_net_;   // YOLO检测网络
    cv::dnn::Net resnet_net_; // ResNet数字识别网络

    // 模型参数
    int input_width_;
    int input_height_;
    float confidence_threshold_;
    float nms_threshold_;

    // letterbox 预处理参数（与训练时的 zero-padding 对齐）
    float scale_;  // 缩放比例
    float pad_x_;  // 左右方向总 padding 的一半（在 640x640 上的像素）
    float pad_y_;  // 上下方向总 padding 的一半（在 640x640 上的像素）

    // 类别名称
    std::vector<std::string> class_names_;
    std::vector<std::string> number_names_;

    // TensorRT
    bool is_trt_ = false;
#if ARMOR_TASK_WITH_TENSORRT_CUDA
    Logger logger_;
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    
    cudaStream_t stream_ = nullptr;
    std::string input_name_;
    std::string output_name_;
    void* device_input_buffer_ = nullptr;
    void* device_output_buffer_ = nullptr;

    uint8_t* device_src_buffer_=nullptr;
    size_t  device_src_buffer_size=0;
    size_t output_size_;
    std::vector<float> cpu_output_buffer_;

    uint8_t* device_image_buffer_=nullptr;
    size_t device_image_buffer_size_=0;
#endif
};
} // namespace armor_task
