#pragma once

#include "structures.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace armor_task {
class Detector
{
  public:
    // 构造函数
    Detector(const std::string &yolo_model_path);

    // 析构函数
    ~Detector() = default;

    /*
    主要检测函数:
        输入：带装甲板的图片     类型：cv::Mat
        输出：装甲板列表        类型：ArmorArray
    */
    ArmorArray detect(const cv::Mat &frame);

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

  private:
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
};
} // namespace armor_task