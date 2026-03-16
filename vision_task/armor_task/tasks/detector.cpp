#include "detector.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/dnn.hpp>

namespace armor_task
{
Detector::Detector(const std::string &yolo_model_path) : input_width_(640), input_height_(640), confidence_threshold_(0.4f), nms_threshold_(0.5f), scale_(1.0f), pad_x_(0.0f), pad_y_(0.0f)
{
    // 初始化类别名称 - 36个装甲板类别
    class_names_.resize(36);
    // 1-9: 蓝色装甲板 (数字1-9)
    for (int i = 1; i <= 9; i++)
    {
        class_names_[i - 1] = "blue_" + std::to_string(i);
    }
    // 10-18: 红色装甲板 (数字1-9)
    for (int i = 10; i <= 18; i++)
    {
        class_names_[i - 1] = "red_" + std::to_string(i - 9);
    }
    // 19-36: 其他类别（如有需要可扩展）
    for (int i = 19; i <= 36; i++)
    {
        class_names_[i - 1] = "other_" + std::to_string(i - 18);
    }
    // 加载YOLO模型
    yolo_net_ = cv::dnn::readNet(yolo_model_path);
    // 步骤 2: 检查模型是否成功加载
    if (yolo_net_.empty())
    {
        std::cerr << "Error: Could not load YOLO model from: " << yolo_model_path << std::endl;
        // 如果模型加载失败，应当立即抛出异常或返回，避免后续操作
        throw std::runtime_error("Failed to load YOLO model.");
    }
    else
    {
        std::cout << "YOLO model loaded successfully from: " << yolo_model_path << std::endl;
    }

    // 步骤 3: 设置 CUDA 后端和目标（仅在模型加载成功后）
    try
    {
        yolo_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA); // 后端
        yolo_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);   // 目标

        std::cout << "YOLO model configured for CUDA GPU acceleration (FP32)." << std::endl;
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "Fatal: CUDA setting failed. Please check your OpenCV build and CUDA environment. Error: " << e.what() << std::endl;
        // 如果 CUDA 设置失败，程序应该报错并退出，避免运行在慢速的 CPU 模式
        throw;
    }

    // 检查模型是否成功加载
    if (yolo_net_.empty())
    {
        std::cerr << "Error: Could not load YOLO model from: " << yolo_model_path << std::endl;
    }
    else
    {
        std::cout << "YOLO model loaded successfully from: " << yolo_model_path << std::endl;
    }
}

ArmorArray Detector::detect(const cv::Mat &frame)
{
    // 模块一：预处理
    cv::Mat input_blob = preprocess(frame, input_width_, input_height_);

    // 模块二：搜索装甲板
    ArmorArray armors = search_armors(frame, input_blob);

    return armors;
}

cv::Mat Detector::preprocess(const cv::Mat &input_image, int target_width, int target_height)
{
    // 确保输入图像是有效的
    if (input_image.empty())
    {
        std::cerr << "Error: Empty input image for preprocessing" << std::endl;
        return cv::Mat();
    }

    // 与训练时一致的 letterbox 预处理（保持长宽比，四周 zero padding）
    const int src_w = input_image.cols;
    const int src_h = input_image.rows;

    // 缩放比例：短边对齐到 target，长边按比例缩放
    float r_w = static_cast<float>(target_width) / static_cast<float>(src_w);
    float r_h = static_cast<float>(target_height) / static_cast<float>(src_h);
    scale_ = std::min(r_w, r_h);

    int new_unpad_w = static_cast<int>(std::round(src_w * scale_));
    int new_unpad_h = static_cast<int>(std::round(src_h * scale_));

    pad_x_ = (target_width - new_unpad_w) * 0.5f;
    pad_y_ = (target_height - new_unpad_h) * 0.5f;

    // 图像缩放到 new_unpad，并复制到 640x640 的黑色背景上
    cv::Mat resized;
    cv::resize(input_image, resized, cv::Size(new_unpad_w, new_unpad_h));

    cv::Mat padded(target_height, target_width, input_image.type(), cv::Scalar(0, 0, 0));
    cv::Rect roi(static_cast<int>(pad_x_), static_cast<int>(pad_y_), new_unpad_w, new_unpad_h);
    resized.copyTo(padded(roi));

    // 创建输入 blob: 归一化 + 通道交换
    cv::Mat blob;
    cv::dnn::blobFromImage(padded, blob, 1.0f / 255.0f, cv::Size(target_width, target_height), cv::Scalar(0, 0, 0), true, false, CV_32F);
    return blob;
}

ArmorArray Detector::search_armors(const cv::Mat &frame, const cv::Mat &input_blob)
{
    ArmorArray detected_armors;

    // 设置输入并运行网络
    yolo_net_.setInput(input_blob);
    std::vector<cv::Mat> outputs;

    try
    {
        // 获取输出层名称
        std::vector<std::string> outNames = yolo_net_.getUnconnectedOutLayersNames();
        yolo_net_.forward(outputs, outNames);
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "Error during DNN forward pass: " << e.what() << std::endl;
        return detected_armors;
    }

    if (outputs.empty())
    {
        std::cerr << "Error: No outputs from the network." << std::endl;
        return detected_armors;
    }

    const cv::Mat &detection_output = outputs[0];

    if (detection_output.empty())
    {
        std::cerr << "Error: Empty detection output." << std::endl;
        return detected_armors;
    }

    // 根据输出张量的维度进行不同的处理
    if (detection_output.dims == 3)
    {
        int rows = detection_output.size[1];
        int cols = detection_output.size[2];

        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        std::vector<std::vector<cv::Point2f>> armor_corners;

        cv::Mat transposed;
        cv::Mat reshaped = detection_output.reshape(1, rows * cols);
        cv::transpose(reshaped.reshape(1, rows), transposed);

        for (int i = 0; i < cols; i++)
        {

            cv::Mat scores = transposed.row(i).colRange(4, 40); // 4-39是36个类别置信度

            cv::Point class_id_point;
            double max_class_score;
            cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id_point);

            // 只保留高置信度的检测结果
            if (max_class_score > confidence_threshold_)
            {
                // 获取边界框坐标
                float cx = transposed.at<float>(i, 0); // 中心x
                float cy = transposed.at<float>(i, 1); // 中心y
                float w = transposed.at<float>(i, 2);  // 宽度
                float h = transposed.at<float>(i, 3);  // 高度

                // 获取装甲板四个关键点（网络输出坐标基于 640x640 letterbox 图像）
                std::vector<cv::Point2f> corners(4);
                for (int j = 0; j < 4; j++)
                {
                    float kx = transposed.at<float>(i, 40 + j * 2);     // 关键点 x（letterbox 坐标）
                    float ky = transposed.at<float>(i, 40 + j * 2 + 1); // 关键点 y

                    // 反变换：去除 padding，再除以缩放系数，恢复到原始图像坐标系
                    float x_orig = (kx - pad_x_) / scale_;
                    float y_orig = (ky - pad_y_) / scale_;

                    corners[j].x = x_orig;
                    corners[j].y = y_orig;
                }

                // bbox 同样在 letterbox 坐标系下，需要反变换回原图
                float left_l = cx - 0.5f * w;
                float top_l = cy - 0.5f * h;

                float left = (left_l - pad_x_) / scale_;
                float top = (top_l - pad_y_) / scale_;
                float width = w / scale_;
                float height = h / scale_;

                // 裁剪到图像范围，避免越界
                left = std::max(0.0f, std::min(left, static_cast<float>(frame.cols - 1)));
                top = std::max(0.0f, std::min(top, static_cast<float>(frame.rows - 1)));
                width = std::max(1.0f, std::min(width, static_cast<float>(frame.cols) - left));
                height = std::max(1.0f, std::min(height, static_cast<float>(frame.rows) - top));

                boxes.push_back(cv::Rect(static_cast<int>(left), static_cast<int>(top), static_cast<int>(width), static_cast<int>(height)));
                class_ids.push_back(class_id_point.x); // x 是列索引
                confidences.push_back(static_cast<float>(max_class_score));
                armor_corners.push_back(corners);
            }
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, indices);

        // 为装甲板分配ID并创建对象
        int armor_id = 0;
        for (size_t i = 0; i < indices.size(); ++i)
        {
            int idx = indices[i];
            cv::Rect box = boxes[idx];
            int class_id = class_ids[idx];
            float conf = confidences[idx];
            std::vector<cv::Point2f> corners = armor_corners[idx];

            // 确保类别ID在范围内且边界框有效
            if (class_id < static_cast<int>(class_names_.size()) && box.width > 0 && box.height > 0 && box.x >= 0 && box.y >= 0 && box.x + box.width < frame.cols && box.y + box.height < frame.rows)
            {
                // 从角点提取灯条信息
                auto lightbars = extract_lightbars_from_corners(corners);

                // 创建装甲板对象
                Armor armor(lightbars.first, lightbars.second, armor_id++, box);
                armor.confidence = conf;
                armor.color = get_armor_color(class_id);
                armor.corners = corners; // 保存角点信息

                // 从类别ID提取数字信息
                if (class_id >= 0 && class_id < 18)
                { // 只有装甲板类别才有数字
                    if (class_id < 9)
                    {
                        armor.car_num = class_id; // 蓝色装甲板 1-9
                    }
                    else
                    {
                        armor.car_num = (class_id - 9); // 红色装甲板 1-9
                    }
                }
                else
                {
                    armor.car_num = -1; // 非装甲板
                }

                detected_armors.push_back(armor);
            }
        }
    }

    return detected_armors;
}

std::pair<LightBar, LightBar> Detector::extract_lightbars_from_corners(const std::vector<cv::Point2f> &corners)
{
    LightBar left_lightbar, right_lightbar;

    if (corners.size() != 4)
    {
        return std::make_pair(left_lightbar, right_lightbar);
    }

    // 按x坐标排序找到左右两对点
    std::vector<std::pair<cv::Point2f, int>> sorted_corners;
    for (int i = 0; i < 4; i++)
    {
        sorted_corners.push_back({corners[i], i});
    }
    std::sort(sorted_corners.begin(), sorted_corners.end(), [](const auto &a, const auto &b) { return a.first.x < b.first.x; });

    // 左侧两个点
    cv::Point2f left1 = sorted_corners[0].first;
    cv::Point2f left2 = sorted_corners[1].first;
    // 右侧两个点
    cv::Point2f right1 = sorted_corners[2].first;
    cv::Point2f right2 = sorted_corners[3].first;

    // 按y坐标确定上下点
    if (left1.y > left2.y) std::swap(left1, left2);
    if (right1.y > right2.y) std::swap(right1, right2);

    // 左灯条：上点和下点
    left_lightbar.top = left1;
    left_lightbar.bottom = left2;
    left_lightbar.center = (left_lightbar.top + left_lightbar.bottom) * 0.5f;
    left_lightbar.top2bottom = left_lightbar.bottom - left_lightbar.top;

    // 右灯条：上点和下点
    right_lightbar.top = right1;
    right_lightbar.bottom = right2;
    right_lightbar.center = (right_lightbar.top + right_lightbar.bottom) * 0.5f;
    right_lightbar.top2bottom = right_lightbar.bottom - right_lightbar.top;

    return std::make_pair(left_lightbar, right_lightbar);
}

Color Detector::get_armor_color(int class_id)
{
    // 根据类别ID设置颜色
    if (class_id >= 0 && class_id < 9)
    {
        return blue; // 蓝色装甲板 (0-8对应1-9号)
    }
    else if (class_id >= 9 && class_id < 18)
    {
        return red; // 红色装甲板 (9-17对应1-9号)
    }
    else
    {
        return purple; // 其他类别
    }
}
} // namespace armor_task