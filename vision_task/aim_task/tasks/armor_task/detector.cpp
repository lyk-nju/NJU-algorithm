#include "detector.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <opencv2/dnn.hpp>

namespace armor_task
{
Detector::Detector() : Detector("../models/best.engine") {}

Detector::Detector(const std::string &yolo_model_path)
    : input_width_(640),
      input_height_(640),
      confidence_threshold_(0.55f),
      nms_threshold_(0.5f),
      scale_(1.0f),
      pad_x_(0.0f),
      pad_y_(0.0f)
{
    // 36 个类别：blue_1..9, red_1..9, other_1..18
    class_names_.resize(36);
    for (int i = 1; i <= 9; i++)
    {
        class_names_[i - 1] = "blue_" + std::to_string(i);
    }
    for (int i = 10; i <= 18; i++)
    {
        class_names_[i - 1] = "red_" + std::to_string(i - 9);
    }
    for (int i = 19; i <= 36; i++)
    {
        class_names_[i - 1] = "other_" + std::to_string(i - 18);
    }

    if (yolo_model_path.find(".engine") != std::string::npos)
    {
        is_trt_ = true;
        std::cout << "Loading TensorRT engine: " << yolo_model_path << std::endl;
        std::ifstream file(yolo_model_path, std::ios::binary);
        if (!file.good())
        {
            std::cerr << "Read engine fail: " << yolo_model_path << std::endl;
            throw std::runtime_error("Read engine fail");
        }

        file.seekg(0, file.end);
        size_t size = file.tellg();
        file.seekg(0, file.beg);
        std::vector<char> engineData(size);
        file.read(engineData.data(), size);

        runtime_ = nvinfer1::createInferRuntime(logger_);
        engine_ = runtime_->deserializeCudaEngine(engineData.data(), size);
        context_ = engine_->createExecutionContext();

        cudaStreamCreate(&stream_);

        int nbIOTensors = engine_->getNbIOTensors();
        for (int i = 0; i < nbIOTensors; ++i)
        {
            const char *name = engine_->getIOTensorName(i);
            nvinfer1::Dims dims = engine_->getTensorShape(name);
            nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(name);

            size_t vol = 1;
            for (int j = 0; j < dims.nbDims; j++) vol *= (dims.d[j] < 0 ? 1 : dims.d[j]);

            void *ptr;
            cudaMalloc(&ptr, vol * sizeof(float));

            if (mode == nvinfer1::TensorIOMode::kINPUT)
            {
                input_name_ = name;
                device_input_buffer_ = ptr;
            }
            else
            {
                output_name_ = name;
                device_output_buffer_ = ptr;
                output_size_ = vol;
            }
        }
        cpu_output_buffer_.resize(output_size_);
        std::cout << "TensorRT engine loaded successfully." << std::endl;
        return;
    }

    // OpenCV DNN (ONNX) 路径
    yolo_net_ = cv::dnn::readNet(yolo_model_path);
    if (yolo_net_.empty())
    {
        std::cerr << "Error: Could not load YOLO model from: " << yolo_model_path << std::endl;
        throw std::runtime_error("Failed to load YOLO model.");
    }
    std::cout << "YOLO model loaded successfully from: " << yolo_model_path << std::endl;

    try
    {
        yolo_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        yolo_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        std::cout << "YOLO model configured for CUDA GPU acceleration (FP32)." << std::endl;
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "Fatal: CUDA setting failed. Please check your OpenCV build and CUDA environment. "
                  << "Error: " << e.what() << std::endl;
        throw;
    }

    // 预分配一块 GPU buffer 用于原始 BGR 图像的 H2D 搬运（仅 TRT 路径使用，这里保留以便后续切换）
    constexpr size_t kMaxImageSize = 2560 * 1440 * 3;
    cudaMalloc(reinterpret_cast<void **>(&device_image_buffer_), kMaxImageSize);
    device_image_buffer_size_ = kMaxImageSize;
}

Detector::~Detector()
{
    if (is_trt_)
    {
        if (device_input_buffer_) cudaFree(device_input_buffer_);
        if (device_output_buffer_) cudaFree(device_output_buffer_);
        if (device_image_buffer_) cudaFree(device_image_buffer_);
        if (stream_) cudaStreamDestroy(stream_);
        delete context_;
        delete engine_;
        delete runtime_;
    }
}

ArmorArray Detector::detect(const cv::Mat &frame)
{
    cv::Mat input_blob = preprocess(frame, input_width_, input_height_);
    return search_armors(frame, input_blob);
}

cv::Mat Detector::preprocess(const cv::Mat &input_image, int target_width, int target_height)
{
    if (input_image.empty()) return cv::Mat();

    if (!is_trt_)
    {
        // letterbox：保持长宽比 + 四周 zero padding，与训练时对齐
        const int src_w = input_image.cols;
        const int src_h = input_image.rows;
        const float r_w = static_cast<float>(target_width) / static_cast<float>(src_w);
        const float r_h = static_cast<float>(target_height) / static_cast<float>(src_h);
        scale_ = std::min(r_w, r_h);

        const int new_unpad_w = static_cast<int>(std::round(src_w * scale_));
        const int new_unpad_h = static_cast<int>(std::round(src_h * scale_));
        pad_x_ = (target_width - new_unpad_w) * 0.5f;
        pad_y_ = (target_height - new_unpad_h) * 0.5f;

        cv::Mat resized;
        cv::resize(input_image, resized, cv::Size(new_unpad_w, new_unpad_h));
        cv::Mat padded(target_height, target_width, input_image.type(), cv::Scalar(0, 0, 0));
        const cv::Rect roi(static_cast<int>(pad_x_), static_cast<int>(pad_y_), new_unpad_w, new_unpad_h);
        resized.copyTo(padded(roi));

        cv::Mat blob;
        cv::dnn::blobFromImage(
            padded, blob, 1.0f / 255.0f, cv::Size(target_width, target_height), cv::Scalar(0, 0, 0), true, false, CV_32F);
        return blob;
    }

    // TRT 路径：预处理完全在 GPU 上做，只做一次 H2D 拷贝原图
    const size_t img_size = input_image.rows * input_image.step;
    if (device_image_buffer_ == nullptr || img_size > device_image_buffer_size_)
    {
        size_t new_size = (device_image_buffer_size_ == 0) ? img_size : device_image_buffer_size_;
        while (new_size < img_size) new_size *= 2;
        if (device_image_buffer_) cudaFree(device_image_buffer_);
        cudaMalloc(reinterpret_cast<void **>(&device_image_buffer_), new_size);
        device_image_buffer_size_ = new_size;
        std::cerr << "[Detector] Resize image buffer to " << device_image_buffer_size_ << " bytes" << std::endl;
    }

    cudaMemcpyAsync(device_image_buffer_, input_image.data, img_size, cudaMemcpyHostToDevice, stream_);

    launch_preprocess_kernel(
        device_image_buffer_, input_image.cols, input_image.rows, input_image.step,
        reinterpret_cast<float *>(device_input_buffer_), target_width, target_height,
        stream_);

    // 坐标还原需要：与 kernel 内部保持一致的 scale/pad 计算（TRT kernel 采用截断，不用 round）
    const float r_w = static_cast<float>(target_width) / input_image.cols;
    const float r_h = static_cast<float>(target_height) / input_image.rows;
    scale_ = std::min(r_w, r_h);
    const int new_unpad_w = static_cast<int>(input_image.cols * scale_);
    const int new_unpad_h = static_cast<int>(input_image.rows * scale_);
    pad_x_ = (target_width - new_unpad_w) / 2.0f;
    pad_y_ = (target_height - new_unpad_h) / 2.0f;

    return cv::Mat();
}

ArmorArray Detector::search_armors(const cv::Mat &frame, const cv::Mat &input_blob)
{
    ArmorArray detected_armors;

    // 推理
    if (is_trt_)
    {
        context_->setTensorAddress(input_name_.c_str(), device_input_buffer_);
        context_->setTensorAddress(output_name_.c_str(), device_output_buffer_);
        context_->enqueueV3(stream_);
    }
    else
    {
        // OpenCV DNN ONNX 路径：CPU 后处理
        std::vector<cv::Mat> outputs;
        yolo_net_.setInput(input_blob);
        try
        {
            yolo_net_.forward(outputs, yolo_net_.getUnconnectedOutLayersNames());
        }
        catch (const cv::Exception &e)
        {
            std::cerr << "Error during DNN forward pass: " << e.what() << std::endl;
            return detected_armors;
        }
        if (outputs.empty() || outputs[0].empty())
        {
            return detected_armors;
        }

        const cv::Mat &detection_output = outputs[0];
        if (detection_output.dims != 3)
        {
            std::cerr << "Error: Unsupported ONNX output dims = " << detection_output.dims << std::endl;
            return detected_armors;
        }

        const int rows = detection_output.size[1];
        const int cols = detection_output.size[2];
        cv::Mat reshaped = detection_output.reshape(1, rows * cols);
        cv::Mat transposed;
        cv::transpose(reshaped.reshape(1, rows), transposed);

        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        std::vector<std::vector<cv::Point2f>> armor_corners;
        class_ids.reserve(cols);
        confidences.reserve(cols);
        boxes.reserve(cols);
        armor_corners.reserve(cols);

        for (int i = 0; i < cols; i++)
        {
            cv::Mat scores = transposed.row(i).colRange(4, 40); // 36 类
            cv::Point class_id_point;
            double max_class_score = 0.0;
            cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id_point);
            if (max_class_score <= confidence_threshold_) continue;

            const float cx = transposed.at<float>(i, 0);
            const float cy = transposed.at<float>(i, 1);
            const float w = transposed.at<float>(i, 2);
            const float h = transposed.at<float>(i, 3);

            float left = (cx - 0.5f * w - pad_x_) / scale_;
            float top = (cy - 0.5f * h - pad_y_) / scale_;
            float width = w / scale_;
            float height = h / scale_;
            left = std::max(0.0f, std::min(left, static_cast<float>(frame.cols - 1)));
            top = std::max(0.0f, std::min(top, static_cast<float>(frame.rows - 1)));
            width = std::max(1.0f, std::min(width, static_cast<float>(frame.cols) - left));
            height = std::max(1.0f, std::min(height, static_cast<float>(frame.rows) - top));

            std::vector<cv::Point2f> corners(4);
            for (int j = 0; j < 4; j++)
            {
                const float kx = transposed.at<float>(i, 40 + j * 2);
                const float ky = transposed.at<float>(i, 40 + j * 2 + 1);
                corners[j].x = (kx - pad_x_) / scale_;
                corners[j].y = (ky - pad_y_) / scale_;
            }

            boxes.emplace_back(
                static_cast<int>(left), static_cast<int>(top), static_cast<int>(width), static_cast<int>(height));
            class_ids.push_back(class_id_point.x);
            confidences.push_back(static_cast<float>(max_class_score));
            armor_corners.push_back(corners);
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, indices);
        int armor_id = 0;
        for (int idx : indices)
        {
            const cv::Rect &box = boxes[idx];
            if (box.width <= 0 || box.height <= 0 || box.x < 0 || box.y < 0 ||
                box.x + box.width >= frame.cols || box.y + box.height >= frame.rows)
            {
                continue;
            }

            const int class_id = class_ids[idx];
            auto lightbars = extract_lightbars_from_corners(armor_corners[idx]);
            Armor armor(lightbars.first, lightbars.second, armor_id++, box);
            armor.confidence = confidences[idx];
            armor.color = get_armor_color(class_id);
            armor.corners = armor_corners[idx];
            armor.car_num = (class_id >= 0 && class_id < 18) ? ((class_id < 9) ? class_id : class_id - 9) : -1;
            detected_armors.push_back(armor);
        }

        return detected_armors;
    }

    // TRT 路径后处理：NMS + 解码由 CUDA kernel 完成，host 侧只做 top-K + sort + 边界检查
    std::vector<DecodedBBox> decoded_boxes;
    launch_decode_kernel(
        reinterpret_cast<float *>(device_output_buffer_), decoded_boxes, confidence_threshold_, nms_threshold_, stream_);

    constexpr size_t kTopKDetections = 128;
    if (decoded_boxes.size() > kTopKDetections)
    {
        std::nth_element(
            decoded_boxes.begin(), decoded_boxes.begin() + kTopKDetections, decoded_boxes.end(),
            [](const DecodedBBox &a, const DecodedBBox &b) { return a.confidence > b.confidence; });
        decoded_boxes.resize(kTopKDetections);
    }
    std::sort(decoded_boxes.begin(), decoded_boxes.end(),
              [](const DecodedBBox &a, const DecodedBBox &b) { return a.confidence > b.confidence; });

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> armor_corners;
    class_ids.reserve(decoded_boxes.size());
    confidences.reserve(decoded_boxes.size());
    boxes.reserve(decoded_boxes.size());
    armor_corners.reserve(decoded_boxes.size());

    for (const auto &db : decoded_boxes)
    {
        const float left = (db.x - 0.5f * db.w - pad_x_) / scale_;
        const float top = (db.y - 0.5f * db.h - pad_y_) / scale_;
        const float width = db.w / scale_;
        const float height = db.h / scale_;

        std::vector<cv::Point2f> corners(4);
        for (int k = 0; k < 4; ++k)
        {
            corners[k].x = (db.kps[k * 2] - pad_x_) / scale_;
            corners[k].y = (db.kps[k * 2 + 1] - pad_y_) / scale_;
        }

        boxes.emplace_back(static_cast<int>(left), static_cast<int>(top), static_cast<int>(width), static_cast<int>(height));
        class_ids.push_back(db.class_id);
        confidences.push_back(db.confidence);
        armor_corners.push_back(std::move(corners));
    }

    int id_counter = 0;
    for (size_t idx = 0; idx < boxes.size(); ++idx)
    {
        const cv::Rect &box = boxes[idx];
        if (box.x < 0 || box.y < 0 || box.x + box.width >= frame.cols || box.y + box.height >= frame.rows) continue;

        const int class_id = class_ids[idx];
        auto lightbars = extract_lightbars_from_corners(armor_corners[idx]);
        Armor armor(lightbars.first, lightbars.second, id_counter++, box);
        armor.confidence = confidences[idx];
        armor.color = get_armor_color(class_id);
        armor.corners = armor_corners[idx];
        armor.car_num = (class_id >= 0 && class_id < 18) ? ((class_id < 9) ? class_id : class_id - 9) : -1;
        detected_armors.push_back(armor);
    }

    if (!detected_armors.empty())
    {
        std::cout << "[Detector] Detected " << detected_armors.size() << " armor(s)" << std::endl;
        for (const auto &armor : detected_armors)
        {
            const char *color_str =
                (armor.color == blue) ? "blue" : (armor.color == red) ? "red" : "purple";
            std::cout << "  - Detect ID: " << armor.detect_id
                      << ", Car Num: " << armor.car_num
                      << ", Color: " << color_str
                      << ", Confidence: " << armor.confidence << std::endl;
        }
    }

    return detected_armors;
}

std::pair<LightBar, LightBar> Detector::extract_lightbars_from_corners(const std::vector<cv::Point2f> &corners)
{
    LightBar left_lightbar, right_lightbar;
    if (corners.size() != 4) return std::make_pair(left_lightbar, right_lightbar);

    // 按 x 排序找左右两对点，再按 y 确定上下
    std::vector<std::pair<cv::Point2f, int>> sorted_corners;
    sorted_corners.reserve(4);
    for (int i = 0; i < 4; i++) sorted_corners.push_back({corners[i], i});
    std::sort(sorted_corners.begin(), sorted_corners.end(),
              [](const auto &a, const auto &b) { return a.first.x < b.first.x; });

    cv::Point2f left1 = sorted_corners[0].first;
    cv::Point2f left2 = sorted_corners[1].first;
    cv::Point2f right1 = sorted_corners[2].first;
    cv::Point2f right2 = sorted_corners[3].first;

    if (left1.y > left2.y) std::swap(left1, left2);
    if (right1.y > right2.y) std::swap(right1, right2);

    left_lightbar.top = left1;
    left_lightbar.bottom = left2;
    left_lightbar.center = (left_lightbar.top + left_lightbar.bottom) * 0.5f;
    left_lightbar.top2bottom = left_lightbar.bottom - left_lightbar.top;

    right_lightbar.top = right1;
    right_lightbar.bottom = right2;
    right_lightbar.center = (right_lightbar.top + right_lightbar.bottom) * 0.5f;
    right_lightbar.top2bottom = right_lightbar.bottom - right_lightbar.top;

    return std::make_pair(left_lightbar, right_lightbar);
}

Color Detector::get_armor_color(int class_id)
{
    if (class_id >= 0 && class_id < 9) return blue;   // blue_1..9
    if (class_id >= 9 && class_id < 18) return red;   // red_1..9
    return purple;                                    // other_*
}

} // namespace armor_task
