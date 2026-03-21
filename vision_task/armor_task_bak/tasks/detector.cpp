#include "detector.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <fstream>
#include <opencv2/dnn.hpp>

namespace armor_task
{
Detector::Detector(const std::string &yolo_model_path) : input_width_(640), input_height_(640), confidence_threshold_(0.10f), nms_threshold_(0.5f), scale_(1.0f), pad_x_(0.0f), pad_y_(0.0f)
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
        for (int i = 0; i < nbIOTensors; ++i) {
            const char* name = engine_->getIOTensorName(i);
            nvinfer1::Dims dims = engine_->getTensorShape(name);
            nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(name);
            
            size_t vol = 1;
            for(int j=0; j<dims.nbDims; j++) vol *= (dims.d[j] < 0 ? 1 : dims.d[j]);
            
            void* ptr;
            cudaMalloc(&ptr, vol * sizeof(float));
            
            if (mode == nvinfer1::TensorIOMode::kINPUT) {
                input_name_ = name;
                device_input_buffer_ = ptr;
            } else {
                output_name_ = name;
                device_output_buffer_ = ptr;
                output_size_ = vol;
            }
        }
        cpu_output_buffer_.resize(output_size_);
        std::cout << "TensorRT engine loaded successfully." << std::endl;
        return;
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
    size_t max_image_size = 2560*1440*3;
    cudaMalloc((void**)&device_image_buffer_,max_image_size);
    device_image_buffer_size_=max_image_size;
    
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
    // 模块一：预处理
    cv::Mat input_blob = preprocess(frame, input_width_, input_height_);

    // 模块二：搜索装甲板
    return search_armors(frame, cv::Mat());
}

// cv::Mat Detector::preprocess(const cv::Mat &input_image, int target_width, int target_height)
// {
   
//     // 确保输入图像是有效的
//     if (input_image.empty())
//     {
//         std::cerr << "Error: Empty input image for preprocessing" << std::endl;
//         return cv::Mat();
//     }

//     // 与训练时一致的 letterbox 预处理（保持长宽比，四周 zero padding）
//     const int src_w = input_image.cols;
//     const int src_h = input_image.rows;

//     // 缩放比例：短边对齐到 target，长边按比例缩放
//     float r_w = static_cast<float>(target_width) / static_cast<float>(src_w);
//     float r_h = static_cast<float>(target_height) / static_cast<float>(src_h);
//     scale_ = std::min(r_w, r_h);

//     int new_unpad_w = static_cast<int>(std::round(src_w * scale_));
//     int new_unpad_h = static_cast<int>(std::round(src_h * scale_));

//     pad_x_ = (target_width - new_unpad_w) * 0.5f;
//     pad_y_ = (target_height - new_unpad_h) * 0.5f;

//     // 图像缩放到 new_unpad，并复制到 640x640 的黑色背景上
//     cv::Mat resized;
//     cv::resize(input_image, resized, cv::Size(new_unpad_w, new_unpad_h));

//     cv::Mat padded(target_height, target_width, input_image.type(), cv::Scalar(0,0,0));
//     cv::Rect roi(static_cast<int>(pad_x_), static_cast<int>(pad_y_), new_unpad_w, new_unpad_h);
//     resized.copyTo(padded(roi));
    
//     // 创建输入 blob: 归一化 + 通道交换
//     cv::Mat blob;
//     cv::dnn::blobFromImage(padded, blob, 1.0f / 255.0f, cv::Size(target_width, target_height), cv::Scalar(0, 0, 0), true, false, CV_32F);
    
    
//     return blob;
// }
cv::Mat Detector::preprocess(const cv::Mat &input_image, int target_width, int target_height)
{
    if (input_image.empty()) return cv::Mat();

    // 1. 懒加载：分配 GPU 内存用于存原始图片
    // 计算需要的字节数 (Height * Step)
    size_t img_size = input_image.rows * input_image.step;
    
    // if (device_image_buffer_ == nullptr || img_size > device_image_buffer_size_) {
    //     if (device_image_buffer_) cudaFree(device_image_buffer_);
    //     cudaMalloc((void**)&device_image_buffer_, img_size);
    //     device_image_buffer_size_ = img_size;
    // }
    if (device_image_buffer_ == nullptr || img_size > device_image_buffer_size_) {
        size_t new_size = (device_image_buffer_size_ == 0) ? img_size : device_image_buffer_size_;
        while (new_size < img_size) {
            new_size *= 2;
        }
        if (device_image_buffer_) cudaFree(device_image_buffer_);
        cudaMalloc((void**)&device_image_buffer_, new_size);
        device_image_buffer_size_ = new_size;
        std::cerr << "[Detector] Resize image buffer to " << device_image_buffer_size_ << " bytes" << std::endl;
    }


    // 2. Host -> Device (拷贝原始 uint8 图片)
    // 这是整个预处理中唯一的 CPU->GPU 拷贝，非常快
    cudaMemcpyAsync(device_image_buffer_, input_image.data, img_size, cudaMemcpyHostToDevice, stream_);

    // 3. 调用 CUDA Kernel 进行所有预处理
    // 结果直接写入 device_input_buffer_ (这是 TensorRT 的输入指针)
    launch_preprocess_kernel(
        device_image_buffer_, input_image.cols, input_image.rows, input_image.step,
        (float*)device_input_buffer_, target_width, target_height,
        stream_
    );
    
    // 更新成员变量供后处理使用 (坐标还原需要)
    // 这部分计算非常快，CPU做就行
    float r_w = (float)target_width / input_image.cols;
    float r_h = (float)target_height / input_image.rows;
    scale_ = std::min(r_w, r_h);
    int new_unpad_w = input_image.cols * scale_;
    int new_unpad_h = input_image.rows * scale_;
    pad_x_ = (target_width - new_unpad_w) / 2.0f;
    pad_y_ = (target_height - new_unpad_h) / 2.0f;

    // 返回空矩阵，告诉调用者不用管 CPU 的 blob 了
    return cv::Mat(); 
}

// ArmorArray Detector::search_armors(const cv::Mat &frame, const cv::Mat &input_blob)
// {
    
//     ArmorArray detected_armors;
//     std::vector<cv::Mat> outputs;

//     if (is_trt_)
//     {
    
//         cudaMemcpyAsync(device_input_buffer_, input_blob.ptr<float>(), input_blob.total() * sizeof(float), cudaMemcpyHostToDevice, stream_);
        
        
//         context_->setTensorAddress(input_name_.c_str(), device_input_buffer_);
//         context_->setTensorAddress(output_name_.c_str(), device_output_buffer_);
        
        
//         context_->enqueueV3(stream_);
         
        
//         cudaMemcpyAsync(cpu_output_buffer_.data(), device_output_buffer_, output_size_ * sizeof(float), cudaMemcpyDeviceToHost, stream_);
//         cudaStreamSynchronize(stream_);
       
        

	
//         nvinfer1::Dims outDims = engine_->getTensorShape(output_name_.c_str());
//         std::vector<int> sizes;
//         for(int i=0; i<outDims.nbDims; i++) sizes.push_back(outDims.d[i]);
        
        
//         cv::Mat outMat(sizes, CV_32F, cpu_output_buffer_.data());
//         outputs.push_back(outMat.clone());
         
        
       
//     }
//     else
//     {
//         // 设置输入并运行网络
//         yolo_net_.setInput(input_blob);

//         try
//         {
//             // 获取输出层名称
//             std::vector<std::string> outNames = yolo_net_.getUnconnectedOutLayersNames();
//             yolo_net_.forward(outputs, outNames);
//         }
//         catch (const cv::Exception &e)
//         {
//             std::cerr << "Error during DNN forward pass: " << e.what() << std::endl;
//             return detected_armors;
//         }
//     }

     

//     if (outputs.empty())
//     {
//         std::cerr << "Error: No outputs from the network." << std::endl;
//         return detected_armors;
//     }

//     const cv::Mat &detection_output = outputs[0];

//     if (detection_output.empty())
//     {
//         std::cerr << "Error: Empty detection output." << std::endl;
//         return detected_armors;
//     }

 
//     // 根据输出张量的维度进行不同的处理
//     if (detection_output.dims == 3)
//     {
//         int rows = detection_output.size[1];
//         int cols = detection_output.size[2];

//         std::vector<int> class_ids;
//         std::vector<float> confidences;
//         std::vector<cv::Rect> boxes;
//         std::vector<std::vector<cv::Point2f>> armor_corners;

//         cv::Mat transposed;
//         cv::Mat reshaped = detection_output.reshape(1, rows * cols);
//         cv::transpose(reshaped.reshape(1, rows), transposed);

//         for (int i = 0; i < cols; i++)
//         {

//             cv::Mat scores = transposed.row(i).colRange(4, 40); // 4-39是36个类别置信度

//             cv::Point class_id_point;
//             double max_class_score;
//             cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id_point);

//             // 只保留高置信度的检测结果
//             if (max_class_score > confidence_threshold_)
//             {
//                 // 获取边界框坐标
//                 float cx = transposed.at<float>(i, 0); // 中心x
//                 float cy = transposed.at<float>(i, 1); // 中心y
//                 float w = transposed.at<float>(i, 2);  // 宽度
//                 float h = transposed.at<float>(i, 3);  // 高度

//                 // 获取装甲板四个关键点（网络输出坐标基于 640x640 letterbox 图像）
//                 std::vector<cv::Point2f> corners(4);
//                 for (int j = 0; j < 4; j++)
//                 {
//                     float kx = transposed.at<float>(i, 40 + j * 2);     // 关键点 x（letterbox 坐标）
//                     float ky = transposed.at<float>(i, 40 + j * 2 + 1); // 关键点 y

//                     // 反变换：去除 padding，再除以缩放系数，恢复到原始图像坐标系
//                     float x_orig = (kx - pad_x_) / scale_;
//                     float y_orig = (ky - pad_y_) / scale_;

//                     corners[j].x = x_orig;
//                     corners[j].y = y_orig;
//                 }

//                 // bbox 同样在 letterbox 坐标系下，需要反变换回原图
//                 float left_l = cx - 0.5f * w;
//                 float top_l = cy - 0.5f * h;

//                 float left = (left_l - pad_x_) / scale_;
//                 float top = (top_l - pad_y_) / scale_;
//                 float width = w / scale_;
//                 float height = h / scale_;

//                 // 裁剪到图像范围，避免越界
//                 left = std::max(0.0f, std::min(left, static_cast<float>(frame.cols - 1)));
//                 top = std::max(0.0f, std::min(top, static_cast<float>(frame.rows - 1)));
//                 width = std::max(1.0f, std::min(width, static_cast<float>(frame.cols) - left));
//                 height = std::max(1.0f, std::min(height, static_cast<float>(frame.rows) - top));

//                 boxes.push_back(cv::Rect(static_cast<int>(left), static_cast<int>(top), static_cast<int>(width), static_cast<int>(height)));
//                 class_ids.push_back(class_id_point.x); // x 是列索引
//                 confidences.push_back(static_cast<float>(max_class_score));
//                 armor_corners.push_back(corners);
//             }
//         }

//         std::vector<int> indices;
//         cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, indices);

//         // 为装甲板分配ID并创建对象
//         int armor_id = 0;
//         for (size_t i = 0; i < indices.size(); ++i)
//         {
//             int idx = indices[i];
//             cv::Rect box = boxes[idx];
//             int class_id = class_ids[idx];
//             float conf = confidences[idx];
//             std::vector<cv::Point2f> corners = armor_corners[idx];

//             // 确保类别ID在范围内且边界框有效
//             if (class_id < static_cast<int>(class_names_.size()) && box.width > 0 && box.height > 0 && box.x >= 0 && box.y >= 0 && box.x + box.width < frame.cols && box.y + box.height < frame.rows)
//             {
//                 // 从角点提取灯条信息
//                 auto lightbars = extract_lightbars_from_corners(corners);

//                 // 创建装甲板对象
//                 Armor armor(lightbars.first, lightbars.second, armor_id++, box);
//                 armor.confidence = conf;
//                 armor.color = get_armor_color(class_id);
//                 armor.corners = corners; // 保存角点信息

//                 // 从类别ID提取数字信息
//                 if (class_id >= 0 && class_id < 18)
//                 { // 只有装甲板类别才有数字
//                     if (class_id < 9)
//                     {
//                         armor.car_num = class_id; // 蓝色装甲板 1-9
//                     }
//                     else
//                     {
//                         armor.car_num = (class_id - 9); // 红色装甲板 1-9
//                     }
//                 }
//                 else
//                 {
//                     armor.car_num = -1; // 非装甲板
//                 }

//                 detected_armors.push_back(armor);
//             }
//         }
//     }


//     return detected_armors;
// }
ArmorArray Detector::search_armors(const cv::Mat &frame, const cv::Mat &input_blob)
{
    ArmorArray detected_armors;

    // 1. 推理 (Inference)
    if (is_trt_)
    {
        // Host -> Device
        //cudaMemcpyAsync(device_input_buffer_, input_blob.ptr<float>(), input_blob.total() * sizeof(float), cudaMemcpyHostToDevice, stream_);
        
        // 推理
        context_->setTensorAddress(input_name_.c_str(), device_input_buffer_);
        context_->setTensorAddress(output_name_.c_str(), device_output_buffer_);
        context_->enqueueV3(stream_);
        
        // Device -> Host
        // cudaMemcpyAsync(cpu_output_buffer_.data(), device_output_buffer_, output_size_ * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        // cudaStreamSynchronize(stream_);
    }
    else
    {
        // 保持原有的 OpenCV 推理逻辑作为 fallback
        std::vector<cv::Mat> outputs;
        yolo_net_.setInput(input_blob);
        yolo_net_.forward(outputs, yolo_net_.getUnconnectedOutLayersNames());
        if (outputs.empty()) return detected_armors;
        // 将 OpenCV 输出的数据拷贝到 cpu_output_buffer_ 以统一处理逻辑
        // 注意：此处仅为兼容代码，实际使用建议全程 TensorRT
        float* data = (float*)outputs[0].data;
        size_t size = outputs[0].total();
        cpu_output_buffer_.assign(data, data + size);
    }

    // 2. 后处理 (Post-processing) - 核心优化
    // 假设输出形状: [Batch, Channels, Anchors] -> [1, 48, 8400]
    // 内存布局: Channel 0 的 8400 个数, Channel 1 的 8400 个数...
    std::vector<DecodedBBox> decoded_boxes;
    launch_decode_kernel((float*)device_output_buffer_, decoded_boxes, confidence_threshold_, nms_threshold_, stream_);
    constexpr size_t kTopKDetections = 128;
    if (decoded_boxes.size() > kTopKDetections)
    {
        std::nth_element(decoded_boxes.begin(), decoded_boxes.begin() + kTopKDetections, decoded_boxes.end(),
                         [](const DecodedBBox &a, const DecodedBBox &b) { return a.confidence > b.confidence; });
        decoded_boxes.resize(kTopKDetections);
    }
    std::sort(decoded_boxes.begin(), decoded_boxes.end(), [](const DecodedBBox &a, const DecodedBBox &b) { return a.confidence > b.confidence; });
    


    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> armor_corners;
    class_ids.reserve(decoded_boxes.size());
    confidences.reserve(decoded_boxes.size());
    boxes.reserve(decoded_boxes.size());
    armor_corners.reserve(decoded_boxes.size());
    for(const auto&db :decoded_boxes){
        float left = (db.x- 0.5f * db.w - pad_x_) / scale_;
        float top  = (db.y - 0.5f * db.h - pad_y_) / scale_;
        float width = db.w / scale_;
        float height = db.h / scale_;
        //std::cout<<"armor-info"<<left<<","<<top<<","<<width<<","<<height<<","<<db.confidence<<std::endl;
        //std::cout<<"armor-confidence"<<db.confidence<<std::endl;

            // 还原关键点
        std::vector<cv::Point2f> corners(4);
        //int class_id=db.class_id-1;
        for (int k = 0; k < 4; ++k)
        {
            corners[k].x = (db.kps[k*2] - pad_x_) / scale_;
            corners[k].y = (db.kps[k*2+1] - pad_y_) / scale_;
            }
        //std::cout<<corners<<std::endl;
        boxes.push_back(cv::Rect(left, top, width, height));
        class_ids.push_back(db.class_id);
        confidences.push_back(db.confidence);
        armor_corners.push_back(corners);
    }
    // 3. 封装结果（NMS 已在 GPU 完成）
    int id_counter = 0;
    for (size_t idx = 0; idx < boxes.size(); ++idx)
    {
        // 边界检查（放在最后做，因为 NMS 之后框很少，检查开销小）
        cv::Rect box = boxes[idx];
        if (box.x < 0 || box.y < 0 || box.x + box.width >= frame.cols || box.y + box.height >= frame.rows)
            continue;

        int class_id = class_ids[idx];
        // 这里的逻辑和你原代码保持一致
        auto lightbars = extract_lightbars_from_corners(armor_corners[idx]);
        Armor armor(lightbars.first, lightbars.second, id_counter++, box);
        armor.confidence = confidences[idx];
        armor.color = get_armor_color(class_id);
        armor.corners = armor_corners[idx];
        
        if (class_id >= 0 && class_id < 18) {
             armor.car_num = (class_id < 9) ? class_id : (class_id - 9); // 1-9
             // 注意：这里需要确认你的 car_num 是存 1-9 还是 0-8，你的原代码逻辑有点混
             // 原代码: class_id 0->1, 8->9. 
             //armor.car_num += 1; // 如果你的 Armor 结构体定义 car_num 1 就是数字1
        } else {
             armor.car_num = -1;
        }
       
        
        detected_armors.push_back(armor);
    }

    return detected_armors;
}
// ArmorArray Detector::search_armors(const cv::Mat &frame, const cv::Mat &input_blob)
// {
//     ArmorArray detected_armors;

//     // 1. 推理 (Inference)
//     if (is_trt_)
//     {
//         // Host -> Device
//         cudaMemcpyAsync(device_input_buffer_, input_blob.ptr<float>(), input_blob.total() * sizeof(float), cudaMemcpyHostToDevice, stream_);
        
//         // 推理
//         context_->setTensorAddress(input_name_.c_str(), device_input_buffer_);
//         context_->setTensorAddress(output_name_.c_str(), device_output_buffer_);
//         context_->enqueueV3(stream_);
        
//         // Device -> Host
//         cudaMemcpyAsync(cpu_output_buffer_.data(), device_output_buffer_, output_size_ * sizeof(float), cudaMemcpyDeviceToHost, stream_);
//         cudaStreamSynchronize(stream_);
//     }
//     else
//     {
//         // 保持原有的 OpenCV 推理逻辑作为 fallback
//         std::vector<cv::Mat> outputs;
//         yolo_net_.setInput(input_blob);
//         yolo_net_.forward(outputs, yolo_net_.getUnconnectedOutLayersNames());
//         if (outputs.empty()) return detected_armors;
//         // 将 OpenCV 输出的数据拷贝到 cpu_output_buffer_ 以统一处理逻辑
//         // 注意：此处仅为兼容代码，实际使用建议全程 TensorRT
//         float* data = (float*)outputs[0].data;
//         size_t size = outputs[0].total();
//         cpu_output_buffer_.assign(data, data + size);
//     }

//     // 2. 后处理 (Post-processing) - 核心优化
//     // 假设输出形状: [Batch, Channels, Anchors] -> [1, 48, 8400]
//     // 内存布局: Channel 0 的 8400 个数, Channel 1 的 8400 个数...
//     
    
//     float* output_data = cpu_output_buffer_.data();
//     int num_channels = 48; // 4(bbox) + 36(cls) + 8(landmarks)
//     int num_anchors = 8400;
//     int stride = num_anchors; // 跳到下一个 Channel 需要的步长

//     std::vector<int> class_ids;
//     std::vector<float> confidences;
//     std::vector<cv::Rect> boxes;
//     std::vector<std::vector<cv::Point2f>> armor_corners;

//     // 预分配内存，避免 vector 扩容开销
//     boxes.reserve(100);
//     class_ids.reserve(100);
//     confidences.reserve(100);
//     armor_corners.reserve(100);

//     // 指针预计算
//     // 0:x, 1:y, 2:w, 3:h
//     float* p_x = output_data + 0 * stride;
//     float* p_y = output_data + 1 * stride;
//     float* p_w = output_data + 2 * stride;
//     float* p_h = output_data + 3 * stride;
//     // 4~39: classes
//     float* p_classes = output_data + 4 * stride;
//     // 40~47: landmarks
//     float* p_landmarks = output_data + 40 * stride;

//     for (int i = 0; i < num_anchors; ++i)
//     {
//         // --- 步骤 2.1: 快速找到最大置信度 ---
//         // 也就是原来的 cv::minMaxLoc，现在用纯指针循环
//         float max_score = -1.0f;
//         int max_class_id = -1;

//         // 遍历 36 个类别
//         for (int c = 0; c < 36; ++c)
//         {
//             // 访问第 c 个类别的第 i 个 Anchor
//             // p_classes 指向第4行(第一个类别行)，加上 c*stride 跳到对应类别行
//             float score = p_classes[c * stride + i]; 
//             if (score > max_score)
//             {
//                 max_score = score;
//                 max_class_id = c;
//             }
//         }

//         // --- 步骤 2.2: 阈值筛选 ---
//         if (max_score > confidence_threshold_)
//         {
//             // 只有当置信度足够高时，才去计算复杂的坐标，节省算力
//             float cx = p_x[i];
//             float cy = p_y[i];
//             float w = p_w[i];
//             float h = p_h[i];

//             // 还原 BBox (反变换)
//             float left = (cx - 0.5f * w - pad_x_) / scale_;
//             float top  = (cy - 0.5f * h - pad_y_) / scale_;
//             float width = w / scale_;
//             float height = h / scale_;

//             // 还原关键点
//             std::vector<cv::Point2f> corners(4);
//             for (int k = 0; k < 4; ++k)
//             {
//                 float kx = p_landmarks[(k * 2) * stride + i];
//                 float ky = p_landmarks[(k * 2 + 1) * stride + i];
//                 corners[k].x = (kx - pad_x_) / scale_;
//                 corners[k].y = (ky - pad_y_) / scale_;
//             }

//             boxes.push_back(cv::Rect(left, top, width, height));
//             class_ids.push_back(max_class_id);
//             confidences.push_back(max_score);
//             armor_corners.push_back(corners);
//         }
//     }

//     // 3. NMS (非极大值抑制)
//     std::vector<int> indices;
//     cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, indices);

//     // 4. 封装结果
//     int id_counter = 0;
//     for (int idx : indices)
//     {
//         // 边界检查（放在最后做，因为 NMS 之后框很少，检查开销小）
//         cv::Rect box = boxes[idx];
//         if (box.x < 0 || box.y < 0 || box.x + box.width >= frame.cols || box.y + box.height >= frame.rows)
//             continue;

//         int class_id = class_ids[idx];
//         // 这里的逻辑和你原代码保持一致
//         auto lightbars = extract_lightbars_from_corners(armor_corners[idx]);
//         Armor armor(lightbars.first, lightbars.second, id_counter++, box);
//         armor.confidence = confidences[idx];
//         armor.color = get_armor_color(class_id);
//         armor.corners = armor_corners[idx];
        
//         if (class_id >= 0 && class_id < 18) {
//              armor.car_num = (class_id < 9) ? class_id : (class_id - 9); // 1-9
//              // 注意：这里需要确认你的 car_num 是存 1-9 还是 0-8，你的原代码逻辑有点混
//              // 原代码: class_id 0->1, 8->9. 
//              armor.car_num += 1; // 如果你的 Armor 结构体定义 car_num 1 就是数字1
//         } else {
//              armor.car_num = -1;
//         }
        
//         detected_armors.push_back(armor);
//     }

//     return detected_armors;
// }
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
