#pragma once

#include "../io/algorithm/ros2_transfer.hpp"
#include "../io/driver/cboard/cboard.hpp"
#include "../io/driver/camera/direct_image_source.hpp"
#include "../io/driver/ros2/ros2_image_source.hpp"
#include "../io/driver/ros2/ros2_manager.hpp"
#include "../io/interface/i_frame_source.hpp"
#include "../io/structs/structs.hpp"
#include "aim_pipeline.hpp"
#include "buff_task/buff_pipeline.hpp"

#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace armor_task
{

enum class ImageSourceType
{
    ROS2_TOPIC,
    DIRECT_CAMERA,
};

class AutoAimSystem
{
  public:
    AutoAimSystem(
        const std::string &yolo_model_path,
        const std::string &config_path,
        double bullet_speed,
        ImageSourceType source_type);
    ~AutoAimSystem();

    void start();
    void run();
    void stop();

    void updateData(const io::JudgerData &judger_data);

    io::Vision2Cboard processFrame(
        const cv::Mat &img,
        const Eigen::Quaterniond &quat,
        std::chrono::steady_clock::time_point image_timestamp);

    io::Vision2Cboard processBuffFrame(
        const cv::Mat &img,
        const Eigen::Quaterniond &quat,
        std::chrono::steady_clock::time_point image_timestamp);

    bool usingTensorRt() const { return pipeline_.usingTensorRt(); }
    const char *backendName() const { return pipeline_.backendName(); }
    const PnpSolver &getPnpSolver() const { return pipeline_.pnp_solver(); }

  private:
    std::string yolo_model_path_;
    std::string config_path_;
    ImageSourceType source_type_;

    AimPipeline pipeline_;
    buff_task::BuffPipeline buff_pipeline_;

    io::CameraInfo camera_info_;

    double bullet_speed_;
    bool enemy_is_red_ = true;

    std::shared_ptr<ROS2Manager> ros_node_;
    std::unique_ptr<io::IFrameSource> image_source_;
    std::unique_ptr<io::Cboard> cboard_owner_;
};

} // namespace armor_task
