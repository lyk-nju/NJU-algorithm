#pragma once

#include "../armor_task/detector.hpp"

namespace buff_task
{
class BuffDetector
{
  public:
    BuffDetector() = default;
    explicit BuffDetector(const std::string &yolo_model_path)
        : detector_(yolo_model_path)
    {
    }

    armor_task::ArmorArray detect(const cv::Mat &frame)
    {
        return detector_.detect(frame);
    }

    bool usingTensorRt() const { return detector_.usingTensorRt(); }
    const char *backendName() const { return detector_.backendName(); }

  private:
    armor_task::Detector detector_;
};
}
