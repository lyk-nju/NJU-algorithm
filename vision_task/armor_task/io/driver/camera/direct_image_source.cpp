#include "direct_image_source.hpp"

namespace io
{

DirectImageSource::DirectImageSource(const std::string &config_path) : camera_(config_path) {}

bool DirectImageSource::read(Frame &out)
{
    cv::Mat img;
    std::chrono::steady_clock::time_point ts;
    camera_.read(img, ts);
    if (img.empty()) return false;

    out.image = std::move(img);
    out.timestamp = ts;
    out.camera_id = camera_id_;
    out.id = ++frame_counter_;
    return true;
}

} // namespace io
