#include "ros2_image_source.hpp"

namespace io
{

Ros2ImageSource::Ros2ImageSource(std::shared_ptr<ROS2Manager> node) : node_(std::move(node)) {}

bool Ros2ImageSource::read(Frame &out)
{
    std::shared_ptr<const ROS2Manager::FramePacket> packet;
    if (!node_ || !node_->get_frame_packet(packet) || !packet || packet->frame.empty())
    {
        return false;
    }
    out.image = packet->frame;
    out.timestamp = packet->timestamp;
    out.camera_id = camera_id_;
    out.id = ++frame_counter_;
    return true;
}

} // namespace io
