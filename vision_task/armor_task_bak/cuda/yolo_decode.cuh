#pragma once
#include <cuda_runtime.h>
#include <vector>

namespace armor_task {

struct DecodedBBox {
    int class_id;
    float confidence;
    float x, y, w, h;
    float kps[8];
};

void launch_decode_kernel(
    const float* device_output,
    std::vector<DecodedBBox>& output_boxes,
    float conf_thresh,
    float nms_thresh,
    cudaStream_t stream
);

} // namespace armor_task
