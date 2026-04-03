#include "yolo_decode.cuh"
#include <algorithm>

#include <stdint.h>

namespace armor_task {

namespace {

constexpr int kClassCount = 36;
constexpr int kNumAnchors = 8400;
constexpr int kMaxBoxes = 1024;

DecodedBBox* d_boxes = nullptr;
DecodedBBox* d_kept_boxes = nullptr;
int* d_count = nullptr;
int* d_kept_count = nullptr;
uint8_t* d_keep_flags = nullptr;
DecodedBBox* h_boxes = nullptr;
bool is_init = false;

__device__ float box_iou(const DecodedBBox& a, const DecodedBBox& b) {
    const float a_left = a.x - 0.5f * a.w;
    const float a_top = a.y - 0.5f * a.h;
    const float a_right = a.x + 0.5f * a.w;
    const float a_bottom = a.y + 0.5f * a.h;

    const float b_left = b.x - 0.5f * b.w;
    const float b_top = b.y - 0.5f * b.h;
    const float b_right = b.x + 0.5f * b.w;
    const float b_bottom = b.y + 0.5f * b.h;

    const float inter_left = max(a_left, b_left);
    const float inter_top = max(a_top, b_top);
    const float inter_right = min(a_right, b_right);
    const float inter_bottom = min(a_bottom, b_bottom);
    const float inter_w = max(0.0f, inter_right - inter_left);
    const float inter_h = max(0.0f, inter_bottom - inter_top);
    const float inter_area = inter_w * inter_h;

    const float area_a = a.w * a.h;
    const float area_b = b.w * b.h;
    const float union_area = area_a + area_b - inter_area;
    if (union_area <= 0.0f) return 0.0f;
    return inter_area / union_area;
}

}  // namespace

__global__ void decode_kernel(
    const float* __restrict__ trt_output,
    DecodedBBox* output_boxes,
    int* count,
    int num_anchors,
    float conf_threshold,
    int max_output_size
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_anchors) return;

    int stride = num_anchors;

    float max_score = -1.0f;
    int max_class_id = -1;
    const float* classes_ptr = trt_output + 4 * stride;
    for (int c = 0; c < kClassCount; ++c) {
        float score = classes_ptr[c * stride + idx];
        if (score > max_score) {
            max_score = score;
            max_class_id = c;
        }
    }

    if (max_score > conf_threshold) {
        int write_idx = atomicAdd(count, 1);
        if (write_idx < max_output_size) {
            DecodedBBox& box = output_boxes[write_idx];
            box.class_id = max_class_id;
            box.confidence = max_score;
            box.x = trt_output[0 * stride + idx];
            box.y = trt_output[1 * stride + idx];
            box.w = trt_output[2 * stride + idx];
            box.h = trt_output[3 * stride + idx];

            const float* kps_ptr = trt_output + 40 * stride;
            for (int k = 0; k < 8; ++k) {
                box.kps[k] = kps_ptr[k * stride + idx];
            }
        }
    }
}

__global__ void nms_kernel(
    const DecodedBBox* decoded_boxes,
    int box_count,
    float nms_threshold,
    uint8_t* keep_flags
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= box_count) return;

    const DecodedBBox current = decoded_boxes[idx];
    uint8_t keep = 1;

    for (int other = 0; other < box_count; ++other) {
        if (other == idx) continue;
        const DecodedBBox candidate = decoded_boxes[other];
        if (candidate.class_id != current.class_id) continue;

        const bool higher_priority =
            (candidate.confidence > current.confidence) ||
            (candidate.confidence == current.confidence && other < idx);
        if (!higher_priority) continue;

        if (box_iou(current, candidate) > nms_threshold) {
            keep = 0;
            break;
        }
    }

    keep_flags[idx] = keep;
}

__global__ void compact_kernel(
    const DecodedBBox* decoded_boxes,
    const uint8_t* keep_flags,
    int box_count,
    DecodedBBox* kept_boxes,
    int* kept_count,
    int max_output_size
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= box_count || keep_flags[idx] == 0) return;

    int write_idx = atomicAdd(kept_count, 1);
    if (write_idx < max_output_size) {
        kept_boxes[write_idx] = decoded_boxes[idx];
    }
}

void launch_decode_kernel(
    const float* device_output,
    std::vector<DecodedBBox>& output_boxes,
    float conf_thresh,
    float nms_thresh,
    cudaStream_t stream
) {
    // last_count：用上一帧的实际框数作为本帧 NMS/compact 的上界，
    // 避免在 decode_kernel 之后插入中间同步来读取 count。
    static int last_count = kMaxBoxes;

    if (!is_init) {
        cudaMalloc(&d_boxes, kMaxBoxes * sizeof(DecodedBBox));
        cudaMalloc(&d_kept_boxes, kMaxBoxes * sizeof(DecodedBBox));
        cudaMalloc(&d_count, sizeof(int));
        cudaMalloc(&d_kept_count, sizeof(int));
        cudaMalloc(&d_keep_flags, kMaxBoxes * sizeof(uint8_t));
        cudaMallocHost(&h_boxes, kMaxBoxes * sizeof(DecodedBBox));
        is_init = true;
    }

    cudaMemsetAsync(d_count, 0, sizeof(int), stream);
    cudaMemsetAsync(d_kept_count, 0, sizeof(int), stream);
    // 清零 keep_flags，防止 last_count > cur_count 时旧帧残留 box 通过 NMS
    cudaMemsetAsync(d_keep_flags, 0, last_count * sizeof(uint8_t), stream);

    int threads = 256;
    int blocks = (kNumAnchors + threads - 1) / threads;
    decode_kernel<<<blocks, threads, 0, stream>>>(
        device_output, d_boxes, d_count, kNumAnchors, conf_thresh, kMaxBoxes
    );

    // 用上一帧 count 启动 NMS，无需中间 sync
    const int nms_blocks = (last_count + threads - 1) / threads;
    nms_kernel<<<nms_blocks, threads, 0, stream>>>(d_boxes, last_count, nms_thresh, d_keep_flags);
    compact_kernel<<<nms_blocks, threads, 0, stream>>>(
        d_boxes, d_keep_flags, last_count, d_kept_boxes, d_kept_count, kMaxBoxes);

    int kept_count = 0;
    int cur_count = 0;
    cudaMemcpyAsync(&kept_count, d_kept_count, sizeof(int), cudaMemcpyDeviceToHost, stream);
    cudaMemcpyAsync(&cur_count, d_count, sizeof(int), cudaMemcpyDeviceToHost, stream);
    cudaMemcpyAsync(h_boxes, d_kept_boxes, kMaxBoxes * sizeof(DecodedBBox), cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    // 更新 last_count 供下一帧使用
    last_count = std::min(cur_count, kMaxBoxes);
    if (last_count == 0) last_count = kMaxBoxes; // 避免空帧导致下帧跳过所有框

    output_boxes.clear();
    kept_count = std::min(kept_count, kMaxBoxes);
    if (kept_count <= 0) return;

    output_boxes.assign(h_boxes, h_boxes + kept_count);
}

void release_decode_kernel_resources()
{
    if (!is_init) return;

    if (d_boxes) cudaFree(d_boxes);
    if (d_kept_boxes) cudaFree(d_kept_boxes);
    if (d_count) cudaFree(d_count);
    if (d_kept_count) cudaFree(d_kept_count);
    if (d_keep_flags) cudaFree(d_keep_flags);
    if (h_boxes) cudaFreeHost(h_boxes);

    d_boxes = nullptr;
    d_kept_boxes = nullptr;
    d_count = nullptr;
    d_kept_count = nullptr;
    d_keep_flags = nullptr;
    h_boxes = nullptr;
    is_init = false;
}

} // namespace armor_task
