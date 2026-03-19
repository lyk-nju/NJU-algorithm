// tasks/postprocess.cu
#include "detector.hpp"
#include <cuda_runtime.h>

namespace armor_task {

__global__ void preprocess_kernel(
    const uint8_t* __restrict__ src, 
    float* dst, 
    int src_w, int src_h, int src_step, 
    int dst_w, int dst_h, 
    float scale, int pad_x, int pad_y
) {
    // 当前线程负责处理的目标像素坐标 (dx, dy)
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= dst_w * dst_h) return;

    int dy = idx / dst_w;
    int dx = idx % dst_w;

    // 1. 去掉 Padding，计算出在“Unpad”图像中的坐标
    int unpad_x = dx - pad_x;
    int unpad_y = dy - pad_y;

    // 计算有效区域的大小
    int valid_w = (int)(src_w * scale);
    int valid_h = (int)(src_h * scale);

    // 2. 初始化颜色为 114 (YOLO默认灰色) 或 0 (黑色)
    // 这里我们用 0，和你原来的 blobFromImage 保持一致
    // dst 的内存排列是 CHW (Planar)
    // R通道 offset: 0
    // G通道 offset: dst_w * dst_h
    // B通道 offset: 2 * dst_w * dst_h
    
    float r = 0.0f, g = 0.0f, b = 0.0f;

    // 3. 如果在有效图像范围内，则进行坐标映射和采样
    if (unpad_x >= 0 && unpad_x < valid_w && unpad_y >= 0 && unpad_y < valid_h) {
        // 坐标映射：dst -> src (最近邻插值，对于检测任务通常足够且最快)
        // 如果想更精确可以用双线性插值，但代码复杂很多
        int sx = (int)(unpad_x / scale);
        int sy = (int)(unpad_y / scale);

        // 边界保护
        if(sx >= src_w) sx = src_w - 1;
        if(sy >= src_h) sy = src_h - 1;

        // 获取源像素 BGR
        // src_step 是这一行的字节数 (通常等于 width * 3，但也可能有 padding)
        int src_idx = sy * src_step + sx * 3;

        // 读取 BGR (OpenCV 默认顺序)
        uint8_t b_val = src[src_idx + 0];
        uint8_t g_val = src[src_idx + 1];
        uint8_t r_val = src[src_idx + 2];

        // 归一化 + BGR转RGB
        r = r_val / 255.0f;
        g = g_val / 255.0f;
        b = b_val / 255.0f;
    }

    // 4. 写入输出 (CHW 格式)
    int area = dst_w * dst_h;
    dst[idx]          = r; // R Plane
    dst[idx + area]   = g; // G Plane
    dst[idx + 2*area] = b; // B Plane
}

void launch_preprocess_kernel(
    const uint8_t* src, int src_w, int src_h, int src_step,
    float* dst, int dst_w, int dst_h,
    cudaStream_t stream
) {
    // 1. 计算缩放比例 (Letterbox逻辑)
    float r_w = (float)dst_w / src_w;
    float r_h = (float)dst_h / src_h;
    float scale = (r_w < r_h) ? r_w : r_h;

    // 计算 padding
    int new_unpad_w = (int)(src_w * scale);
    int new_unpad_h = (int)(src_h * scale);
    int pad_x = (dst_w - new_unpad_w) / 2;
    int pad_y = (dst_h - new_unpad_h) / 2;

    // 2. 启动 Kernel
    int num_pixels = dst_w * dst_h;
    int threads = 256;
    int blocks = (num_pixels + threads - 1) / threads;

    preprocess_kernel<<<blocks, threads, 0, stream>>>(
        src, dst, 
        src_w, src_h, src_step, 
        dst_w, dst_h, 
        scale, pad_x, pad_y
    );
}

}
