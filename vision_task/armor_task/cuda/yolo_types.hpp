#pragma once

struct GPUBoudingBox{
    float cx, cy, w, h;
    float score;
    int class_id;
    float kpts[8];
};