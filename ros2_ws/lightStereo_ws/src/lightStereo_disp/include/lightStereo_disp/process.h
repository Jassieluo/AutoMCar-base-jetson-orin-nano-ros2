#pragma once
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cstdint>
#include <opencv2/opencv.hpp>

struct AffineMatrix {
    float value[6];
};

void preprocess(uint8_t* src, const int& src_width, const int& src_height, float* dst,
 const int& dst_width, const int& dst_height, cudaStream_t stream, float& scale);

void preprocess_no_resize(uint8_t* src, const int& src_width, const int& src_height,
 float* dst, const int& dst_width, const int& dst_height, cudaStream_t stream);