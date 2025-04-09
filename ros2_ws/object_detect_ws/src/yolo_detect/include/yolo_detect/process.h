#pragma once
#include <opencv2/opencv.hpp>
#include "NvInfer.h"
#include "config.h"
#include <map>
#include <device_launch_parameters.h>
#include <cuda_runtime.h>
#include <detect_msgs/msg/detect_msgs.hpp>

struct AffineMatrix {
    float value[6];
};

void preprocess(uint8_t* src, const int& src_width, const int& src_height,
    float* dst, const int& dst_width, const int& dst_height,
    cudaStream_t stream, float& scale);
void postprocess(int kClass, int num_ancors, int keep_ancors, float conf, float* src, float* dst, int xyxy, cudaStream_t stream);

void NMS(std::vector<Detection>& res, float* output, const float& conf_thresh, const float& nms_thresh);

void NMS_DetMsg(detect_msgs::msg::DetectMsgs& detect_msg_res, float* output, const float& conf_thresh, const float& nms_thresh);

void drawBbox(cv::Mat& img, std::vector<Detection>& res, float& scale, std::map<int, std::string>& Labels);

void drawBbox_DetMsg(cv::Mat& img, detect_msgs::msg::DetectMsgs& detect_msg_res, float& scale, std::map<int, std::string>& Labels);