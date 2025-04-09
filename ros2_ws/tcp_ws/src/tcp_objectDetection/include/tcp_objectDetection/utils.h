#pragma once
#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include <detect_msgs/msg/detect_msgs.hpp>
#include <opencv2/opencv.hpp>

struct alignas(float) Detection {
  float bbox[4];
  float conf;
  float class_id;
};

void readClassFile(const std::string& class_file, std::map<int, std::string>& labels);

void drawBbox_DetMsg(cv::Mat& img, detect_msgs::msg::DetectMsgs& detect_msg_res, std::map<int, std::string>& Labels);

void NMS_DetMsg(detect_msgs::msg::DetectMsgs& detect_msg_res, float* output, const float& conf_thresh, const float& nms_thresh);