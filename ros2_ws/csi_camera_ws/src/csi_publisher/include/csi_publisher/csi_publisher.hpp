#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <std_msgs/msg/header.h>

int sensor_id_R = 0;
int sensor_id_L = 1;
int capture_width = 1280;
int capture_height = 720;
int display_width = 512;
int display_height = 256;
int framerate = 10;
int flip_method = 0;