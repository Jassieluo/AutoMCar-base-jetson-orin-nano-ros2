#include "csi_publisher/csi_publisher.hpp"

std::string gstreamer_pipeline(int sensor_id, int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc sensor-id="+std::to_string(sensor_id)+" ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

class csi_L_pubNode : public rclcpp::Node
{
    private:
        cv::VideoCapture capL;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ImageRpublisher;
        rclcpp::TimerBase::SharedPtr time_;
        cv::Mat imgL;

        void timer_publish()
        {
            capL>>imgL;
            if (imgL.size().height!=0)
            {
                // cv::resize(imgL, imgL, cv::Size(display_width, display_height));
                ImageRpublisher->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imgL).toImageMsg());
            }
        }

    public:
        csi_L_pubNode(const std::string pipline, const std::string nodeName, const std::string topicName, int cL=3):Node(nodeName)
        {
            capL = cv::VideoCapture(pipline, cv::CAP_GSTREAMER);
            ImageRpublisher = this->create_publisher<sensor_msgs::msg::Image>(topicName, cL);
            time_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&csi_L_pubNode::timer_publish, this));
        }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::string pipline = gstreamer_pipeline(
        sensor_id_L,
        capture_width,
        capture_height,
        display_width,
        display_height,
        framerate,
        flip_method
    );
    rclcpp::spin(std::make_shared<csi_L_pubNode>(pipline, "csi_L_puber", "csi_L_image"));
    rclcpp::shutdown(); 
}