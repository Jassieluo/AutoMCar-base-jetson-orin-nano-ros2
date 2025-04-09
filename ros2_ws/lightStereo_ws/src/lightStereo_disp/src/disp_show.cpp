#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class disp_show_node : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr DispSuber;
        int isColor = 1;

        void disp_callback_color_show(sensor_msgs::msg::Image disp_sensor_image)
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(disp_sensor_image, sensor_msgs::image_encodings::TYPE_32FC1);

            cv::Mat normalized_disp_pred, color_normalized_disp_pred;
            double minVal, maxVal;
            cv::minMaxLoc(cv_ptr->image, &minVal, &maxVal);
            cv_ptr->image.convertTo(normalized_disp_pred, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
            if (isColor)
            {
                cv::applyColorMap(normalized_disp_pred, color_normalized_disp_pred, cv::COLORMAP_JET);
                cv::imshow("color_normalized_disp_pred", color_normalized_disp_pred);
            }
            else
            {
                cv::imshow("normalized_disp_pred", normalized_disp_pred);
            }
            cv::waitKey(1);
        }

    public:
        disp_show_node(std::string nodeName, std::string disp_image_topiName="lightStereo_disp_image", int isColor_=1, int cL=3):Node(nodeName)
        {
            isColor = isColor_;
            DispSuber = this->create_subscription<sensor_msgs::msg::Image>(disp_image_topiName, cL, std::bind(&disp_show_node::disp_callback_color_show, this, std::placeholders::_1));
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::string nodeName = "disp_show_node";
    std::string disp_image_topiName="lightStereo_disp_image";
    int isColor = 1;
    auto dispShowNode = std::make_shared<disp_show_node>(nodeName, disp_image_topiName, isColor);
    rclcpp::spin(dispShowNode);
    rclcpp::shutdown();
    return 0;
}