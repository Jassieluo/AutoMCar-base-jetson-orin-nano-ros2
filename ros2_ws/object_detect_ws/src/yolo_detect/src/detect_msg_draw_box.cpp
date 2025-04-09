#include<opencv2/opencv.hpp>
#include"yolo_detect/process.h"
#include"yolo_detect/utils.h"
#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

class detect_msg_draw_box_node : public rclcpp::Node
{
    private:
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_suber;
        std::shared_ptr<message_filters::Subscriber<detect_msgs::msg::DetectMsgs>> detect_msg_suber;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, detect_msgs::msg::DetectMsgs>>> SyncID;
        std::map<int, std::string> labels;
        float scale = 1.0;

    public:
        detect_msg_draw_box_node(std::string nodeName="detect_msg_draw_box_node", std::string imageTopicName="csi_L_image", std::string detTopicName="yolo11n_detmsgs",
         std::string class_file="/home/jassie/my_works/yolov8_tensorrt-main/weights/classes.txt", float scale_=1.0, int imageCl=3, int cL=10):Node(nodeName)
        {
            image_suber = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, imageTopicName);
            detect_msg_suber = std::make_shared<message_filters::Subscriber<detect_msgs::msg::DetectMsgs>>(this, detTopicName);
            SyncID = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, detect_msgs::msg::DetectMsgs>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, detect_msgs::msg::DetectMsgs>(imageCl), *image_suber, *detect_msg_suber);
            SyncID->registerCallback(&detect_msg_draw_box_node::image_detmsg_call_callback, this);
            readClassFile(class_file, labels);
            scale = scale_;
        }

        void image_detmsg_call_callback(sensor_msgs::msg::Image image_msg, detect_msgs::msg::DetectMsgs detect_msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            drawBbox_DetMsg(cv_ptr->image, detect_msg, scale, labels);
            cv::imshow("detect_msg_Imshow", cv_ptr->image);
            cv::waitKey(1);
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<detect_msg_draw_box_node>("detect_msg_draw_box_node"));
    rclcpp::shutdown();
}