#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <detect_msgs/msg/detect_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tcp_objectDetection/utils.h>
#include <sys/socket.h>
#include <netinet/in.h>   
#include <netdb.h>   
#include <arpa/inet.h>  
#include <sys/types.h>
#include <atomic>
#include <unistd.h>
#include <errno.h>
#include <vector>
#include <tcp_objectDetection/utils.h>

class tcp_objectDetection_pubNode:public rclcpp::Node
{
    private:
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_suber;
        std::shared_ptr<message_filters::Subscriber<detect_msgs::msg::DetectMsgs>> detect_msg_suber;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, detect_msgs::msg::DetectMsgs>>> SyncID;
        std::map<int, std::string> labels;

        int sock;
        int client_sock;
        int compressSucess=0;
        int count=0;

    public:
        tcp_objectDetection_pubNode(std::string nodeName="tcp_objectDetection_pubNode", std::string topicName_image="csi_L_image", std::string topicName_detectMsg="yolo11n_detmsgs",
         std::string class_file="/home/jassie/my_works/ros2_ws/tcp_ws/src/tcp_objectDetection/classes.txt", int cl=1):Node(nodeName)
        {
            sock = socket(AF_INET, SOCK_STREAM, 0);

            if (sock<0)
            {
                std::cerr << "Error at connect(): " << errno <<std::endl;
                close(sock);
            }

            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            sockaddr_in local = { 0 };

	        local.sin_family = AF_INET;
	        local.sin_port = htons(8081);
		    local.sin_addr.s_addr = INADDR_ANY;

            if (-1 == bind(sock, (struct sockaddr*)&local, sizeof(local)))
            {
                std::cerr << "Error at bind(): " << errno <<std::endl;
                close(sock);
            }

            if (-1 == listen(sock, 10))
            {
                std::cerr << "Error at listen(): " << errno <<std::endl;
                close(sock);
            }
            RCLCPP_INFO(this->get_logger(),"\nWaiting for Connect.\n");
		    client_sock = accept(sock, NULL, NULL);
            if (-1 == client_sock)
            {
                std::cerr << "Error at accept(): " << errno <<std::endl;
                close(sock);
            }

            image_suber = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, topicName_image);
            detect_msg_suber = std::make_shared<message_filters::Subscriber<detect_msgs::msg::DetectMsgs>>(this, topicName_detectMsg);
            SyncID = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, detect_msgs::msg::DetectMsgs>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, detect_msgs::msg::DetectMsgs>(cl), *image_suber, *detect_msg_suber);
            SyncID->registerCallback(&tcp_objectDetection_pubNode::image_detmsg_call_callback, this);
            readClassFile(class_file, labels);
        }

        std::vector<uchar> compressImage(cv::Mat image)
        {
            std::vector<uchar> compreesImageData;
            std::vector<int> params;
            params.push_back(cv::IMWRITE_JPEG_QUALITY);
            params.push_back(50);
            compressSucess = cv::imencode(".jpg", image, compreesImageData, params);
            return compreesImageData;
        }

        void image_detmsg_call_callback(sensor_msgs::msg::Image image_msg, detect_msgs::msg::DetectMsgs detect_msg)
        {
            RCLCPP_INFO(this->get_logger(), "Start\n");
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            drawBbox_DetMsg(cv_ptr->image, detect_msg, labels);
            std::vector<uchar> ImageData;
            
		    ImageData = compressImage(cv_ptr->image);

            RCLCPP_INFO(this->get_logger(), "num: %d, bytesSend: %d\n", count, ImageData.size());
            count++;

            if (compressSucess)
            {
                int bytesSend = send(client_sock, ImageData.data(), sizeof(uchar) * ImageData.size(), 0);
                cv::waitKey(30);

                if (bytesSend<0)
                {
                    close(client_sock);
                    RCLCPP_INFO(this->get_logger(), "Connetion lost, waiting for new connect");
                    if (-1 == listen(sock, 10))
                    {
                        std::cerr << "Error at listen(): " << errno <<std::endl;
                        close(sock);
                    }
                    RCLCPP_INFO(this->get_logger(),"\nWaiting for Connect.\n");
                    client_sock = accept(sock, NULL, NULL);
                }
                if (-1 == client_sock)
                {
                    std::cerr << "Error at accept(): " << errno <<std::endl;
                    close(sock);
                }
            }
        }
        
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto tcp_objectDetectionNode = std::make_shared<tcp_objectDetection_pubNode>();
    rclcpp::spin(tcp_objectDetectionNode);
    rclcpp::shutdown();
    return 0;
}
