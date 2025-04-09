#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <netinet/in.h>   
#include <netdb.h>   
#include <arpa/inet.h>  
#include <sys/types.h>
#include <atomic>
#include <unistd.h>
#include <errno.h>
#include <vector>

class tcp_disp_send_node: public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr DispSuber;
        int sock_disp;
        int client_sock_disp;
        int compressSucess=0;
        int count_disp=0;

    public:
        tcp_disp_send_node(std::string nodeName="disp_tcp_send_node", std::string topicName_disp="lightStereo_disp_image", std::string topicName_obj="objectDetection_image", int cl_disp=1, int cl_obj=1):Node(nodeName)
        {
            sock_disp = socket(AF_INET, SOCK_STREAM, 0);

            if (sock_disp<0)
            {
                std::cerr << "Error at connect(): " << errno <<std::endl;
                close(sock_disp);
            }

            int opt = 1;
            setsockopt(sock_disp, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            sockaddr_in local_disp = { 0 };

	        local_disp.sin_family = AF_INET;
	        local_disp.sin_port = htons(8080);
		    local_disp.sin_addr.s_addr = INADDR_ANY;

            if (-1 == bind(sock_disp, (struct sockaddr*)&local_disp, sizeof(local_disp)))
            {
                std::cerr << "Error at bind_disp(): " << errno <<std::endl;
                close(sock_disp);
            }

            if (-1 == listen(sock_disp, 10))
            {
                std::cerr << "Error at listen_disp(): " << errno <<std::endl;
                close(sock_disp);
            }

            RCLCPP_INFO(this->get_logger(),"\nWaiting for Connect.\n");
		    client_sock_disp = accept(sock_disp, NULL, NULL);

            if (-1 == client_sock_disp)
            {
                std::cerr << "Error at accept_disp(): " << errno <<std::endl;
                close(sock_disp);
            }

            DispSuber = this->create_subscription<sensor_msgs::msg::Image>(topicName_disp, cl_disp, std::bind(&tcp_disp_send_node::disp_callback_tcp_send, this, std::placeholders::_1));
        }
        
        std::vector<uchar> compressImage(cv::Mat image)
        {
            std::vector<uchar> compreesImageData;
            std::vector<int> params;
            params.push_back(cv::IMWRITE_JPEG_QUALITY);
            params.push_back(60);
            compressSucess = cv::imencode(".jpg", image, compreesImageData, params);
            return compreesImageData;
        }

        void disp_callback_tcp_send(sensor_msgs::msg::Image disp_sensor_image)
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(disp_sensor_image, sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat normalized_disp_pred;
            double minVal, maxVal;
            cv::minMaxLoc(cv_ptr->image, &minVal, &maxVal);
            cv::normalize(cv_ptr->image, normalized_disp_pred, 0, 1, cv::NORM_MINMAX);
            cv::Mat ucharMat;
            normalized_disp_pred.convertTo(ucharMat, CV_8UC1, 255);

            std::vector<uchar> ImageData;
		    ImageData = compressImage(ucharMat);
            if (compressSucess)
            {
                std::vector<uchar> sendData;
                sendData.resize(sizeof(double)*2+ sizeof(int) + sizeof(uchar) * ImageData.size());
                memcpy(sendData.data(), &minVal, sizeof(double));
                memcpy(sendData.data()  + sizeof(double), &maxVal, sizeof(double));
                int ImageDataSize = ImageData.size();
                memcpy(sendData.data()  + sizeof(double) * 2, &ImageDataSize, sizeof(int));
                memcpy(sendData.data()  + sizeof(double) * 2 + sizeof(int), ImageData.data(), sizeof(uchar) * ImageData.size());
                RCLCPP_INFO(this->get_logger(), "num: %d, min: %.5lf, max: %.5lf, bytesSend: %d\n", count_disp, minVal, maxVal, ImageDataSize);
                count_disp++;

                int bytesSend_disp = send(client_sock_disp, sendData.data(), (sizeof(double) * 2 + sizeof(int) + sizeof(uchar) * ImageData.size()), 0);
                cv::waitKey(30);
                if (bytesSend_disp<0)
                {
                    close(client_sock_disp);
                    RCLCPP_INFO(this->get_logger(), "Connetion lost, waiting for new connect");
                    if (-1 == listen(sock_disp, 10))
                    {
                        std::cerr << "Error at listen(): " << errno <<std::endl;
                        close(sock_disp);
                    }
                    RCLCPP_INFO(this->get_logger(),"\nWaiting for Connect.\n");
                    client_sock_disp = accept(sock_disp, NULL, NULL);
                }
                if (-1 == client_sock_disp)
                {
                    std::cerr << "Error at accept(): " << errno <<std::endl;
                    close(sock_disp);
                }
            }
        }
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto disp_node = std::make_shared<tcp_disp_send_node>();
    rclcpp::spin(disp_node);
    rclcpp::shutdown();
    return 0;
}