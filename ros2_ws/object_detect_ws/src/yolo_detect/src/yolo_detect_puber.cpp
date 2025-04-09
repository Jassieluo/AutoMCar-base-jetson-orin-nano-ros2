#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include "yolo_detect/utils.h"
#include "yolo_detect/process.h"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

Logger gLogger;

class yolo_detect_pub_Node : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sensor_image_suber;
        rclcpp::Publisher<detect_msgs::msg::DetectMsgs>::SharedPtr detect_msgs_puber;
        nvinfer1::IRuntime* runtime = nullptr;
        nvinfer1::ICudaEngine* engine = nullptr;
        nvinfer1::IExecutionContext* context = nullptr;
        cudaStream_t stream;
        int kOutputSize;
        float* device_buffers[2];
        uint8_t* image_device = nullptr;
        float* output_buffer_host;
        float scale = 1.0;

    public:
        yolo_detect_pub_Node(std::string nodeName, std::string image_topicName,std::string topicName,
         std::string yolo_engine_name, int imageCl=3, int cL=10):Node(nodeName)
        {
            detect_msgs_puber = this->create_publisher<detect_msgs::msg::DetectMsgs>(topicName, cL);
            sensor_image_suber = this->create_subscription<sensor_msgs::msg::Image>(image_topicName, imageCl,
             std::bind(&yolo_detect_pub_Node::sensor_image_yolo_detect_call_back, this, std::placeholders::_1));
            readEngineFile(yolo_engine_name, runtime, engine, context);
            cudaStreamCreate(&stream);
            kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;
            output_buffer_host = new float[kBatchSize * kOutputSize];

            cudaMalloc((void**)&image_device, kMaxInputImageSize * 3);
            cudaMalloc((void**)&device_buffers[0], kBatchSize * 3 * kInputH * kInputW * sizeof(float));
            cudaMalloc((void**)&device_buffers[1], (kNumClass+4) * kNumAnchors * sizeof(float));

            context->setTensorAddress(engine->getIOTensorName(0), device_buffers[0]);
            context->setTensorAddress(engine->getIOTensorName(1), device_buffers[1]);
        }

        void sensor_image_yolo_detect_call_back(sensor_msgs::msg::Image sensor_image)
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(sensor_image, sensor_msgs::image_encodings::BGR8);
            int img_size = cv_ptr->image.cols * cv_ptr->image.rows * 3;
            auto t_beg = std::chrono::high_resolution_clock::now();
            cudaMemcpyAsync(image_device, cv_ptr->image.data, img_size, cudaMemcpyHostToDevice, stream);
            preprocess(image_device, cv_ptr->image.cols, cv_ptr->image.rows, device_buffers[0], kInputW, kInputH, stream, scale);
            context->enqueueV3(stream);
            postprocess(kNumClass, kNumAnchors, kMaxNumOutputBbox, 0.01, device_buffers[1], output_buffer_host, 1, stream);
            cudaStreamSynchronize(stream);

            detect_msgs::msg::DetectMsgs detect_msg_res;
            NMS_DetMsg(detect_msg_res, output_buffer_host, kConfThresh, kNmsThresh);
	        for (size_t j = 0; j < detect_msg_res.detect_msg.size(); j++) {
                detect_msg_res.detect_msg[j].bbox[0] /= scale;
                detect_msg_res.detect_msg[j].bbox[1] /= scale;
                detect_msg_res.detect_msg[j].bbox[2] /= scale;
                detect_msg_res.detect_msg[j].bbox[3] /= scale;
            }
            auto t_end = std::chrono::high_resolution_clock::now();
            float total_inf = std::chrono::duration<float, std::milli>(t_end - t_beg).count();
            std::cout << "Inference time: " << int(total_inf) << std::endl;

            detect_msgs_puber->publish(detect_msg_res);
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::string engine_name = "/home/jassie/my_works/weights/yolo11n_bottle_256_512_no_nms.engine";
    rclcpp::spin(std::make_shared<yolo_detect_pub_Node>("yolo11nDet", "csi_L_image", "yolo11n_detmsgs", engine_name));
    rclcpp::shutdown();
}
