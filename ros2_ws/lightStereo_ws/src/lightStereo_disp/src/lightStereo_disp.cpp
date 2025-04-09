#include "lightStereo_disp/process.h"
#include "lightStereo_disp/utils.h"
#include "lightStereo_disp/config.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

class lightStereo_disp_node : public rclcpp::Node
{
    private:
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> LImageSuber;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> RImageSuber;
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> SyncLR;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr DispPuber;

        nvinfer1::IRuntime* runtime;
        nvinfer1::ICudaEngine* engine;
        nvinfer1::IExecutionContext* context;
        cudaStream_t stream;

        float* device_buffers[3];
        uint8_t* image_l_device = nullptr;
        uint8_t* image_r_device = nullptr;
        float* output_buffer_host;
        uint8_t* image_device = nullptr;

        int input_img_l_height;
        int input_img_l_weight;
        int input_img_r_height;
        int input_img_r_weight;
        int output_img_height;
        int output_img_weight;

        void sync_callback_LR_lightStereo_infer_disp_pub(sensor_msgs::msg::Image sensorLImage, sensor_msgs::msg::Image sensorRImage)
        {
            cv_bridge::CvImagePtr cv_ptr_l;
            cv_bridge::CvImagePtr cv_ptr_r;
            cv_ptr_l = cv_bridge::toCvCopy(sensorLImage, sensor_msgs::image_encodings::BGR8);
            cv_ptr_r = cv_bridge::toCvCopy(sensorRImage, sensor_msgs::image_encodings::BGR8);

            cv::Mat disp_pred(cv::Size(output_img_weight, output_img_height), CV_32FC1);

            auto t_beg = std::chrono::high_resolution_clock::now();

            cudaMemcpyAsync(image_device, cv_ptr_l->image.data, cv_ptr_l->image.size().area() * 3, cudaMemcpyHostToDevice, stream);
            preprocess_no_resize(image_device, cv_ptr_l->image.size().width, cv_ptr_l->image.size().height, device_buffers[0], input_img_l_weight, input_img_l_height, stream);
            cudaMemcpyAsync(image_device, cv_ptr_r->image.data, cv_ptr_r->image.size().area() * 3, cudaMemcpyHostToDevice, stream);
            preprocess_no_resize(image_device, cv_ptr_r->image.size().width, cv_ptr_r->image.size().height, device_buffers[1], input_img_r_weight, input_img_r_height, stream);
            
            context->enqueueV3(stream);
        
            cudaMemcpyAsync(disp_pred.data, device_buffers[2], output_img_height * output_img_weight * sizeof(float), cudaMemcpyDeviceToHost, stream);

            auto t_end = std::chrono::high_resolution_clock::now();
            float total_inf = std::chrono::duration<float, std::milli>(t_end - t_beg).count();

            DispPuber->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", disp_pred).toImageMsg());
        }

    public:
        lightStereo_disp_node(std::string engine_path, std::string nodeName="lightStereo_disp_node", std::string topicName="lightStereo_disp_image",
         std::string LimgtopicName="csi_L_image", std::string RimgtopicName="csi_R_image", int ImageCl=3, int cL=10):Node(nodeName)
        {
            LImageSuber = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, LimgtopicName);
            RImageSuber = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, RimgtopicName);
            SyncLR = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(ImageCl), *LImageSuber, *RImageSuber);
            SyncLR->registerCallback(&lightStereo_disp_node::sync_callback_LR_lightStereo_infer_disp_pub, this);

            DispPuber = this->create_publisher<sensor_msgs::msg::Image>(topicName, cL);

            readEngineFile(engine_path, runtime, engine, context);
            cudaStreamCreate(&stream);

            input_img_l_height = engine->getTensorShape(engine->getIOTensorName(0)).d[engine->getTensorShape(engine->getIOTensorName(0)).nbDims-2];
            input_img_l_weight = engine->getTensorShape(engine->getIOTensorName(0)).d[engine->getTensorShape(engine->getIOTensorName(0)).nbDims-1];
            input_img_r_height = engine->getTensorShape(engine->getIOTensorName(1)).d[engine->getTensorShape(engine->getIOTensorName(1)).nbDims-2];
            input_img_r_weight = engine->getTensorShape(engine->getIOTensorName(1)).d[engine->getTensorShape(engine->getIOTensorName(1)).nbDims-1];
            output_img_height = engine->getTensorShape(engine->getIOTensorName(2)).d[engine->getTensorShape(engine->getIOTensorName(2)).nbDims-2];
            output_img_weight = engine->getTensorShape(engine->getIOTensorName(2)).d[engine->getTensorShape(engine->getIOTensorName(2)).nbDims-1];

            output_buffer_host = new float[kBatchSize * 3 * output_img_height * output_img_weight];
            
            cudaMalloc((void**)&image_device, 3 * input_img_l_height * input_img_l_weight);
            cudaMalloc((void**)&device_buffers[0], sizeof(float) * 3 * input_img_l_height * input_img_l_weight);
            cudaMalloc((void**)&device_buffers[1], sizeof(float) * 3 * input_img_r_height * input_img_r_weight);
            cudaMalloc((void**)&device_buffers[2], sizeof(float) * 1 * output_img_height * output_img_weight);

            context->setTensorAddress(engine->getIOTensorName(0), device_buffers[0]);
            context->setTensorAddress(engine->getIOTensorName(1), device_buffers[1]);
            context->setTensorAddress(engine->getIOTensorName(2), device_buffers[2]);
        }

        void release_free()
        {
            // Release stream and buffers
            cudaStreamDestroy(stream);
            cudaFree(device_buffers[0]);
            cudaFree(device_buffers[1]);
            cudaFree(device_buffers[2]);
            delete[] output_buffer_host;
            // Destroy the engine
            delete context;
            delete engine;
            delete runtime;
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::string engine_path = "/home/jassie/my_works/weights/LightStereo-S-SceneFlow_fp16_.engine";
    std::string nodeName = "lightStereo_disp_node";
    std::string topicName = "lightStereo_disp_image";
    std::string LimgtopicName = "csi_L_image";
    std::string RimgtopicName = "csi_R_image";
    auto lightStereoNode = std::make_shared<lightStereo_disp_node>(engine_path, nodeName, topicName, LimgtopicName, RimgtopicName, 3, 10);
    rclcpp::spin(lightStereoNode);
    rclcpp::shutdown();
    return 0;
}
