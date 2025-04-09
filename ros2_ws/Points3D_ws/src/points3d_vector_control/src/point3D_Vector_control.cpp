#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ffdirewolfbutton_msgs_tools/FFDirewolfButton_msgsTools.hpp>

int CameraWidth = 512;
int CameraHeight = 256;
float Pi = 3.1415926;
int cx = CameraWidth / 2;
int cy = CameraHeight / 2;
float FOV = 120.f * (Pi / 180.0f);
float f = CameraWidth / (2 * tan(FOV / 2.f));
float T = 0.125;

cv::Mat Q = (cv::Mat_<double>(4, 4) <<
    1, 0, 0, -cx, // 
    0, 1, 0, -cy, //
    0, 0, 0, f,   //
    0, 0, -1 / T, 0  //
    );

float* processPointClouds(cv::Mat PointCloud) {
    cv::Mat pointCloud = PointCloud;

    const float minSafeDistance = 0.5f; // 
    const float maxSpeed = 1.0f; // 
    const float maxLateralSpeed = 1.0f; // 
    const float maxRotationSpeed = 1.0f; //

    float* controlCommand = new float[3];

    int roiLeft = pointCloud.cols / 4; // 
    int roiRight = 3 * pointCloud.cols / 4;
    int roiTop = 0; // 
    int roiBottom = pointCloud.rows;

    int obstacleCountR = 0;
    int obstacleCountL = 0;
    int totalPoints = 0;

    for (int y = roiTop; y < roiBottom; y++) {
        for (int x = roiLeft; x < roiRight; x++) {
            cv::Vec3f point = pointCloud.at<cv::Vec3f>(y, x);
            float distance = -point[2]; // 
            if (y >= 70 && y <= 180 && x >= 256 && x <= 376 && distance < minSafeDistance && distance > 0) {
                obstacleCountR++;
            }
            if (y >= 70 && y <= 180 && x >= 136 && x <= 256 && distance < minSafeDistance && distance > 0) {
                obstacleCountL++;
            }
            totalPoints++;
        }
    }

    float obstacleRatioR = static_cast<float>(obstacleCountR) / totalPoints;
    float obstacleRatioL = static_cast<float>(obstacleCountL) / totalPoints;

    if (obstacleRatioL > 0.15) {
        controlCommand[0] = 0;
        controlCommand[1] = 0;
        controlCommand[2] = -maxRotationSpeed;
    }
    else if(obstacleRatioR > 0.15){
        controlCommand[0] = 0;
        controlCommand[1] = 0;
        controlCommand[2] = maxRotationSpeed;
    }
    else
    {
        controlCommand[0] = maxSpeed;
        controlCommand[1] = 0;
        controlCommand[2] = 0;
    }
    return controlCommand;
}

class point3D_Vector_controlNode: public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr disp_suber;
        rclcpp::Publisher<ffdirewolfbutton_msgs::msg::AxisButtonMsgs>::SharedPtr ffdMsg_puber;
        ffdirewolfbutton_msgs::msg::AxisButtonMsgs FFDMsg;

    public:
        point3D_Vector_controlNode(std::string NodeName="point3D_Vector_controlNode", std::string topicName="point3D_controll_Vector", std::string topicName_Disp="lightStereo_disp_image", std::string topicName_FFD="FFDirewolfMsgsPub", int cl_Disp=3, int cl=10):Node(NodeName)
        {
            ffdMsg_puber = this->create_publisher<ffdirewolfbutton_msgs::msg::AxisButtonMsgs>(topicName_FFD, cl);
            FFDirewolfAxisButtonInit(FFDMsg);
            disp_suber = this->create_subscription<sensor_msgs::msg::Image>(topicName_Disp, cl_Disp, std::bind(&point3D_Vector_controlNode::DispToVectorAndFFD, this, std::placeholders::_1));
        }

        ffdirewolfbutton_msgs::msg::AxisButtonMsgs VectorToFFD(float* vector)
        {
            ffdirewolfbutton_msgs::msg::AxisButtonMsgs TempFFDMsg;
            FFDirewolfAxisButtonInit(TempFFDMsg);
            TempFFDMsg.axis[1] = -vector[0] * AxisMaxValue / 3;
            TempFFDMsg.axis[2] = vector[2] * AxisMaxValue / 3;
            TempFFDMsg.control_mode = 2;
            return TempFFDMsg;
        }

        void DispToVectorAndFFD(sensor_msgs::msg::Image DispMsg)
        {
            cv_bridge::CvImagePtr  DispPtr;
            DispPtr = cv_bridge::toCvCopy(DispMsg, sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat dsip_flip;
            cv::Mat DepthPred;
            cv::flip(DispPtr->image, dsip_flip, 1);
            cv::reprojectImageTo3D(dsip_flip, DepthPred, Q, false);
            float* command = processPointClouds(DepthPred);
            printf("%f, %f, %f\n", command);
            FFDMsg = VectorToFFD(command);
            ffdMsg_puber->publish(FFDMsg);
        }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto NodeTest = std::make_shared<point3D_Vector_controlNode>();
    rclcpp::spin(NodeTest);
    rclcpp::shutdown();
    return 0;
}
