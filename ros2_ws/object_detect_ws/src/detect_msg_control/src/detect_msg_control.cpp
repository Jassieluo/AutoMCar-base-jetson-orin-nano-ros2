#include <rclcpp/rclcpp.hpp>
#include <ffdirewolfbutton_msgs_tools/FFDirewolfButton_msgsTools.hpp>
#include <detect_msgs/msg/detect_msgs.hpp>

//float cameraHeight = 256.f;
float cameraWidth = 512.f;
float thresh = 4/9;
float maxCommand = 1.f;
float configThresh = 0.25f;

class detect_msg_Vector_controlNode: public rclcpp::Node
{
    private:
        rclcpp::Subscription<detect_msgs::msg::DetectMsgs>::SharedPtr detMsg_suber;
        rclcpp::Publisher<ffdirewolfbutton_msgs::msg::AxisButtonMsgs>::SharedPtr ffdMsg_puber;
        ffdirewolfbutton_msgs::msg::AxisButtonMsgs FFDMsg;

    public:
        detect_msg_Vector_controlNode(std::string NodeName="detect_msg_Vector_controlNode", std::string topicName="detect_msg_control_Vector", std::string topicName_detmsg="yolo11n_detmsgs", std::string topicName_FFD="FFDirewolfMsgsPub", int cl_detmsg=3, int cl=10):Node(NodeName)
        {
	    RCLCPP_INFO(this->get_logger(), "111\n");
            ffdMsg_puber = this->create_publisher<ffdirewolfbutton_msgs::msg::AxisButtonMsgs>(topicName_FFD, cl);
	    RCLCPP_INFO(this->get_logger(), "222\n");
            FFDirewolfAxisButtonInit(FFDMsg);
	    RCLCPP_INFO(this->get_logger(), "333\n");
            detMsg_suber = this->create_subscription<detect_msgs::msg::DetectMsgs>(topicName_detmsg, cl_detmsg, std::bind(&detect_msg_Vector_controlNode::DetMsgToVectorAndFFD, this, std::placeholders::_1));
	    RCLCPP_INFO(this->get_logger(), "444\n");
        }

        float* processDetMsg(detect_msgs::msg::DetectMsgs detMsg) // need more modified
        {
            float* command = new float[3];
            command[0] = 0;
            command[1] = 0;
            command[2] = 0;
            if (detMsg.detect_msg.empty() || detMsg.detect_msg[0].conf<configThresh) return command;
            float targetX = (detMsg.detect_msg[0].bbox[0] + detMsg.detect_msg[0].bbox[2]) / 2;
            //float targetY = (detMsg.detect_msg[0].bbox[1] + detMsg.detect_msg[0].bbox[3]) / 2;
            if (targetX<=(cameraWidth*thresh))
            {
                command[0] = 0;
                command[1] = 0;
                command[2] = maxCommand;
                
            }
            else if(targetX>=(cameraWidth*(1-thresh)))
            {
                command[0] = 0;
                command[1] = 0;
                command[2] = -maxCommand;
            }
            else
            {
                command[0] = maxCommand;
                command[1] = 0;
                command[2] = 0;
            }
            
            return command;
        }

        ffdirewolfbutton_msgs::msg::AxisButtonMsgs VectorToFFD(float* vector)
        {
            ffdirewolfbutton_msgs::msg::AxisButtonMsgs TempFFDMsg;
            FFDirewolfAxisButtonInit(TempFFDMsg);
            TempFFDMsg.axis[1] = -vector[0] * AxisMaxValue / 3;
            TempFFDMsg.axis[2] = vector[2] * AxisMaxValue / 3;
            TempFFDMsg.control_mode = 1;
            return TempFFDMsg;
        }

        void DetMsgToVectorAndFFD(detect_msgs::msg::DetectMsgs detMsg)
        {

            float* command = processDetMsg(detMsg);
            RCLCPP_INFO(this->get_logger(), "%f, %f, %f\n", command[0], command[1], command[2]);
            FFDMsg = VectorToFFD(command);
            ffdMsg_puber->publish(FFDMsg);
        }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    printf("11111111\n");
    auto NodeTest = std::make_shared<detect_msg_Vector_controlNode>();
    printf("222222\n");
    rclcpp::spin(NodeTest);
    rclcpp::shutdown();
    return 0;
}
