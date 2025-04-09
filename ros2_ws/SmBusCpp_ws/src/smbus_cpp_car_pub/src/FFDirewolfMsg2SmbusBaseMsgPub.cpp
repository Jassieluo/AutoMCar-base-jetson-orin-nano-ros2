#include <rclcpp/rclcpp.hpp>
#include <ffdirewolfbutton_msgs_tools/FFDirewolfButton_msgsTools.hpp>
#include <smbus_car_msgs_tools/SmbusCarBaseMsgsTools.hpp>
#include <math.h>

class FFDireMsgRecSmbusBaseMsgPubNode : public:: rclcpp::Node
{
    private:
        // ffdirewolfbutton_msgs::msg::AxisButtonMsgs FFDireMsgs = ffdirewolfbutton_msgs::msg::AxisButtonMsgs();
        smbus_car_msgs::msg::SmbusCarBaseMsgs SmbusBaseMsgs = smbus_car_msgs::msg::SmbusCarBaseMsgs();
        rclcpp::Subscription<ffdirewolfbutton_msgs::msg::AxisButtonMsgs>::SharedPtr FFDMsgSub;
        rclcpp::Publisher<smbus_car_msgs::msg::SmbusCarBaseMsgs>::SharedPtr SmbusBaseMsgPub;
        int SpeedControl = 1;
        int ControlMode = 0;  //0 hand 2 auto 1 detect
        int8_t LimitedSpeed[4] = {25, 25, 25, 25};
        int8_t LimitedPwm[4] = {50, 50, 50, 50};
        int8_t LimitedSpeedAff[4] = {25, 25, 25, 25};
        int8_t LimitedPwmAff[4] = {50, 50, 50, 50};
        int8_t LimitedSpeedAffStep = 5;
        int8_t LimitedPwmAffStep = 5;

    public:
        FFDireMsgRecSmbusBaseMsgPubNode(const std::string NodeName, const std::string FFDTopicName="FFDirewolfMsgsPub", const std::string SmbusTopicName="SmbusCarBaseMsgPub",
         int FFDcL=10, int SmbuscL=10) : Node(NodeName)
        {
            // FFDirewolfAxisButtonInit(FFDireMsgs);
            SmbusCarBaseMsgsInit(SmbusBaseMsgs);
            FFDMsgSub = this->create_subscription<ffdirewolfbutton_msgs::msg::AxisButtonMsgs>(FFDTopicName, FFDcL,
             std::bind(&FFDireMsgRecSmbusBaseMsgPubNode::MsgRecPub, this, std::placeholders::_1));
            SmbusBaseMsgPub = this->create_publisher<smbus_car_msgs::msg::SmbusCarBaseMsgs>(SmbusTopicName, SmbuscL);
            RCLCPP_INFO(this->get_logger(), "%s launch successfully.\n", NodeName.c_str());
        }

        int minSpeedPwm(int8_t* Spw)
        {
            int temp_count = 0;
            for (int i=1;i<4;++i)
            {
                if (Spw[temp_count]>Spw[i])
                {
                    temp_count = i;
                }
            }
            return temp_count;
        }

        int find_minSpeedPwmSpf(int8_t* Spw, double* Spf)
        {
            int temp_count = 0;
            for (int i=1;i<4;++i)
            {
                if (fabs((double(Spw[temp_count])/Spf[temp_count]))>fabs((double(Spw[i])/Spf[i])))
                {
                    temp_count = i;
                }
            }
            return temp_count;
        }
         
        void FFDMsg2SmbusCarBaseMsg(const ffdirewolfbutton_msgs::msg::AxisButtonMsgs &FFDMsgs, smbus_car_msgs::msg::SmbusCarBaseMsgs &SmbusMsgs)
        {
            SmbusMsgs.speed_control = (SpeedControl==1) ? 1:0;
            SmbusMsgs.pwm_control = (SpeedControl==1) ? 0:1;
            // para
            double angle = 0;
            double spf[4] = {0.}; //Speed And Force
            if (FFDMsgs.axis[0]!=0) angle = atan(-double(FFDMsgs.axis[1])/double(FFDMsgs.axis[0])); //AxisMax: 32767
            else if (FFDMsgs.axis[1]<0) angle = M_PI/2;
            else angle = -M_PI/2;
            double Tangle = 0;
            if (angle>0 && FFDMsgs.axis[0]>=0) Tangle = angle;
            else if (angle>=0 && FFDMsgs.axis[0]<0) Tangle = angle + M_PI;
            else if (angle<0 && FFDMsgs.axis[0]>=0) Tangle = angle + 2*M_PI;
            else if (angle<0 && FFDMsgs.axis[0]<0) Tangle = angle + M_PI;
            RCLCPP_INFO(this->get_logger(), "angle: %f\n",angle);
            RCLCPP_INFO(this->get_logger(), "Tangle: %f\n",Tangle);
            spf[0] = cos(M_PI/4 - Tangle) / double(ForceDirection[0]);
            spf[3] = spf[0] * double(ForceDirection[0]) / double(ForceDirection[3]);
            spf[1] = sin(M_PI/4 - Tangle) / double(ForceDirection[1]);
            spf[2] = spf[1] * double(ForceDirection[1]) / double(ForceDirection[2]);
            int AffI=0;
            AffI = (SpeedControl==1) ? find_minSpeedPwmSpf(LimitedSpeed, spf):find_minSpeedPwmSpf(LimitedPwm, spf);
            double AffIValue = (SpeedControl==1) ? fabs((double(LimitedSpeed[AffI])/spf[AffI])):fabs((double(LimitedPwm[AffI])/spf[AffI]));
            RCLCPP_INFO(this->get_logger(), "AffIValue1: %f\n",AffIValue);
            double angleA = (((angle<(M_PI/4)) && ((angle>(-M_PI/4))))) ? (M_PI/2-angle):angle;
            AffIValue = AffIValue * sqrt((double(FFDMsgs.axis[0])*double(FFDMsgs.axis[0]))+(double(FFDMsgs.axis[1])*double(FFDMsgs.axis[1]))) / 
             (double(AxisMaxValue)/fabs(sin(angleA)));
             RCLCPP_INFO(this->get_logger(), "AffIValue2: %f\n",AffIValue);
            RCLCPP_INFO(this->get_logger(), "angleA: %f\n",angleA);
            int AAffI = (SpeedControl==1) ? minSpeedPwm(LimitedSpeedAff):minSpeedPwm(LimitedPwmAff);
            double AAffIValue = double(FFDMsgs.axis[2]) / double(AxisMaxValue);
            AAffIValue = (SpeedControl==1) ? AAffIValue*double(LimitedSpeedAff[AAffI]):AAffIValue*double(LimitedPwmAff[AAffI]);
            double AAspf[4] = {0.};
            AAspf[0] = -double(ForceDirection[0])*AAffIValue;
            AAspf[1] = -double(ForceDirection[1])*AAffIValue;
            AAspf[2] = double(ForceDirection[2])*AAffIValue;
            AAspf[3] = double(ForceDirection[3])*AAffIValue;

            if (SpeedControl==1)
            {
                for (int i=0;i<4;++i)
                {
                    SmbusMsgs.speed[i] = int(spf[i] * AffIValue + AAspf[i]);
                }
            }
            else
            {
                for (int i=0;i<4;++i)
                {
                    SmbusMsgs.pwm[i] = int(spf[i] * AffIValue + AAspf[i]);
                }
            }
        }

        void MsgRecPub(const ffdirewolfbutton_msgs::msg::AxisButtonMsgs &SubFFDMsgs)
        {
            if (SubFFDMsgs.button[10]==1) SpeedControl = 0 - SpeedControl; // SELECT
            if (SubFFDMsgs.button[11]==1) ControlMode = (ControlMode+1)%3;
            if (SubFFDMsgs.axis[4]!=AxisMinValue && SubFFDMsgs.axis[4]!=0)
            {
                if (SpeedControl==1)
                {
                    LimitedSpeed[0] = ((LimitedSpeed[0]+LimitedSpeedAff[0])<=MaxSpeed[0])?(LimitedSpeed[0]+int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedSpeed[0];
                    LimitedSpeed[1] = ((LimitedSpeed[1]+LimitedSpeedAff[1])<=MaxSpeed[1])?(LimitedSpeed[1]+int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedSpeed[1];
                    LimitedSpeed[2] = ((LimitedSpeed[2]+LimitedSpeedAff[2])<=MaxSpeed[2])?(LimitedSpeed[2]+int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedSpeed[2];
                    LimitedSpeed[3] = ((LimitedSpeed[3]+LimitedSpeedAff[3])<=MaxSpeed[3])?(LimitedSpeed[3]+int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedSpeed[3];
                }
                else
                {
                    LimitedPwm[0] = ((LimitedPwm[0]+LimitedPwmAff[0])<=MaxPwmSpeed[0])?(LimitedPwm[0]+int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedPwm[0];
                    LimitedPwm[1] = ((LimitedPwm[1]+LimitedPwmAff[1])<=MaxPwmSpeed[1])?(LimitedPwm[1]+int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedPwm[1];
                    LimitedPwm[2] = ((LimitedPwm[2]+LimitedPwmAff[2])<=MaxPwmSpeed[2])?(LimitedPwm[2]+int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedPwm[2];
                    LimitedPwm[3] = ((LimitedPwm[3]+LimitedPwmAff[3])<=MaxPwmSpeed[3])?(LimitedPwm[3]+int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedPwm[3];
                }
            }
            if (SubFFDMsgs.axis[5]!=AxisMinValue && SubFFDMsgs.axis[5]!=0)
            {
                if (SpeedControl==1)
                {
                    LimitedSpeed[0] = ((LimitedSpeed[0]+LimitedSpeedAff[0])<=MaxSpeed[0])?(LimitedSpeed[0]-int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedSpeed[0];
                    LimitedSpeed[1] = ((LimitedSpeed[1]+LimitedSpeedAff[1])<=MaxSpeed[1])?(LimitedSpeed[1]-int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedSpeed[1];
                    LimitedSpeed[2] = ((LimitedSpeed[2]+LimitedSpeedAff[2])<=MaxSpeed[2])?(LimitedSpeed[2]-int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedSpeed[2];
                    LimitedSpeed[3] = ((LimitedSpeed[3]+LimitedSpeedAff[3])<=MaxSpeed[3])?(LimitedSpeed[3]-int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedSpeed[3];
                }
                else
                {
                    LimitedPwm[0] = ((LimitedPwm[0]+LimitedPwmAff[0])<=MaxPwmSpeed[0])?(LimitedPwm[0]-int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedPwm[0];
                    LimitedPwm[1] = ((LimitedPwm[1]+LimitedPwmAff[1])<=MaxPwmSpeed[1])?(LimitedPwm[1]-int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedPwm[1];
                    LimitedPwm[2] = ((LimitedPwm[2]+LimitedPwmAff[2])<=MaxPwmSpeed[2])?(LimitedPwm[2]-int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedPwm[2];
                    LimitedPwm[3] = ((LimitedPwm[3]+LimitedPwmAff[3])<=MaxPwmSpeed[3])?(LimitedPwm[3]-int8_t(double(SubFFDMsgs.axis[3]-AxisMinValue)/AxisMinValue)):LimitedPwm[3];
                }
            }
            if (SubFFDMsgs.axis[6]!=0)
            {
                if (SpeedControl==1) for (int i=0;i<4;++i) LimitedSpeedAff[i] = ((LimitedSpeed[i]+LimitedSpeedAff[i])<=MaxSpeed[i])?(LimitedSpeedAff[i]+int8_t(SubFFDMsgs.axis[6]/AxisMaxValue)*LimitedSpeedAffStep):LimitedSpeedAff[i];
                else for (int i=0;i<4;++i) LimitedPwmAff[i] = ((LimitedPwm[i]+LimitedPwmAff[i])<=MaxPwmSpeed[i])?(LimitedPwmAff[i]+int8_t(SubFFDMsgs.axis[6]/AxisMaxValue)*LimitedPwmAffStep):LimitedPwmAff[i];
            }
            if (SubFFDMsgs.axis[7]!=0)
            {
                if (SpeedControl==1) LimitedSpeedAffStep = (LimitedSpeedAffStep<=10)?(LimitedSpeedAffStep-int8_t(SubFFDMsgs.axis[7]/AxisMaxValue)):LimitedSpeedAffStep;
                else LimitedPwmAffStep = (LimitedPwmAffStep<=10)?(LimitedPwmAffStep-int8_t(SubFFDMsgs.axis[7]/AxisMaxValue)):LimitedPwmAffStep;
            }
            if (ControlMode==SubFFDMsgs.control_mode)
            {
                this->FFDMsg2SmbusCarBaseMsg(SubFFDMsgs, SmbusBaseMsgs);
                SmbusBaseMsgPub->publish(SmbusBaseMsgs);
            }
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto FFDSmbusNode = std::make_shared<FFDireMsgRecSmbusBaseMsgPubNode>("FFDSmbusBaseNode");
    rclcpp::spin(FFDSmbusNode);
    rclcpp::shutdown();
    return 0;
}
