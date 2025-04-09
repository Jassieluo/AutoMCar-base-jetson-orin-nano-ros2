#include "SmbusCarBaseMsgsTools.hpp"

void SmbusCarBaseMsgsInit(smbus_car_msgs::msg::SmbusCarBaseMsgs &SmbusBaseMsg)
{
    SmbusBaseMsg.pwm_control = 0;
    SmbusBaseMsg.speed_control = 1;

    SmbusBaseMsg.pwm.push_back(0); //0
    SmbusBaseMsg.pwm.push_back(0); //1
    SmbusBaseMsg.pwm.push_back(0); //2
    SmbusBaseMsg.pwm.push_back(0); //3

    SmbusBaseMsg.speed.push_back(0); //0
    SmbusBaseMsg.speed.push_back(0); //1
    SmbusBaseMsg.speed.push_back(0); //2
    SmbusBaseMsg.speed.push_back(0); //3
}