#include <rclcpp/rclcpp.hpp>
#include <smbus_car_msgs_tools/SmbusCarBaseMsgsTools.hpp>
#include <iomanip>
#include <cstdint>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstring>

// 设置I2C总线号
#define I2C_BUS 1

// 电机类型及编码方向极性
const uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
const uint8_t MotorEncoderPolarity = 0;

// void writeRegister(int file, uint8_t regAddr, const void* data, size_t dataSize) {
//     uint8_t buffer[1 + dataSize];
//     buffer[0] = regAddr;
//     std::memcpy(buffer + 1, data, dataSize);
//     if (write(file, buffer, 1 + dataSize) != 1 + dataSize) {
//         std::cerr << "Failed to write to I2C device" << std::endl;
//     }
// }

// void readRegister(int file, uint8_t regAddr, void* data, size_t dataSize) {
//     if (write(file, &regAddr, 1) != 1) {
//         std::cerr << "Failed to select register" << std::endl;
//         return;
//     }
//     if (read(file, data, dataSize) != dataSize) {
//         std::cerr << "Failed to read from I2C device" << std::endl;
//     }
// }

void writeRegister(int file, uint8_t regAddr, const void* data, size_t dataSize) {
    std::vector<uint8_t> buffer(1 + dataSize);
    buffer[0] = regAddr;
    std::memcpy(buffer.data() + 1, data, dataSize);
    ssize_t bytesWritten = write(file, buffer.data(), 1 + dataSize);
    if (bytesWritten != static_cast<ssize_t>(1 + dataSize)) {
        std::cerr << "Failed to write to I2C device: " << strerror(errno) << std::endl;
    }
}
 
void readRegister(int file, uint8_t regAddr, void* data, size_t dataSize) {
    if (write(file, &regAddr, 1) != 1) {
        std::cerr << "Failed to select register: " << strerror(errno) << std::endl;
        return;
    }
    ssize_t bytesRead = read(file, data, dataSize);
    if (bytesRead != static_cast<ssize_t>(dataSize)) {
        std::cerr << "Failed to read from I2C device: " << strerror(errno) << std::endl;
    }
}

class SmbusCtrMsgSub : public :: rclcpp::Node
{
    private:
        smbus_car_msgs::msg::SmbusCarBaseMsgs SmbusMsgs;
        rclcpp::Subscription<smbus_car_msgs::msg::SmbusCarBaseMsgs>::SharedPtr SmbusMsgSub;
        int file;
        std::string filename;
        int lantency;

    public:
        SmbusCtrMsgSub(const std::string NodeName, int file_, const std::string TopicName="SmbusCarBaseMsgPub", int I2cBusNum=I2C_BUS, int lantency_=10,int cL=10):Node(NodeName)
        {
            lantency = lantency_;
            file = file_;
            if (file < 0) 
            {
                RCLCPP_INFO(this->get_logger(), "file<0, Failed to open the i2c bus\n");
                exit(EXIT_FAILURE);
            }

            if (ioctl(file, I2C_SLAVE, MOTOR_ADDR) < 0) 
            {
                RCLCPP_INFO(this->get_logger(), "Failed to acquire bus access and/or talk to slave.\n");
                exit(EXIT_FAILURE);
            }

            writeRegister(file, MOTOR_TYPE_ADDR, &MotorType, 1);
            usleep(500000);
            writeRegister(file, MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1);
            SmbusMsgSub = this->create_subscription<smbus_car_msgs::msg::SmbusCarBaseMsgs>(TopicName, cL, std::bind(&SmbusCtrMsgSub::SmbusSpeedPwmCtr, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "%s is Running.\n", NodeName.c_str());
        }

        void SmbusSpeedPwmCtr(const smbus_car_msgs::msg::SmbusCarBaseMsgs &SmbusBaseMsgs)
        {
            uint16_t voltage;
            readRegister(file, ADC_BAT_ADDR, &voltage, 2);
            RCLCPP_INFO(this->get_logger(), "V = %d mV\n", static_cast<int>(voltage));
            int32_t encode[4];
            readRegister(file, MOTOR_ENCODER_TOTAL_ADDR, encode, sizeof(encode));
            RCLCPP_INFO(this->get_logger(), "Encode1 = %d, Encode2 = %d, Encode3 = %d, Encode4 = %d\n", encode[0], encode[1], encode[2], encode[3]);
            if (SmbusBaseMsgs.speed_control==1)
            {
                int8_t speed[4] = {0};
                for (int i=0;i<4;++i) speed[i] = (SmbusBaseMsgs.speed[i]<=MaxSpeed[i])?int8_t(SmbusBaseMsgs.speed[i]):int8_t(MaxSpeed[i]/5);
                writeRegister(file, MOTOR_FIXED_SPEED_ADDR, speed, 4);
                usleep(lantency * 1000);
            }
            else
            {
                int8_t pwmSpeed[4] = {0};
                for (int i=0;i<4;++i) pwmSpeed[i] = (SmbusBaseMsgs.pwm[i]<=MaxPwmSpeed[i])?int8_t(SmbusBaseMsgs.pwm[i]):int8_t(MaxPwmSpeed[i]/5);
                writeRegister(file, MOTOR_FIXED_PWM_ADDR, pwmSpeed, 4);
            }
        }
};


int main(int argc, char** argv) {
    int file=-1;
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", I2C_BUS);
    file = open(filename, O_RDWR);
    rclcpp::init(argc, argv);
    auto SmbusCtrMsgSubNode = std::make_shared<SmbusCtrMsgSub>("SmbusCtrMsgSuber", file);
    rclcpp::spin(SmbusCtrMsgSubNode);
    close(file);
    rclcpp::shutdown();
    return 0;
}
