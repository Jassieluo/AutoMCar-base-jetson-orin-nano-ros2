#include <iostream>
#include <iomanip>
#include <cstdint>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstring>

// 设置I2C总线号，通常为1
#define I2C_BUS 7
// 设置四路电机驱动模块的I2C地址
#define MOTOR_ADDR 0x34
// 寄存器地址
#define ADC_BAT_ADDR 0x00
#define MOTOR_TYPE_ADDR 0x14 // 编码电机类型设置
#define MOTOR_ENCODER_POLARITY_ADDR 0x15 // 设置编码方向极性
#define MOTOR_FIXED_PWM_ADDR 0x1F // 固定PWM控制，属于开环控制
#define MOTOR_FIXED_SPEED_ADDR 0x33 // 固定转速控制，属于闭环控制
#define MOTOR_ENCODER_TOTAL_ADDR 0x3C // 4个编码电机各自的总脉冲值

// 电机类型具体值
#define MOTOR_TYPE_WITHOUT_ENCODER 0
#define MOTOR_TYPE_TT 1
#define MOTOR_TYPE_N20 2
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3 // 磁环每转是44个脉冲 减速比：90 默认

// 电机类型及编码方向极性
const uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
const uint8_t MotorEncoderPolarity = 0;

int8_t MaxSpeed[4] = {50, 50, 50, 50};
int8_t MaxPwmSpeed[4] = {100, 100, 100, 100};


// 用于写入寄存器地址和数据的函数
void writeRegister(int file, uint8_t regAddr, const void* data, size_t dataSize) {
    uint8_t buffer[1 + dataSize];
    buffer[0] = regAddr;
    std::memcpy(buffer + 1, data, dataSize);
    if (write(file, buffer, 1 + dataSize) != 1 + dataSize) {
        std::cerr << "Failed to write to I2C device" << std::endl;
    }
}

// 用于读取寄存器数据的函数
void readRegister(int file, uint8_t regAddr, void* data, size_t dataSize) {
    if (write(file, &regAddr, 1) != 1) {
        std::cerr << "Failed to select register" << std::endl;
        return;
    }
    if (read(file, data, dataSize) != dataSize) {
        std::cerr << "Failed to read from I2C device" << std::endl;
    }
}

int main() {
    // 打开I2C设备
    int file;
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", I2C_BUS);
    file = open(filename, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open the i2c bus" << std::endl;
        return 1;
    }

    // 设置从机地址
    if (ioctl(file, I2C_SLAVE, MOTOR_ADDR) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
        return 1;
    }

    // 电机初始化
    writeRegister(file, MOTOR_TYPE_ADDR, &MotorType, 1);
    usleep(500000); // 0.5秒延时
    writeRegister(file, MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1);

    while (true) {
        // 读取电池电压
        uint16_t voltage;
        readRegister(file, ADC_BAT_ADDR, &voltage, 2);
        std::cout << "V = " << static_cast<int>(voltage) << "mV" << std::endl;

        // 读取编码电机脉冲值
        int32_t encode[4];
        readRegister(file, MOTOR_ENCODER_TOTAL_ADDR, encode, sizeof(encode));
        std::cout << "Encode1 = " << encode[0] << "  Encode2 = " << encode[1]
                  << "  Encode3 = " << encode[2] << "  Encode4 = " << encode[3] << std::endl;

        // 固定转速控制
        int8_t speed1[4] = {80, 80, 80, 80};
        writeRegister(file, MOTOR_FIXED_SPEED_ADDR, speed1, 4);
        sleep(3);

        int8_t speed2[4] = {-80, -80, -80, -80};
        writeRegister(file, MOTOR_FIXED_SPEED_ADDR, speed2, 4);
        sleep(3);
    }

    close(file);
    return 0;
}