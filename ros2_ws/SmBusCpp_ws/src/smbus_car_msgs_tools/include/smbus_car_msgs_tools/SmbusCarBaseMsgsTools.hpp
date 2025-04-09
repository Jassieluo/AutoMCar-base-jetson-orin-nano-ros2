#include <smbus_car_msgs/msg/smbus_car_base_msgs.hpp>
#include <stdint.h>

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

int WheelDirection[4] = {-1, 1, 1, -1}; // k=+-1
int ForceDirection[4] = {1, -1, -1, 1}; // k=+-1
int8_t MaxSpeed[4] = {50, 50, 50, 50};
int8_t MaxPwmSpeed[4] = {100, 100, 100, 100};

void SmbusCarBaseMsgsInit(smbus_car_msgs::msg::SmbusCarBaseMsgs &SmbusBaseMsg);
