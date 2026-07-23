#include "DATOU.h"
#include "usart.h"
#include "gpio.h"
#include <stdint.h>
#include <math.h>

#define STEP_ANGLE_DEG     1.8f    // 步距角 (°)
#define MICROSTEP          16      // 微步数
#define REDUCTION_RATIO    1.0f    // 减速比
// 每转脉冲数（四舍五入）
#define PPR  ((uint32_t)((360.0f / STEP_ANGLE_DEG) * MICROSTEP / REDUCTION_RATIO + 0.5f))

void zero_param_x(void) 
{ // 00 1E 30RPM， 00 0F 15RPM
    uint8_t zero_prame[] = {0x01,  0x4C,  0xAE,  0x01, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x27, 0x10, 0x01,  0x2C,  0x03,  0x20,  0x00,  0x3C,  0x00,  0x6B};
    UART6_Transmit(zero_prame, sizeof(zero_prame), 100);
}

void zero_param_y(void) 
{
// 0x01, 0x2C : 300RPM
// 0x00, 0x1E : 30RPM
uint8_t zero_prame[] = {0x01,  0x4C,  0xAE,  0x01, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x27, 0x10, 0x01,  0x2C,  0x03,  0x20,  0x00,  0x3C,  0x00,  0x6B};
UART3_Transmit(zero_prame, sizeof(zero_prame), 100);
}

void set_yzero(void)
{
// 返回零点
uint8_t set_zero[] = {0x01, 0x9A, 0x00, 0x00, 0x6B};
//UART3_Transmit(set_zero, sizeof(set_zero), 100);
UART6_Transmit(set_zero, sizeof(set_zero), 100); 
}

void set_xzero(void)
{
    uint8_t set_zeroo[] = {0x01, 0x9A, 0x00, 0x00, 0x6B};
    UART3_Transmit(set_zeroo, sizeof(set_zeroo), 100);
}

void pos_control_x(float angle_deg)
{
//位置指令
/* 命令格式：地址 + 0xFD + 方向 + 速度+ 加速度 + 脉冲数 + 相对/绝对模式标志 + 多机同步标志 + 校验字节
   例:01, 0xFD, 0X01, 0x05, 0xDC, 0x00, 0x01, 0x00, 0x7D, 0x00, 0x01, 0x00, 0x6B
   数据解析：01表示旋转方向为CCW（00表示CW），
   05 DC表示速度为0x05DC = 1500(RPM)，
   00表示加速度档位为0x00 = 0，
   00 00 7D 00表示脉冲数为0x00007D00 = 32000个，
   00表示相对位置模式（01表示绝对位置模式），
   00表示不启用多机同步（01表示启用）
   00 6B表示校验字节

   角度 (°) = 脉冲数 ÷ PPR × 360°
   PPR = (360° ÷ 步距角) × 微步数 ÷ 减速比
   脉冲数 = θ ÷ 360° × PPR
*/

// 1) 角度 → 脉冲数
    float pulses_f = angle_deg / 360.0f * (float)PPR;
    uint32_t pulses = (uint32_t)(pulses_f + 0.5f);

    // 2) 拆成高/低 16 位
    uint16_t pos1 = (uint16_t)((pulses >> 16) & 0xFFFF);
    uint16_t pos2 = (uint16_t)( pulses        & 0xFFFF);

    // 3) 构造帧，最后一位为固定校验字 0x68
    uint8_t cmd[13] = {
        0x01,       // [0] 地址
        0xFD,       // [1] 功能码（位置模式）
        0x01,       // [2] 方向（0x00=CW, 0x01=CCW）
        0x01, 0x2C, // [3-4] 速度 300 RPM
        0x00,       // [5] 加速度
        // [6..9] 脉冲数占位：
        0x00, 0x00, 0x00, 0x00,
        0x01,       // [10] 相对/绝对 模式（0=相对,1=绝对）
        0x00,       // [11] 多机同步 标志
        0x6B        // [12] 校验字（固定）
    };

    // 4) 覆盖脉冲字段
    cmd[6] = (uint8_t)((pos1 >> 8) & 0xFF);
    cmd[7] = (uint8_t)( pos1        & 0xFF);
    cmd[8] = (uint8_t)((pos2 >> 8) & 0xFF);
    cmd[9] = (uint8_t)( pos2        & 0xFF);

    // 5) 发送帧
    UART3_Transmit((uint8_t *)cmd, 13, 100);
}

void pos_control_y(float angle_deg)
{
// 1) 角度 → 脉冲数
    float pulses_f = angle_deg / 360.0f * (float)PPR;
    uint32_t pulses = (uint32_t)(pulses_f + 0.5f);

    // 2) 拆成高/低 16 位
    uint16_t pos1 = (uint16_t)((pulses >> 16) & 0xFFFF);
    uint16_t pos2 = (uint16_t)( pulses        & 0xFFFF);

    // 3) 构造帧，最后一位为固定校验字 0x68
    uint8_t cmd[13] = {
        0x01,       // [0] 地址
        0xFD,       // [1] 功能码（位置模式）
        0x01,       // [2] 方向（0x00=CW, 0x01=CCW）
        0x01, 0x2C, // [3-4] 速度 300 RPM
        0x00,       // [5] 加速度
        // [6..9] 脉冲数占位：
        0x00, 0x00, 0x00, 0xB4,
        0x01,       // [10] 相对/绝对 模式（0=相对,1=绝对）
        0x00,       // [11] 多机同步 标志
        0x6B        // [12] 校验字（固定）
    };

    // 4) 覆盖脉冲字段
    cmd[6] = (uint8_t)((pos1 >> 8) & 0xFF);
    cmd[7] = (uint8_t)( pos1        & 0xFF);
    cmd[8] = (uint8_t)((pos2 >> 8) & 0xFF);
    cmd[9] = (uint8_t)( pos2        & 0xFF);

    // 5) 发送帧
    UART6_Transmit(cmd, 13, 100);
}

void stop(void)
{
uint8_t txcmd[] = {0x01, 0xFE, 0x98, 0x00, 0x6B};
UART3_Transmit(txcmd, sizeof(txcmd), 100);
UART6_Transmit(txcmd, sizeof(txcmd), 100);
}

void set_zero_pos_x(void)
{
    uint8_t set_zero_pos[] = {0x01, 0x93, 0x88, 0x01, 0x6B};
    UART6_Transmit(set_zero_pos, sizeof(set_zero_pos), 100);
}
void set_zero_pos_y(void)
{
    uint8_t set_zero_pos[] = {0x01, 0x93, 0x88, 0x01, 0x6B};
    UART3_Transmit(set_zero_pos, sizeof(set_zero_pos), 100);
}

void Remove(void)
{
    set_xzero();
    set_yzero();
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); 
}
