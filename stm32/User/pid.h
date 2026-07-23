/**
 * @file    pid.h
 * @brief   双轴PID控制器 — K230+STM32F407电赛E题复刻
 * @note    支持粗调/精调两套参数，根据偏差大小自动切换
 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float Kp;           /* 比例增益 */
    float Ki;           /* 积分增益 */
    float Kd;           /* 微分增益 */

    float setpoint;     /* 目标值（通常为0，即画面中心） */
    float integral;     /* 积分累加 */
    float prev_error;   /* 上一次误差 */
    float integral_max; /* 积分限幅 */
    float output_max;   /* 输出限幅 */
} PID_HandleTypeDef;


void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd,
              float integral_max, float output_max);
float PID_Calculate(PID_HandleTypeDef *pid, float input, float dt);
void PID_Reset(PID_HandleTypeDef *pid);
void PID_SetGains(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd);

#endif /* __PID_H */
