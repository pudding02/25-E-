/**
 * @file    pid.c
 * @brief   双轴PID控制器实现
 */
#include "pid.h"
#include <string.h>

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd,
              float integral_max, float output_max)
{
    memset(pid, 0, sizeof(PID_HandleTypeDef));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
}

float PID_Calculate(PID_HandleTypeDef *pid, float input, float dt)
{
    float error = pid->setpoint - input;
    float P_out = pid->Kp * error;

    pid->integral += error * dt;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float I_out = pid->Ki * pid->integral;

    float derivative = (error - pid->prev_error) / dt;
    float D_out = pid->Kd * derivative;
    pid->prev_error = error;

    float output = P_out + I_out + D_out;
    if (output >  pid->output_max) output =  pid->output_max;
    if (output < -pid->output_max) output = -pid->output_max;

    return output;
}

void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void PID_SetGains(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    PID_Reset(pid);
}
