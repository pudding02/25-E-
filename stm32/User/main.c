/**
 * @file    main.c
 * @brief   K230+STM32F407 电赛E题复刻 — 主程序
 *
 * 与原项目的区别:
 *   1. 使用 PID 闭环控制替代开环固定脉冲
 *   2. 通过 UART4 接收 K230 发来的像素偏差 (dx,dy)
 *   3. 像素偏差 → PID计算 → 角度输出 → 步进电机执行
 *   4. 连续对准后触发激光(可选)
 *
 * 通信协议 (K230 → UART4):
 *   "dx,dy,flag1,flag2\n"    正常追踪
 *   "404,404,0,0\n"          目标丢失
 *
 * 硬件连接:
 *   UART4  → K230 (接收视觉偏差)
 *   UART3  → X轴步进电机驱动器
 *   UART6  → Y轴步进电机驱动器
 *   UART1  → 调试输出 (USB转TTL)
 *   UART2  → 调试输出 (备用)
 *   PB9    → 电机使能
 *   按键1  → 系统复位/归零
 *   按键2  → 开始追踪
 *   按键3  → 停止追踪
 */
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "DATOU.h"
#include "pid.h"
#include "Key.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ===================== PID参数配置 ===================== */

/* 粗调参数 — 偏差较大时快速逼近 */
#define KP_COARSE    0.12f
#define KI_COARSE    0.0005f
#define KD_COARSE    0.02f

/* 精调参数 — 偏差较小时稳定对准 */
#define KP_FINE      0.06f
#define KI_FINE      0.0003f
#define KD_FINE      0.05f

/* 粗调→精调 切换阈值 (像素) */
#define FINE_THRESHOLD    15.0f

/* 对准判定容差 (像素) */
#define ALIGN_TOLERANCE   5.0f

/* 对准持续帧数（避免误触发） */
#define ALIGN_CONFIRM_MS  500

/* PID计算周期 (ms) */
#define PID_DT            0.01f     /* 10ms */

/* 最大角度输出 (度) */
#define MAX_ANGLE_OUTPUT  15.0f

/* 像素→角度 比例因子 (°/pixel)
 * 计算: 水平视野角60° / 800像素 = 0.075°/pixel
 * 实际需根据镜头标定 */
#define PIXEL_TO_DEG_X    0.075f
#define PIXEL_TO_DEG_Y    0.056f    /* 垂直: 45° / 800px */

/* ====================================================== */

/* 串口接收缓冲区 */
uint8_t  uart4_rx_buf[64];
volatile uint8_t  uart4_rx_idx = 0;
volatile uint8_t  uart4_frame_ready = 0;

/* K230发来的偏差数据 */
volatile int16_t  vision_dx = 0;
volatile int16_t  vision_dy = 0;
volatile uint8_t  vision_aligned = 0;
volatile uint8_t  vision_lost = 0;

/* PID控制器 */
PID_HandleTypeDef pid_x;
PID_HandleTypeDef pid_y;

/* 追踪状态 */
typedef enum {
    STATE_IDLE = 0,       /* 待机 */
    STATE_TRACK,          /* 追踪中 */
    STATE_ALIGNED,        /* 已对准 */
    STATE_LOST            /* 目标丢失 */
} TrackState;

volatile TrackState track_state = STATE_IDLE;
volatile uint32_t aligned_start_ms = 0;
volatile uint32_t lost_start_ms = 0;

/* 电机使能引脚: PB9 */
#define MOTOR_EN_PORT    GPIOB
#define MOTOR_EN_PIN     GPIO_PIN_9

/* 激光引脚: PB8 (可选) */
#define LASER_PORT       GPIOB
#define LASER_PIN        GPIO_PIN_8


void SystemClock_Config(void);
void parse_vision_data(uint8_t *buf, uint8_t len);
void track_control_loop(void);


int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    MX_UART4_Init();

    /* 调试信息 */
    char init_msg[] = "\r\n=== K230+STM32F407 追踪系统 v2.0 ===\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)init_msg, strlen(init_msg), 100);

    /* 初始化PID */
    PID_Init(&pid_x, KP_COARSE, KI_COARSE, KD_COARSE, 500.0f, MAX_ANGLE_OUTPUT);
    PID_Init(&pid_y, KP_COARSE, KI_COARSE, KD_COARSE, 500.0f, MAX_ANGLE_OUTPUT);

    /* 使能电机 */
    HAL_GPIO_WritePin(MOTOR_EN_PORT, MOTOR_EN_PIN, GPIO_PIN_SET);

    /* 启动 UART4 接收中断 */
    HAL_UART_Receive_IT(&huart4, &uart4_rx_buf[0], 1);

    char ready_msg[] = "系统就绪，按 KEY2 开始追踪\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)ready_msg, strlen(ready_msg), 100);

    uint32_t last_pid_ms = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* ---- 按键处理 ---- */
        uint8_t key = Key_getnum();

        if (key == 1) {
            /* KEY1: 归零复位 */
            track_state = STATE_IDLE;
            PID_Reset(&pid_x);
            PID_Reset(&pid_y);
            set_xzero();
            set_yzero();
            char msg[] = "归零完成，待机模式\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        }

        if (key == 2) {
            /* KEY2: 开始追踪 */
            if (track_state == STATE_IDLE) {
                track_state = STATE_TRACK;
                PID_Reset(&pid_x);
                PID_Reset(&pid_y);
                uart4_rx_idx = 0;
                uart4_frame_ready = 0;
                vision_lost = 0;

                /* 唤醒K230进入检测模式 */
                uint8_t wakeup = 0xA1;
                HAL_UART_Transmit(&huart4, &wakeup, 1, 100);

                char msg[] = "开始追踪\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            }
        }

        if (key == 3) {
            /* KEY3: 停止追踪 */
            track_state = STATE_IDLE;
            stop();
            HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
            char msg[] = "停止追踪，待机模式\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        }

        /* ---- 处理接收到的视觉数据帧 ---- */
        if (uart4_frame_ready) {
            uart4_frame_ready = 0;
            parse_vision_data(uart4_rx_buf, uart4_rx_idx);
            uart4_rx_idx = 0;
        }

        /* ---- PID控制循环 (10ms周期) ---- */
        if (track_state == STATE_TRACK || track_state == STATE_ALIGNED) {
            if (now - last_pid_ms >= 10) {
                last_pid_ms = now;
                track_control_loop();
            }
        }

        /* ---- 目标丢失超时处理 ---- */
        if (vision_lost && track_state == STATE_TRACK) {
            if (lost_start_ms == 0) {
                lost_start_ms = now;
            } else if (now - lost_start_ms > 2000) {
                track_state = STATE_LOST;
                stop();
                char msg[] = "目标丢失超时，停止电机\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            }
        } else {
            lost_start_ms = 0;
        }

        HAL_Delay(1);
    }
}


/**
 * @brief 解析K230发来的CSV格式数据
 * @param buf  接收缓冲区
 * @param len  数据长度
 *
 * 格式: "dx,dy,flag1,flag2\n"
 * 例:   "-25,18,0,0\n"
 */
void parse_vision_data(uint8_t *buf, uint8_t len)
{
    if (len < 6) return;

    buf[len] = 0;  /* 确保字符串终止 */

    char *token;
    int values[4] = {0};
    int i = 0;

    token = strtok((char*)buf, ",\n\r");
    while (token != NULL && i < 4) {
        values[i++] = atoi(token);
        token = strtok(NULL, ",\n\r");
    }

    vision_dx = (int16_t)values[0];
    vision_dy = (int16_t)values[1];

    /* 检查是否为丢失信号 */
    if (vision_dx == 404 && vision_dy == 404) {
        vision_lost = 1;
        if (track_state == STATE_ALIGNED) {
            track_state = STATE_TRACK;
        }
        return;
    }
    vision_lost = 0;

    /* 检查对准标志 */
    vision_aligned = (uint8_t)values[2];
}


/**
 * @brief PID追踪控制循环 — 每10ms调用一次
 */
void track_control_loop(void)
{
    float dx_f = (float)vision_dx;
    float dy_f = (float)vision_dy;

    /* 根据偏差大小切换粗调/精调参数 */
    float abs_err = (dx_f > 0 ? dx_f : -dx_f) + (dy_f > 0 ? dy_f : -dy_f);

    if (abs_err < FINE_THRESHOLD) {
        pid_x.Kp = KP_FINE;  pid_x.Ki = KI_FINE;  pid_x.Kd = KD_FINE;
        pid_y.Kp = KP_FINE;  pid_y.Ki = KI_FINE;  pid_y.Kd = KD_FINE;
    } else {
        pid_x.Kp = KP_COARSE; pid_x.Ki = KI_COARSE; pid_x.Kd = KD_COARSE;
        pid_y.Kp = KP_COARSE; pid_y.Ki = KI_COARSE; pid_y.Kd = KD_COARSE;
    }

    /* 像素偏差 → 角度命令 */
    float angle_cmd_x = vision_lost ? 0.0f :
                        PID_Calculate(&pid_x, dx_f * PIXEL_TO_DEG_X, PID_DT);
    float angle_cmd_y = vision_lost ? 0.0f :
                        PID_Calculate(&pid_y, dy_f * PIXEL_TO_DEG_Y, PID_DT);

    /* 死区 — 偏差极小时不动作 */
    float abs_angle_x = (angle_cmd_x > 0) ? angle_cmd_x : -angle_cmd_x;
    float abs_angle_y = (angle_cmd_y > 0) ? angle_cmd_y : -angle_cmd_y;

    if (abs_angle_x > 0.05f) {
        pos_control_x(angle_cmd_x);
    }
    if (abs_angle_y > 0.05f) {
        pos_control_y(angle_cmd_y);
    }

    /* 对准检测 & 激光触发 */
    if (vision_aligned) {
        if (track_state != STATE_ALIGNED) {
            if (aligned_start_ms == 0) {
                aligned_start_ms = HAL_GetTick();
            } else if (HAL_GetTick() - aligned_start_ms > ALIGN_CONFIRM_MS) {
                track_state = STATE_ALIGNED;
                HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_SET);
                char msg[] = "对准完成！激光触发\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            }
        }
    } else {
        aligned_start_ms = 0;
        if (track_state == STATE_ALIGNED) {
            track_state = STATE_TRACK;
            HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
        }
    }
}


/**
 * @brief 串口接收完成回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        uint8_t ch = uart4_rx_buf[uart4_rx_idx];

        if (ch == '\n' || uart4_rx_idx >= 63) {
            uart4_frame_ready = 1;
        } else {
            uart4_rx_idx++;
            HAL_UART_Receive_IT(&huart4, &uart4_rx_buf[uart4_rx_idx], 1);
        }
    }
}


void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}


void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
