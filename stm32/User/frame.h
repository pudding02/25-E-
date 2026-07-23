#ifndef __FRAME_PARSER_H__
#define __FRAME_PARSER_H__

#include "usart.h"
#include <stdint.h>
#include <stdbool.h>

// UART协议相关定义
// 串口发送相关结构体
typedef struct
{
	uint8_t	head1;							// 帧头1
	uint8_t	head2;							// 帧头2
	uint8_t length;							// 有效数据长度
	uint8_t cnt;							// 总数据长度
	uint8_t data[40];						// 有效数据数组
	uint8_t transmit_data[50];	// 实际发送的数组 附带上帧头1 帧头2 有效数据长度位 校验位
}DataTransmit;

// 串口接收相关结构体
typedef struct
{
	uint8_t	head1;							// 帧头1
	uint8_t	head2;							// 帧头2
	uint8_t length;							// 有效数据长度
	uint8_t cnt;							// 总数据长度
	uint8_t state;							// 接收状态
	uint8_t i;                              // 有效数据下标
	uint8_t receive_data[50];				// 实际接收的数组
	uint8_t data;                           
	uint8_t complete;                       // 接收完成标志位
}DataReceive;

// 接收数据解析相关结构体
typedef struct
{
	uint16_t	x;							// 目标x轴坐标
	uint16_t	y;							// 目标y轴坐标
	uint8_t   flag;						// 目标标志位
}TargetProperty;

// 数据传输相关函数
void Data_Transmit_Init(DataTransmit *data, uint8_t head1, uint8_t head2, uint8_t length);
void Data_Pack(DataTransmit *data);

// 数据接收相关函数  
void Data_Receive_Init(DataReceive *data, uint8_t head1, uint8_t head2);
void Data_Receive(DataReceive *data, uint8_t buf);
void Data_Receive_Custom(DataReceive *data, uint8_t buf);

// 目标解析相关函数
void Target_Init(TargetProperty *target);
uint8_t Target_Parse(DataReceive *data, TargetProperty *target, uint8_t cmd_data[4]);

// 电机命令执行函数
void Execute_Motor_Command(uint8_t cmd_data[4]);

// DMA发送相关函数
HAL_StatusTypeDef UART_SendData_DMA(const char* data, uint16_t len);
void UART_CheckTransmissionComplete(void);

#endif // __FRAME_PARSER_H__
