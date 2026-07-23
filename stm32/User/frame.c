#include "frame.h"
#include "DATOU.h"
#include "usart.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern uint8_t key_num;  // 从KEY源文件中获取的按键值
// usart1用于接收maixcam, usart2用于发送调试信息

/*************************************************************************************************************
  * @brief  串口发送完成回调
  * @note   该函数被移到main.c中实现，避免符号重复定义
  * @param  huart: 串口句柄
  * @retval None
  */
/* 函数已移到main.c中实现
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    
}
*/

/**********************************************************************************************************************
 * @brief 串口发送数据结构体初始化函数
 * @param[in] data   指向要初始化的DataTransmit结构体的指针
 * @param[in] head1  协议帧头1
 * @param[in] head2  协议帧头2
 * @param[in] length 有效数据的长度
 * @note  此函数用于设置串口发送数据帧的固定部分，并清空数据缓冲区。
 **********************************************************************************************************************/
void Data_Transmit_Init(DataTransmit *data, uint8_t head1, uint8_t head2, uint8_t length)
{
	data -> head1  = head1;
	data -> head2  = head2;
	data -> length = length;
	data -> cnt    = length + 4; // 总长度 = 帧头1 + 帧头2 + 长度位 + 有效数据 + 校验和

	// 清空有效数据缓缓冲区
	for(uint8_t i = 0; i < length; i++)
	{
		data -> data[i] = 0;
	}

	// 清空完整发送帧缓冲区
	for(uint8_t j = 0; j < data -> cnt; j++)
	{
		data -> transmit_data[j] = 0;
	} 
}

/**********************************************************************************************************************
 * @brief 串口发送数据打包函数
 * @param[in,out] data 指向需要打包的DataTransmit结构体的指针
 * @note  此函数将有效数据和协议控制信息（帧头、长度、校验和）组装成一个完整的待发送数据帧。
 **********************************************************************************************************************/
void Data_Pack(DataTransmit *data)
{
	// 填充帧头和长度
	data -> transmit_data[0] = data -> head1;
	data -> transmit_data[1] = data -> head2;
	data -> transmit_data[2] = data -> length;

	// 填充有效数据
	for(uint8_t i = 0; i < data -> length; i++)
	{
		data -> transmit_data[3+i] = data -> data[i];
	}

	uint8_t sum = 0;

    // 计算校验和（从帧头到有效数据的所有字节累加）
    for(uint8_t j = 0; j < data -> length + 3; j++)
	{
		sum = sum + data -> transmit_data[j];
	}

    // 填充校验和
    data -> transmit_data[data -> length + 3] = sum;
}

/**********************************************************************************************************************
 * @brief 串口接收数据结构体初始化函数
 * @param[in] data   指向要初始化的DataReceive结构体的指针
 * @param[in] head1  要识别的协议帧头1
 * @param[in] head2  要识别的协议帧头2
 * @note  此函数用于重置串口接收状态机和缓冲区，为接收新的数据帧做准备。
 **********************************************************************************************************************/
void Data_Receive_Init(DataReceive *data, uint8_t head1, uint8_t head2)
{
	data -> head1  = head1;
	data -> head2  = head2;
	data -> length = 0;     // 有效数据长度
	data -> cnt    = 0;     // 接收数据计数
	data -> state  = 0;     // 接收状态机状态
	data -> i      = 0;     // 辅助计数器
	data -> data   = 0;     // (此行似乎未在后续使用，可考虑移除)
	data -> complete = 0; // 初始化接收完成标志位

	// 清空接收缓冲区
	for(uint8_t j = 0; j < 50; j++)
	{
		data -> receive_data[j] = 0;
	}
}

/**********************************************************************************************************************
 * @brief 串口接收数据处理函数（状态机）
 * @param[in,out] data 指向用于处理接收的DataReceive结构体的指针
 * @param[in]     buf  从串口接收到的单个字节数据
 * @note  此函数是协议解析的核心，通过一个状态机来逐字节地识别和解析数据帧。
 *        应在串口接收中断中被调用。
 **********************************************************************************************************************/
void Data_Receive(DataReceive *data, uint8_t buf)
{
	if(data -> state == 0 && buf == data -> head1)
	{
		data -> state = 1;
		data -> receive_data[0] = buf;
	}
	else if(data -> state == 1 && buf == data -> head2)
	{
		data -> state = 2;
		data -> receive_data[1] = buf;
	}
	else if(data -> state == 2 && buf < 40)
	{
		data -> state = 3;
		data -> length = buf;
		data -> cnt = buf + 5;
		data -> receive_data[2] = buf;
	}
	else if(data -> state == 3 && data -> length > 0)
	{
		data -> length--;
		data -> receive_data[3 + data -> i] = buf;
		data -> i++;

		if(data -> length == 0)
		{
			data -> state = 4;
		}
	}
	else if(data -> state == 4)
	{
		data -> receive_data[3 + data -> i] = buf;
		data -> i++;
		data -> state = 5;
	}
	else if(data -> state == 5 && buf == 0x55)
	{
		data -> receive_data[3 + data -> i] = buf;
		data -> complete = 1;
		data -> state = 0;
		data -> i = 0;
	}
	else
	{
		data -> state = 0;
		data -> i = 0;
	}
}

/**********************************************************************************************************************
 * @brief 目标属性数据结构体初始化函数
 * @param[in] target 指向要初始化的TargetProperty结构体的指针
 * @note  用于清空存储解析结果的结构体。
 **********************************************************************************************************************/
void Target_Init(TargetProperty *target)
{
	target -> x     = 0;
	target -> y     = 0;
	target -> flag  = 0;
}

/**********************************************************************************************************************
 * @brief 串口接收数据解析函数
 * @param[in] data   指向包含待解析数据的DataReceive结构体的指针
 * @param[out] target 指向用于存储解析结果的TargetProperty结构体的指针
 * @param[out] cmd_data 指向用于存储命令数据的4字节数组
 * @return 成功解析自定义格式时返回命令数据第一个字节，否则返回0
 * @note  此函数在接收到一帧完整的数据后被调用，用于校验数据完整性并将数据解析到目标结构体中。
 **********************************************************************************************************************/
uint8_t Target_Parse(DataReceive *data, TargetProperty *target, uint8_t cmd_data[4])
{
    // 显示接收到的原始数据用于调试
    char raw_msg[150];
    sprintf(raw_msg, "原始接收数据(%d字节): ", data->cnt);
    for(uint8_t i = 0; i < data->cnt; i++) {
        char byte_str[6];
        sprintf(byte_str, "%02X ", data->receive_data[i]);
        strcat(raw_msg, byte_str);
    }
    strcat(raw_msg, "\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)raw_msg, strlen(raw_msg), HAL_MAX_DELAY);
    
    // 优先检查标准协议格式：AA BB LEN PAYLOAD CHK 55
    if(data->cnt == 10 && data->receive_data[data->cnt-1] == 0x55) {
        uint8_t received_checksum = data->receive_data[data->cnt - 2];
        
        // 计算校验和（除校验和和帧尾）
        uint8_t sum_all = 0;
        for(uint8_t i = 0; i < data->cnt - 2; i++) {
            sum_all += data->receive_data[i];
        }
        
        if(sum_all == received_checksum) {
            // 校验成功，解析数据（大端序）
            target->x = (data->receive_data[3] << 8) | data->receive_data[4];
            target->y = (data->receive_data[5] << 8) | data->receive_data[6];
            target->flag = data->receive_data[7];
            
            char result_msg[80];
            sprintf(result_msg, "Standard format parsed: x=%d, y=%d, flag=%d\r\n",  
                    target->x, target->y, target->flag);   //标准格式解析成功
            HAL_UART_Transmit(&huart2, (uint8_t*)result_msg, strlen(result_msg), HAL_MAX_DELAY);
        }
        else {
            char error_msg[100];
            sprintf(error_msg, "Standard format checksum error: calc=0x%02X, recv=0x%02X\r\n", 
                    sum_all, received_checksum); // 标准格式校验和错误: 计算=0x%02X, 接收=0x%02X
            HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        }
        return 0;
    }
    
  // 检查无帧尾格式
    if(data->cnt == 9 && data->receive_data[data->cnt-1] != 0x55) {
        uint8_t received_checksum = data->receive_data[data->cnt - 1];
        
        // 计算所有字节校验和（除最后的校验和）
        uint8_t sum_all = 0;
        for(uint8_t i = 0; i < data->cnt - 1; i++) {
            sum_all += data->receive_data[i];
        }
        
        if(sum_all == received_checksum) {
            // 校验成功，解析数据（大端序）
            target->x = (data->receive_data[3] << 8) | data->receive_data[4];
            target->y = (data->receive_data[5] << 8) | data->receive_data[6];
            target->flag = data->receive_data[7];
            
            char result_msg[80];
            sprintf(result_msg, "No trailer format parsed: x=%d, y=%d, flag=%d\r\n", 
                    target->x, target->y, target->flag);
            HAL_UART_Transmit(&huart2, (uint8_t*)result_msg, strlen(result_msg), HAL_MAX_DELAY);
        }
        else {
            char error_msg[100];
            sprintf(error_msg, "无帧尾格式校验和错误: 计算=0x%02X, 接收=0x%02X\r\n", 
                    sum_all, received_checksum);
            HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        }
        return 0;
    }
    
    // 检查自定义格式：AA 55 DATA(4字节) CHK FF
    if(data->cnt == 8 && data->receive_data[0] == 0xAA && data->receive_data[1] == 0x55 && data->receive_data[7] == 0xFF) {
        uint8_t received_checksum = data->receive_data[6];
        
        // 计算校验和（帧头+数据部分）
        uint8_t sum_all = 0;
        for(uint8_t i = 0; i < 6; i++) { // 0-5字节（AA 55 + 4字节数据）
            sum_all += data->receive_data[i];
        }
        
        if(sum_all == received_checksum) {
            // 校验成功，解析4字节数据
            cmd_data[0] = data->receive_data[2];
            cmd_data[1] = data->receive_data[3];
            cmd_data[2] = data->receive_data[4];
            cmd_data[3] = data->receive_data[5];
            
            char result_msg[100];
            sprintf(result_msg, "Custom format parsed: 0x%02X 0x%02X 0x%02X 0x%02X\r\n", 
                    cmd_data[0], cmd_data[1], cmd_data[2], cmd_data[3]);
            HAL_UART_Transmit(&huart2, (uint8_t*)result_msg, strlen(result_msg), HAL_MAX_DELAY);
            
            // 返回命令数据的第一个字节
            return cmd_data[0];
        }
        else {
            char error_msg[100];
            sprintf(error_msg, "Custom format checksum error: calc=0x%02X, recv=0x%02X\r\n", 
                    sum_all, received_checksum);
            HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        }
        return 0;
    }
    
    // 未知格式
    char debug_msg[100];
    sprintf(debug_msg, "未知格式: %d 字节, 最后一字节=0x%02X\r\n", 
            data->cnt, data->receive_data[data->cnt-1]);
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
    
    return 0; // 解析失败返回0
}

/**********************************************************************************************************************
 * @brief 自定义格式串口接收数据处理函数（状态机）
 * @param[in,out] data 指向用于处理接收的DataReceive结构体的指针
 * @param[in]     buf  从串口接收到的单个字节数据
 * @note  专门处理 AA 55 DATA(4字节) CHK FF 格式的数据帧
 **********************************************************************************************************************/
void Data_Receive_Custom(DataReceive *data, uint8_t buf)
{
	// 添加调试信息
	char debug_msg[60];
	sprintf(debug_msg, "State: %d, Byte: 0x%02X, Index: %d\r\n", data->state, buf, data->i);
	HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);
	
	// 在任何状态下，如果收到0xAA都强制重新开始（防止状态机混乱）
	if(buf == 0xAA) {
		data->state = 1;
		data->receive_data[0] = buf;
		data->i = 1;
		data->complete = 0;
		char reset_msg[40];
		sprintf(reset_msg, "Frame sync: 0xAA detected\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)reset_msg, strlen(reset_msg), 100);
		return;
	}
	
	switch(data->state) {
		case 0: // 等待帧头1 (0xAA)
			if(buf == 0xAA) {
				data->state = 1;
				data->receive_data[0] = buf;
				data->i = 1;
				data->complete = 0;
			}
			break;
			
		case 1: // 等待帧头2 (0x55)
			if(buf == 0x55) {
				data->state = 2;
				data->receive_data[1] = buf;
				data->i = 2;
				char head2_msg[30];
				sprintf(head2_msg, "Got 0x55, state->2\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)head2_msg, strlen(head2_msg), 100);
			} else {
				// 如果不是期望的第二个帧头，重新开始
				data->state = 0;
				data->i = 0;
				data->complete = 0;
				char err_msg[50];
				sprintf(err_msg, "Expected 0x55, got 0x%02X\r\n", buf);
				HAL_UART_Transmit(&huart2, (uint8_t*)err_msg, strlen(err_msg), 100);
			}
			break;
			
		case 2: // 接收4字节数据
			data->receive_data[data->i] = buf;
			data->i++;
			if(data->i == 6) { // 接收完4字节数据
				data->state = 3;
			}
			break;
			
		case 3: // 接收校验位
			data->receive_data[data->i] = buf;
			data->i++;
			data->state = 4;
			break;
			
		case 4: // 等待帧尾 (0xFF)
			if(buf == 0xFF) {
				data->receive_data[data->i] = buf;
				data->cnt = 8; // 固定8字节
				data->complete = 1;
				data->state = 0;
				data->i = 0;
				char complete_msg[30];
				sprintf(complete_msg, "Frame complete!\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)complete_msg, strlen(complete_msg), 100);
			} else {
				// 如果不是期望的帧尾，重新开始寻找帧头
				data->state = 0;
				data->i = 0;
				data->complete = 0;
			}
			break;
			
		default:
			data->state = 0;
			data->i = 0;
			data->complete = 0;
			break;
	}
}

// 串口忙标志，静态变量，防止外部直接访问

     
static char uart1_txbuf[64];                // 持久缓冲

/**
  * @brief  使用普通方式发送数据到USART1/USART2
  * @param  data: 要发送的数据指针
  * @param  len: 数据长度
  * @retval HAL状态
  */
HAL_StatusTypeDef UART_SendData(const char* data, uint16_t len)
{
  memcpy(uart1_txbuf, data, len);
  return HAL_UART_Transmit(&huart2, (uint8_t*)uart1_txbuf, len, 100);
}

/**
  * @brief  检查并更新串口传输完成状态
  * @param  None
  * @retval None
  */
void UART_CheckTransmissionComplete(void)
{
    
}

/* HAL_UART_TxCpltCallback 已在 main.c 中实现
 * 在这里删除空实现，避免符号冲突
 */

/**
  * @brief  串口接收半完成回调
  * @param  huart: 串口句柄
  * @retval None
  */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) 
{
    // 基本的接收半完成处理
}

/**
  * @brief  串口错误回调
  * @param  huart: 串口句柄
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) 
{
    // 基本的错误处理
}

/**
  * @brief  串口接收事件回调
  * @note   此函数在使用HAL_UARTEx_ReceiveToIdle_IT时会被调用
  * @param  huart: 串口句柄
  * @param  size: 接收到的数据大小
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) 
{

}

/**********************************************************************************************************************
 * @brief 执行电机命令函数
 * @param[in] cmd_data 4字节命令数据数组
 * @note  根据不同的命令数据调用对应的DATOU函数
 **********************************************************************************************************************/



 void Execute_Motor_Command(uint8_t cmd_data[4])
{
    // 根据第一个字节判断命令类型
    switch(cmd_data[0]) {
        case 0x01:  // 电机控制命令1
            {
                char debug_msg[80];
                sprintf(debug_msg, "执行命令0x01: 电机动作1\r\n");
                HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
                
                // TODO: 根据实际需求调用DATOU函数
                // 例如：HAL_UART_Transmit(&huart3, (uint8_t*)turn_z, sizeof(turn_z), 100);
            }
            break;
            
        case 0x02:  // 速度控制命令
            {
                uint8_t addr = cmd_data[1];       // 电机地址
                uint8_t dir = cmd_data[2];        // 方向
                uint8_t vel_param = cmd_data[3];  // 速度参数（可能需要扩展）
                
                char debug_msg[80];
                sprintf(debug_msg, "执行速度命令: addr=0x%02X, dir=%d, vel_param=%d\r\n", addr, dir, vel_param);
                HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
                
                // TODO: 调用DATOU速度控制函数
                // uint16_t vel = vel_param * 100;  // 根据需要转换速度值
                // Emm_V5_Vel_Control(addr, dir, vel, 10, false);
            }
            break;
            
        case 0x03:  // 停止旋转
            {
              stop();
            }
            break;
            
        case 0x04:  // 停止命令
            {
              stop();
            }
            break;
            
        case 0x05:  // 回零命令
            {
                uint8_t addr = cmd_data[1];       // 电机地址
                uint8_t o_mode = cmd_data[2];     // 回零模式
                bool snF = (cmd_data[3] != 0);    // 同步标志
                
                char debug_msg[80];
                sprintf(debug_msg, "执行回零命令: addr=0x%02X, mode=%d, sync=%d\r\n", addr, o_mode, snF);
                HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
                
                // TODO: 调用DATOU回零函数
                // Emm_V5_Origin_Trigger_Return(addr, o_mode, snF);
            }
            break;
            
        case 0x11:  // 你原来的0x11命令
            {
                char debug_msg[80];
                sprintf(debug_msg, "执行0x11命令: 所有数据都是0x11\r\n");
                HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
                
                // TODO: 根据实际需求调用相应函数
                // 例如：特定的电机动作或状态查询
            }
            break;
            
        default:
            {
                char debug_msg[80];
                sprintf(debug_msg, "未知命令: 0x%02X\r\n", cmd_data[0]);
                HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
            }
            break;
    }
}