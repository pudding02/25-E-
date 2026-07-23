#ifndef __DATOU_H
#define __DATOU_H

#include "usart.h"
#include "dma.h"  // 添加DMA头文件引用

void set_xzero(void);
void set_yzero(void);
void set_zero_pos_x(void);
void set_zero_pos_y(void);
void zero_param_x(void);
void zero_param_y(void);
void pos_control_x(float angle_deg);
void pos_control_y(float angle_deg);
void stop(void);
void Remove(void);

#endif
