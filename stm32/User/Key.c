#include "key.h"
#include "usart.h"
volatile uint8_t key_num = 0; // 按键编号

uint8_t Key_getnum()
{
    if(HAL_GPIO_ReadPin(key1_GPIO_Port,key1_Pin)==0)
    {
        HAL_Delay(10);
        key_num=1;
    }
    if(HAL_GPIO_ReadPin(key2_GPIO_Port,key2_Pin)==0)
    {
        HAL_Delay(10);
        key_num=2;
    }
    if(HAL_GPIO_ReadPin(key3_GPIO_Port,key3_Pin)==0)
    {
        HAL_Delay(10);
        key_num=3;
    }
    if(HAL_GPIO_ReadPin(key4_GPIO_Port,key4_Pin)==0)
    {
        HAL_Delay(10);
        key_num=4;
    }
    if(HAL_GPIO_ReadPin(key5_GPIO_Port,key5_Pin)==0)
    {
        HAL_Delay(10);
        key_num=5;
    }
    return key_num;
    
}