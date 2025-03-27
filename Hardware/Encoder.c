/*
 * @Author: 星必尘Sguan
 * @Date: 2025-03-27 15:09:01
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-03-27 22:08:02
 * @FilePath: \demo_MyCar\Hardware\Encoder.c
 * @Description: 磁编码器
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Encoder.h"

extern volatile uint32_t uwTick;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

static uint32_t motor_tick;
int Encoder_Left;
int Encoder_Right;


int Read_MotorCounter(TIM_HandleTypeDef *htim)
{
    int32_t temp =(short)__HAL_TIM_GetCounter(htim);
    __HAL_TIM_SetCounter(htim,0);
    return temp;
}

void Read_Speed(void)
{
    if (uwTick-motor_tick<10)
    {
        return;
    }
    motor_tick=uwTick;
    Encoder_Left = -Read_MotorCounter(&htim2);
    Encoder_Right = Read_MotorCounter(&htim4);
}


