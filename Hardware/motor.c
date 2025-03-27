/*
 * @Author: 星必尘Sguan
 * @Date: 2025-03-27 12:09:35
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-03-27 20:50:11
 * @FilePath: \demo_MyCar\Hardware\motor.c
 * @Description: 电机驱动代码
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "motor.h"

#define PWM_Max 7200
#define PWM_Min -7200
extern TIM_HandleTypeDef htim1;

#define Motor_AIN1 GPIO_PIN_13
#define Motor_AIN1_Port GPIOB
#define Motor_AIN2 GPIO_PIN_12
#define Motor_AIN2_Port GPIOB
#define Motor_BIN1 GPIO_PIN_14
#define Motor_BIN1_Port GPIOB
#define Motor_BIN2 GPIO_PIN_15
#define Motor_BIN2_Port GPIOB


/**
 * @description: 取速度的的绝对值，得到速度的大小
 * @param {int} speed
 * @return {*}
 */
int Motor_Abs(int speed)
{
    if (speed>0){
        return speed;
    }
    else{
        return -speed;
    }
}


/**
 * @description: 双电机正反转及转速PWM脉宽调控
 * @param {int} motor1
 * @param {int} motor2
 * @return {*}
 */
void Load(int motor1,int motor2)
{
    if (motor1<0)
    {
        HAL_GPIO_WritePin(Motor_AIN1_Port,Motor_AIN1,GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_AIN2_Port,Motor_AIN2,GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(Motor_AIN1_Port,Motor_AIN1,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_AIN2_Port,Motor_AIN2,GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,Motor_Abs(motor1));


    if (motor2<0)
    {
        HAL_GPIO_WritePin(Motor_BIN1_Port,Motor_BIN1,GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_BIN2_Port,Motor_BIN2,GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(Motor_BIN1_Port,Motor_BIN1,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_BIN2_Port,Motor_BIN2,GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,Motor_Abs(motor2));
}



void Limit(int *motor1,int *motor2)
{
    *motor1 = *motor1>PWM_Max?PWM_Max:(*motor1<PWM_Min?PWM_Min:*motor1);
    *motor2 = *motor2>PWM_Max?PWM_Max:(*motor2<PWM_Min?PWM_Min:*motor2);
}

