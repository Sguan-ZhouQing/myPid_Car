#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"

int Motor_Abs(int speed);
void Load(int motor1,int motor2);
void Limit(int *motor1,int *motor2);

#endif
