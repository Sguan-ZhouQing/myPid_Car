#ifndef __PID_H
#define __PID_H

#include "stm32f1xx_hal.h"

int Vertical(float Med,float Angle,float gyro_Y);
int Velocity(int Target,int Encoder_L,int Encoder_R);
int Turn(float Turn_Err,float gyro_Z);
void Pid_Control(void);

#endif
