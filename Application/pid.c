/*
 * @Author: 星必尘Sguan
 * @Date: 2025-03-27 17:59:29
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-03-27 22:35:38
 * @FilePath: \demo_MyCar\Application\pid.c
 * @Description: PID闭环控制部分
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "pid.h"
#include "Encoder.h"
#include "IIC.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"


//外部变量声明
extern int Encoder_Left;
extern int Encoder_Right;
extern float pitch,roll,yaw;
extern short gyro_x,gyro_y,gyro_z;
extern short accX,accY,accZ;

//全局变量定义
uint8_t Velocity_Stop;
int Vertical_Out,Velocity_Out,Turn_Out,Target_Speed,Target_Turn;
int motor_speed1,motor_speed2;
float Med_Angle=3;//角度值的偏移量(用于平衡小车重心)

/**
 * @description: PID闭环控制器，参数定义列表（待调）
 * @return {*}
 */
float Vertical_Kp=-120,Vertical_Kd=-0.03;
float Velocity_Kp=1.1,Velocity_Ki=0.0055;
float Turn_Kp=10,Turn_Kd=0.1;


/**
 * @description: 直立环PD算法
 * @param {float} Med    期望角度
 * @param {float} Angle  真实角度
 * @param {float} gyro_Y 角速度(角度的微分)
 * @return {*}
 */
int Vertical(float Med,float Angle,float gyro_Y)
{
    int temp;
    temp=Vertical_Kp*(Angle-Med)+Vertical_Kd*gyro_Y;
    return temp;
}


/**
 * @description: 速度环PI算法
 * @param {int} Target_Speed 期望速度
 * @param {int} Encoder_L    左电机真实速度
 * @param {int} Encoder_R    右电机真实速度
 * @return {*}
 */
int Velocity(int Target_Speed,int Encoder_L,int Encoder_R)
{
    static int Speed_Err,Encoder_S;
    static float Err_Wave=0.7;
    int Err0,Err1,temp;
    //1.计算偏差
    Err0=(Encoder_L+Encoder_R) - Target_Speed;

    //2.低通滤波
    Err1=(1-Err_Wave)*Err0+Err_Wave*Speed_Err;
    Speed_Err=Err1;
    //3.积分
    Encoder_S += Speed_Err;

    //4.积分限幅Limit
    Encoder_S = Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
    if(Velocity_Stop==1)Encoder_S=0,Velocity_Stop=0;
    //5.速度环核心
    temp=Velocity_Kp*Speed_Err+Velocity_Ki*Encoder_S;

    return temp;
}


/**
 * @description: 转向环PD算法
 * @param {float} Turn_Err 手动调整的转向角度误差值
 * @param {float} gyro_Z   转向的角加速度
 * @return {*}
 */
int Turn(float Turn_Err,float gyro_Z)
{
    int temp;
    temp=Turn_Kp*Turn_Err+Turn_Kd*gyro_Z;
    return temp;
}


/**
 * @description: 串级PID调控（设定定时器每10ms调控一次）
 * @return {*}
 */
void Pid_Control(void)
{
    int PWM_Out;
    //1.传感器测得真实值(编码器and陀螺仪)
    Read_Speed();
    mpu_dmp_get_data(&pitch,&roll,&yaw);
    MPU_Get_Gyroscope(&gyro_x,&gyro_y,&gyro_z);
    MPU_Get_Accelerometer(&accX,&accY,&accZ);

    //2.数据导入到串级PID控制器中，并导出输出结果(左右俩电机的转速值)
    Velocity_Out = Velocity(Target_Speed,Encoder_Left,Encoder_Right);
    Vertical_Out = Vertical(Velocity_Out + Med_Angle,roll,gyro_x);
    PWM_Out=Vertical_Out;
    Turn_Out = Turn(Target_Turn,gyro_z);

    //3.实际控制
    motor_speed1 = PWM_Out-Turn_Out;
    motor_speed2 = PWM_Out+Turn_Out;
    Limit(&motor_speed1,&motor_speed2);
    Load(motor_speed1,motor_speed2);
}

