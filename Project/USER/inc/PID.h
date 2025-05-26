#ifndef __PID_H__
#define __PID_H__

#include "headfile.h"

typedef struct
{
    float SumError; // 误差累积

    float Kp; // 比例系数
    float Ki; // 积分系数
    float Kd; // 微分系数
    float K_gory;

    float LastError;  // 上一次误差值
    float LLastError; // 上上次误差
} PID;

// 结构体变量声明
extern PID TurnPID;
extern PID SpeedPID;
extern PID L_SpeedPID;
extern PID R_SpeedPID;

// 函数声明
void IncPIDInit(PID *sptr);
int16 LocP_DCalc(PID *sptr, int16 Setpoint, int16 Turepoint);
int32 IncPIDCalc(PID *sptr, float Setpoint, float Turepoint, float Kf); // 增量式PID控制
int16 PlacePID_Control(PID *sptr, int16 Setpoint, int16 Turepiont);     // 动态位置式PID控制
int16 PID_Turn(PID *sptr, int16 Error, int16 Gory_z);
float PID_Turn_DT(PID *sptr, float Error, int16 Gory_z);
#endif
