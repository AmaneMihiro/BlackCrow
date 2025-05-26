#include "PID.h"
#include "math.h"
/************************************************
函数名：IncPIDInit(PID *sptr)
功  能：PID参数初始化
参  数：
返回值：void
************************************************/
void IncPIDInit(PID *sptr)
{
    sptr->SumError = 0;
    sptr->LastError = 0;
    sptr->LLastError = 0;

    sptr->Kp = 0;
    sptr->Ki = 0;
    sptr->Kd = 0;
}

/************************************************
函数名：LocP_DCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：位置式PID控制
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：float
************************************************/
int16 LocP_DCalc(PID *sptr, int16 Setpoint, int16 Turepoint)
{
    int16 iError, dError;
    int16 output;

    iError = Setpoint - Turepoint;                // 偏差
    sptr->SumError += iError;                     // 积分(由于时间较短时，可能一次并不执行完一次微分，积累大的误差)
    dError = (int16)(iError - (sptr->LastError)); // 微分
    sptr->LastError = iError;
    if (sptr->SumError > 2000)
        sptr->SumError = 2000; // 积分限幅
    if (sptr->SumError < -2000)
        sptr->SumError = -2000;
    output = (int16)(sptr->Kp * iError             // 比例项
                     + (sptr->Ki * sptr->SumError) // 积分项
                     + sptr->Kd * dError);         // 微分项
    return (output);
}
/************************************************
函数名：IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：增量式PID控制
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：int32 iIncpid
************************************************/
int32 IncPIDCalc(PID *sptr, float Setpoint, float Turepoint, float Kf)
{
    uint8 enable_Ki;
    float Error, output;
    static float LastSetpoint;
    // 当前误差
    Error = Setpoint - Turepoint; // 偏差

    if (fabs(Error) < 6)
    {
        enable_Ki = 1;
    }
    else
    {
        enable_Ki = 0;
    }

    output = sptr->Kp * (Error - sptr->LastError) + enable_Ki * sptr->Ki * Error + Kf * (Setpoint - LastSetpoint);

    sptr->LastError = Error;
    LastSetpoint = Setpoint;
    return (output);
}
/************************************************
函数名：PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont)
功  能：动态位置式PID控制 (一般用于转向控制)
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：int32 Actual
************************************************/
int16 PlacePID_Control(PID *sptr, int16 Setpoint, int16 Turepiont)
{
    int16 iError, Actual;
    float KP; // 动态P，注意与Kp区分

    iError = Setpoint - Turepiont;
    KP = (iError)*sptr->Ki + sptr->Kp; // 动态P的计算
    // sptr->SumError+=iError;

    Actual = KP * iError + sptr->Kd * (iError - sptr->LastError);

    sptr->LastError = iError;
    return Actual;
}
/************************************************
函数名：LocP_DCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：D型位置式PID控制，带角速度项
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：float
************************************************/
float PID_Turn_DT(PID *sptr, float Error, int16 Gory_z)
{
    uint8 enable_Kd;
    float output;

    if (fabs(Error) < 2)
    {
        enable_Kd = 1;
    }
    else
    {
        enable_Kd = 0;
    }

    output = sptr->Kp * Error                                   // 比例项(动态p)
             + enable_Kd * sptr->Kd * (Error - sptr->LastError) // 微分项
             + sptr->K_gory * Gory_z;                           // 角速度项

    sptr->LastError = Error;

    return (output);
}