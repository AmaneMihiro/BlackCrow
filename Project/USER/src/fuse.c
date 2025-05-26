#include "fuse.h"
#include "math.h"
#include "vofa.h"

PID SpeedPID = {0};
PID L_SpeedPID = {0};
PID R_SpeedPID = {0};
PID TurnPID = {0};

int16 GORY_Z = 0;
uint8 outline_stop = 0;
float temp_left = 0;
float temp_right = 0;
/****************************PID参数初始化**************************************
函数：  void PID_int(void)
参数：  void
说明：  PID每个环参数初始化
返回值：void
********************************************************************************/
void PID_int(void)
{
    L_SpeedPID.Kp = 1000; // 左轮速度环PID参数
    L_SpeedPID.Ki = 4;
    L_SpeedPID.Kd = 0;

    R_SpeedPID.Kp = 1000; // 右速度环PID参数
    R_SpeedPID.Ki = 4;
    R_SpeedPID.Kd = 0;

    TurnPID.Kp = 10;
    TurnPID.Ki = 0;
    TurnPID.Kd = 0;

    TurnPID.K_gory = 0;
}

static TASK_COMPONENTS TaskComps[] =
    {
        {0, 1, 1, Motor_output_control}, // 角速度内环和D车速度环2ms
                                         //    {0, 2, 2, Trailing_control},           //转向外环10ms
                                         //    {0, 4, 4, Speed_control},              //C车速度环20ms
};
/**************************************************************************************
 * FunctionName   : TaskRemarks()
 * Description    : 任务标志处理
 * EntryParameter : None
 * ReturnValue    : None
 * attention      : ***在定时器中断中调用此函数即可***
 **************************************************************************************/
void TaskRemarks(void)
{
    uint8 i;
    for (i = 0; i < TASKS_MAX; i++) // 逐个任务时间处理
    {
        if (TaskComps[i].Timer) // 时间不为0
        {
            TaskComps[i].Timer--;        // 减去一个节拍
            if (TaskComps[i].Timer == 0) // 时间减完了
            {
                TaskComps[i].Timer = TaskComps[i].ItvTime; // 恢复计时器值，从新下一次
                TaskComps[i].Run = 1;                      // 任务可以运行
            }
        }
    }
}

/**************************************************************************************
 * FunctionName   : TaskProcess()
 * Description    : 任务处理|判断什么时候该执行那一个任务
 * EntryParameter : None
 * ReturnValue    : None
 * * attention      : ***放在mian的while(1)即可***
 **************************************************************************************/
void TaskProcess(void)
{
    uint8 i;
    for (i = 0; i < TASKS_MAX; i++) // 逐个任务时间处理
    {
        if (TaskComps[i].Run) // 时间不为0
        {
            TaskComps[i].TaskHook(); // 运行任务
            TaskComps[i].Run = 0;    // 标志清0
        }
    }
}

/***************************************************************************************
函数名：int16 range_protect(int16 duty, int16 min, int16 max)
功  能：限幅保护
参  数：
返回值：duty
**************************************************************************************/
float range_protect(float duty, float min, float max) // 限幅保护
{
    if (duty >= max)
    {
        return max;
    }
    if (duty <= min)
    {
        return min;
    }
    else
    {
        return duty;
    }
}

/****************************角速度内环和D车速度环**************************************
函数：  void Motor_output_control()
参数：  void
说明：  角速度内环和D车速度环(D车/三轮车才会用)
返回值：void
***************************************************************************************/
void Motor_output_control()
{

    icm20602_get_gyro();
    GORY_Z = icm20602_gyro_transition(icm20602_gyro_z); // 单位为°/s
    speed_measure();                                    // 编码器测量
    Get_deviation();                                    // 电磁采集并获取赛道偏差

    ADC_PWM = PID_Turn_DT(&TurnPID, Current_Dir, 0);



    if ((Left_Adc < 3 && Right_Adc < 3 && Left_Shu_Adc < 3 && Right_Shu_Adc < 3) || outline_stop == 1)
    {
        outline_stop = 1;
        go_motor(0, 0); // 动力输出
    }
    else
    {
        // Speed_pwm_left += IncPIDCalc(&L_SpeedPID, aim_speed + ADC_PWM, left_real_speed, 0);
        // Speed_pwm_right += IncPIDCalc(&R_SpeedPID, aim_speed - ADC_PWM, right_real_speed, 0);
        temp_left = range_protect(aim_speed + ADC_PWM, 1.0f, 2 * aim_speed);
        temp_right = range_protect(aim_speed - ADC_PWM, 1.0f, 2 * aim_speed);
        Speed_pwm_left += IncPIDCalc(&L_SpeedPID, temp_left, left_real_speed, 0);
        Speed_pwm_right += IncPIDCalc(&R_SpeedPID, temp_right, right_real_speed, 0);
        go_motor(Speed_pwm_left, Speed_pwm_right); // 动力输出
    }
}


