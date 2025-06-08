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
/****************************PID������ʼ��**************************************
������  void PID_int(void)
������  void
˵����  PIDÿ����������ʼ��
����ֵ��void
********************************************************************************/
void PID_int(void)
{
    L_SpeedPID.Kp = 800; // �����ٶȻ�PID����
    L_SpeedPID.Ki = 0;
    L_SpeedPID.Kd = 0;

    R_SpeedPID.Kp = 800; // ���ٶȻ�PID����
    R_SpeedPID.Ki = 0;
    R_SpeedPID.Kd = 0;

    TurnPID.Kp = 15;
    TurnPID.Ki = 0;
    TurnPID.Kd = 15;

    TurnPID.K_gory = 0;
}

static TASK_COMPONENTS TaskComps[] =
    {
        {0, 1, 1, Motor_output_control}, // ���ٶ��ڻ���D���ٶȻ�2ms
                                         //    {0, 2, 2, Trailing_control},           //ת���⻷10ms
                                         //    {0, 4, 4, Speed_control},              //C���ٶȻ�20ms
};
/**************************************************************************************
 * FunctionName   : TaskRemarks()
 * Description    : �����־����
 * EntryParameter : None
 * ReturnValue    : None
 * attention      : ***�ڶ�ʱ���ж��е��ô˺�������***
 **************************************************************************************/
void TaskRemarks(void)
{
    uint8 i;
    for (i = 0; i < TASKS_MAX; i++) // �������ʱ�䴦��
    {
        if (TaskComps[i].Timer) // ʱ�䲻Ϊ0
        {
            TaskComps[i].Timer--;        // ��ȥһ������
            if (TaskComps[i].Timer == 0) // ʱ�������
            {
                TaskComps[i].Timer = TaskComps[i].ItvTime; // �ָ���ʱ��ֵ��������һ��
                TaskComps[i].Run = 1;                      // �����������
            }
        }
    }
}

/**************************************************************************************
 * FunctionName   : TaskProcess()
 * Description    : ������|�ж�ʲôʱ���ִ����һ������
 * EntryParameter : None
 * ReturnValue    : None
 * * attention      : ***����mian��while(1)����***
 **************************************************************************************/
void TaskProcess(void)
{
    uint8 i;
    for (i = 0; i < TASKS_MAX; i++) // �������ʱ�䴦��
    {
        if (TaskComps[i].Run) // ʱ�䲻Ϊ0
        {
            TaskComps[i].TaskHook(); // ��������
            TaskComps[i].Run = 0;    // ��־��0
        }
    }
}

/***************************************************************************************
��������int16 range_protect(int16 duty, int16 min, int16 max)
��  �ܣ��޷�����
��  ����
����ֵ��duty
**************************************************************************************/
float range_protect(float duty, float min, float max) // �޷�����
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

/****************************���ٶ��ڻ���D���ٶȻ�**************************************
������  void Motor_output_control()
������  void
˵����  ���ٶ��ڻ���D���ٶȻ�(D��/���ֳ��Ż���)
����ֵ��void
***************************************************************************************/
void Motor_output_control()
{

    icm20602_get_gyro();
    GORY_Z = icm20602_gyro_transition(icm20602_gyro_z); // ��λΪ��/s
    speed_measure();                                    // ����������
    Get_deviation();                                    // ��Ųɼ�����ȡ����ƫ��

    ADC_PWM = PID_Turn_DT(&TurnPID, Current_Dir, 0);

    if ((Left_Adc < 3 && Right_Adc < 3 && Left_Shu_Adc < 3 && Right_Shu_Adc < 3) || outline_stop == 1)
    {
        outline_stop = 1;
        Speed_pwm_left += IncPIDCalc(&L_SpeedPID, 0, left_real_speed, 0);
        Speed_pwm_right += IncPIDCalc(&R_SpeedPID, 0, right_real_speed, 0);
        go_motor(Speed_pwm_left, Speed_pwm_right); // �������
    }
    else
    {
        if (ADC_PWM >= 0)
        {
            temp_left = aim_speed + 0.6f * ADC_PWM;
            temp_right = aim_speed - 1.2f * ADC_PWM;
        }
        else
        {
            temp_left = aim_speed + 1.2f * ADC_PWM;
            temp_right = aim_speed - 0.6f * ADC_PWM;
        }
        Speed_pwm_left += IncPIDCalc(&L_SpeedPID, temp_left, left_real_speed, 0);
        Speed_pwm_right += IncPIDCalc(&R_SpeedPID, temp_right, right_real_speed, 0);
        go_motor(Speed_pwm_left, Speed_pwm_right); // �������
    }
}


