#include "speed.h"
#include "math.h"

float aim_speed = 50;  // 目标速度
float real_speed = 0;  // 车身中心平均速度
float left_speed = 0;  // 左轮速度
float right_speed = 0; // 右轮速度
float last_speed = 0;
float left_last_speed = 0;
float right_last_speed = 0;
float left_real_speed = 0;
float right_real_speed = 0;

int16 All_PWM_left = 0;     // 左轮输出PWM数值
int16 All_PWM_right = 0;    // 右轮输出PWM数值
int32 Speed_pwm_left = 0;   // 左轮速度环PWM（C车用）
int32 Speed_pwm_right = 0;  // 右轮速度环PWM（C车用）
int16 Real_Speed_left = 0;  // 左轮实际速度
int16 Real_Speed_right = 0; // 右轮实际速度
int16 Speed_pwm_all = 0;    // 车身平均速度环PWM（D车用）
int16 Steer_pwm = 0;        // 转向舵机PWM
uint16 Open_pack_time = 0;  // 打开干货罐定时
uint16 Stop_time = 0;       // 停车定时

/******************************* 电机初始化***********************************
函数名  void init_PWM(unsigned char Motor_Set)
功能    Motor_Set---为0时初始化为BTN驱动方式，为1时初始化DRV驱动方式
说明    参数10000
            pwm_init(PWMA_CH1P_P60, 10000, 0);
        初始化PWM  使用引脚P6.0  输出PWM频率10000HZ  占空比为百分之 pwm_duty / PWM_DUTY_MAX * 100
返回值：无
*****************************************************************************/
unsigned char MOTOR_MODE = 0; // 中间变量，请不要修改删除这个变量
void init_PWM(unsigned char Motor_Set)
{
    MOTOR_MODE = Motor_Set;
    if (MOTOR_MODE == 0)
    {
        //-----MOS驱动-----------
        pwm_init(Left_Z_Pin, 20000, 0); // 左轮初始化
        pwm_init(Left_F_Pin, 20000, 0);
        pwm_init(Right_Z_Pin, 20000, 0); // 右轮初始化
        pwm_init(Right_F_Pin, 20000, 0);
    }
    else
    {
        //------DRV驱动-------------
        pwm_init(Left_PWM_Pin, 20000, 0);  // 左轮驱动
        pwm_init(Right_PWM_Pin, 20000, 0); // 右轮驱动
        gpio_mode(P6_4, GPO_PP);           // 功能：  编码器初始化为输出模式
        gpio_mode(P6_0, GPO_PP);           // 功能：  编码器初始化为输出模式
    }
}
/****************************编码器初始化****************************
函数名  void encoder_init(void)
功能    编码器初始化
参数    无
说明    ctimer_count_init(CTIM0_P34);
        如果想使用TIM3或TIM4，请在工程添加相应的头文件即可
        如果想使用串口，必须注意是STC内核（外部晶振不可修改）
返回值：无
********************************************************************/
void encoder_init()
{
    // 参数：  无
    ctimer_count_init(Left_Ecoder_Pin1);
    // 右轮编码器初始化
    ctimer_count_init(Right_Ecoder_Pin1);
}
/***************************速度测量********************************
函数名：speed_measure()
功能    速度测量，采集同步，由于编码器安装和车辆前进方向不对称
        后采集的值符号是否正确，只需修改* (-1)符号，改到左右相同
参数    void
返回值：void
******************************************************************/
void speed_measure()
{
    int16 temp_L, temp_R;
    // float max_change = 20.0f; // 最大允许变化量
    // float left_change = left_speed - left_last_speed;
    // float right_change = right_speed - right_last_speed;

    temp_L = ctimer_count_read(Left_Ecoder_Pin1); // 获取左轮当前速度
    temp_R = ctimer_count_read(Right_Ecoder_Pin1);

    ctimer_count_clean(Left_Ecoder_Pin1); // 清空计数器
    ctimer_count_clean(Right_Ecoder_Pin1);

    if (Left_Ecoder_Pin2 == 0)
        left_speed = (-1) * temp_L;
    else
        left_speed = temp_L;
    if (Right_Ecoder_Pin2 == 1)
        right_speed = (-1) * temp_R;
    else
        right_speed = temp_R;

    // // 组合滤波算法（限幅+低通）

    // // 1. 限幅部分 - 防止突变噪声

    // if (fabs(left_change) > max_change)
    // {
    //     // 限制变化幅度
    //     left_speed = left_last_speed + (left_change > 0 ? max_change : -max_change);
    // }

    // if (fabs(right_change) > max_change)
    // {
    //     // 限制变化幅度
    //     right_speed = right_last_speed + (right_change > 0 ? max_change : -max_change);
    // }

    // // 2. 低通滤波部分 - 平滑速度变化
    // left_real_speed = left_speed * 0.9 + left_last_speed * 0.1;
    // right_real_speed = right_speed * 0.9 + right_last_speed * 0.1;

    // // 保存当前值用于下次计算
    // left_last_speed = left_real_speed;
    // right_last_speed = right_real_speed;

    left_real_speed = left_speed;
    right_real_speed = right_speed;

    // 计算车身中心平均速度
    real_speed = (right_real_speed + left_real_speed) / 2;
}
/*******************************电机定时打开干货罐等***********************************
函数名：timed_task(void)
功能    定时操作
参数    void
返回值：void
*************************************************************************************/
void timed_task(void)
{
    if (flag_start)
    {
        Open_pack_time = Open_pack_time + 10;
    }
    if (flag_end)
    {
        T_inku_wait = T_inku_wait + 5;
    }
    if (T_inku_J)
    {
        T_inku_S = T_inku_S + 5;
    }
}

/*****************************电机驱动*******************************************
函数名：go_motor (int16 left_PWM,int16 right_PWM)
参数：int16 left_PWM,int16 right_PWM
说明：pwm_duty(PWMA_CH1P_P60, duty);
      该函数输入的电机逻辑是左一正右一正，左二输入零右二输入零
      传入的参数为左右轮输出转动的大小转动方向由符号决定
返回值：void
********************************************************************************/
#define Duty_Max 20000 // 限幅最大值

void go_motor(int32 left_PWM, int32 right_PWM)
{
    if (MOTOR_MODE == 0)
    {
        //---------------------------------MOS驱动-----------------------------------------
        if (left_PWM > 0) // 正转
        {
            left_PWM = left_PWM <= Duty_Max ? left_PWM : Duty_Max;
            pwm_duty(Left_Z_Pin, left_PWM);
            pwm_duty(Left_F_Pin, 0); // 正转
        }
        else
        {
            left_PWM = left_PWM >= -Duty_Max ? (-left_PWM) : Duty_Max;
            pwm_duty(Left_Z_Pin, 1);
            pwm_duty(Left_F_Pin, left_PWM); // 反转
        }
        if (right_PWM > 0) // 正转
        {
            right_PWM = right_PWM <= Duty_Max ? right_PWM : Duty_Max;
            pwm_duty(Right_Z_Pin, right_PWM);
            pwm_duty(Right_F_Pin, 0); // 正转
        }
        else
        {
            right_PWM = right_PWM >= -Duty_Max ? (-right_PWM) : Duty_Max;
            pwm_duty(Right_Z_Pin, 1);
            pwm_duty(Right_F_Pin, right_PWM); // 反转
        }
    }
    else
    {
        //-------------------------------------------DRV驱动-------------------------------------
        if (left_PWM > 0) // 正转
        {
            left_PWM = left_PWM <= Duty_Max ? left_PWM : Duty_Max;
            Left_DIR_Pin = 0;
            pwm_duty(Left_PWM_Pin, left_PWM); // 正转
        }
        else
        {
            left_PWM = left_PWM >= -Duty_Max ? (-left_PWM) : Duty_Max;
            Left_DIR_Pin = 1;
            pwm_duty(Left_PWM_Pin, left_PWM); // 正转
        }
        if (right_PWM > 0) // 正转
        {
            right_PWM = right_PWM <= Duty_Max ? right_PWM : Duty_Max;
            Right_DIR_Pin = 0;
            pwm_duty(Right_PWM_Pin, right_PWM); // 正转
        }
        else
        {
            right_PWM = right_PWM >= -Duty_Max ? (-right_PWM) : Duty_Max;
            Right_DIR_Pin = 1;
            pwm_duty(Right_PWM_Pin, right_PWM); // 正转
        }
    }
}