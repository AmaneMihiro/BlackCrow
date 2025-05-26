#ifndef __speed_H_
#define __speed_H_

#include "headfile.h"

#define Limit_Min_Max(data,min,max) (((data)>(max)) ? (max) : (((data) < (min)) ? (min) : (data)))

//��������

extern int16 aim_speeda ;
extern int16 aim_speedb ; 
extern int16 aim_speedc ;

extern float aim_speed;  
extern float real_speed;        
extern float left_speed;  
extern float last_speed;
extern float left_last_speed;
extern float right_last_speed;
extern float left_real_speed;
extern float right_real_speed;
extern float right_speed;       
extern int16 All_PWM_left;     
extern int16 All_PWM_right;    
extern int32 Speed_pwm_left;      
extern int32 Speed_pwm_right;
extern int16 Real_Speed_left;    //����ʵ���ٶ�
extern int16 Real_Speed_right;   //����ʵ���ٶ�
extern int16 Speed_pwm_all;      
extern int16 Steer_pwm;
extern uint16 Open_pack_time;

//��������
void init_PWM(unsigned char Motor_Set);
void encoder_init(void);
void speed_measure(void);
void go_motor (int32 left_PWM,int32 right_PWM);
void timed_task(void);

#endif
