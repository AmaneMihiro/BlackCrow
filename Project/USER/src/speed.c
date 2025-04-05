#include "speed.h"
#include "math.h"

int16 aim_speed        = 10;  //Ŀ���ٶ�  
int16 real_speed       = 0;    //������ƽ���ٶ� 
int16 left_speed       = 0;    //�����ٶ�
int16 right_speed      = 0;    //�����ٶ�
int16 last_speed       = 0;    
int16 left_last_speed  = 0;
int16 right_last_speed = 0;
int16 left_real_speed  = 0;
int16 right_real_speed = 0;

int16 All_PWM_left     = 0;     //��������PWM���
int16 All_PWM_right    = 0;     //��������PWM���
int16 Speed_pwm_left   = 0;     //�����ٶȻ�PWM��C���ã�
int16 Speed_pwm_right  = 0;     //�����ٶȻ�PWM��C���ã�
int16 Last_speed_pwm_left = 0;
int16 Last_speed_pwm_right = 0;
int16 Real_Speed_left  = 0;     //����ʵ���ٶ�
int16 Real_Speed_right = 0;     //����ʵ���ٶ�
int16 Speed_pwm_all    = 0;     //����ƽ���ٶȻ�PWM��D���ã�
int16 Steer_pwm        = 0;     //ת���ڻ�PWM
uint16 Open_pack_time  = 0;     //�򿪸ɻɹܶ�ʱ
uint16 Stop_time       = 0;     //ͣ����ʱ
 
/******************************* �����ʼ��***********************************
������  void init_PWM(unsigned char Motor_Set)
������  Motor_Set---Ϊ0ʱ��ʼ��ΪBTN������ʽ��Ϊ1ʱ��ʼ��DRV������ʽ
˵����  ��ĸ10000
		    pwm_init(PWMA_CH1P_P60, 10000, 0); 
        ��ʼ��PWM  ʹ������P6.0  ���PWMƵ��10000HZ  ռ�ձ�Ϊ�ٷ�֮ pwm_duty / PWM_DUTY_MAX * 100				
����ֵ����  
*****************************************************************************/
unsigned char MOTOR_MODE=0;//�м�����������޸�ɾ��������
void init_PWM(unsigned char Motor_Set)
{
	MOTOR_MODE = Motor_Set;
	if (MOTOR_MODE==0) 
	{
//-----MOS����-----------
    pwm_init(Left_Z_Pin,  20000,0);//���ֳ�ʼ��
	  pwm_init(Left_F_Pin,  20000,0);
	  pwm_init(Right_Z_Pin, 20000,0);//���ֳ�ʼ��
	  pwm_init(Right_F_Pin, 20000,0);
	}
		else
	{
//------DRV����-------------
	  pwm_init(Left_PWM_Pin, 20000,0);//���ֳ�ʼ��
  	pwm_init(Right_PWM_Pin,20000,0);//���ֳ�ʼ��
		gpio_mode(P6_4,GPO_PP);       // ����DRV��������ΪΪ�������
	  gpio_mode(P6_0,GPO_PP);       // ����DRV��������ΪΪ�������
 } 
}
/****************************��������ʼ��****************************
������  void encoder_init(void)
���ܣ�  ��������ʼ��
������  ��
˵����  ctimer_count_init(CTIM0_P34);
        ������ʹ��TIM3��TIM4�����������ֻ���޸ĺ궨�弴�� 
        ������ʹ�ô�����ı�������STC��֧���������룩
����ֵ����
********************************************************************/
void encoder_init()
{
    //���������ʼ��
		ctimer_count_init(Left_Ecoder_Pin1);
		//�ұ�������ʼ��
		ctimer_count_init(Right_Ecoder_Pin1);
}  
/***************************�ٶȲ���********************************
��������speed_measure()
��  �ܣ��ٶȲ�������ȡ��������ֵ����ͬ��������װ�ͳ���ǰ�����򲻶Ի�
        ���²ɼ���ֵ�����Ƿ������ģ�ֻ���޸�* (-1)���У��ĵ�����Ϳ�
        ����
��  ����void
����ֵ��void
******************************************************************/
void speed_measure()
{ 
	  int16 temp_L, temp_R;
    temp_L = ctimer_count_read(Left_Ecoder_Pin1); // �����ֵ�ǰ�ٶ�
    temp_R = ctimer_count_read(Right_Ecoder_Pin1);

    ctimer_count_clean(Left_Ecoder_Pin1); // ����������
    ctimer_count_clean(Right_Ecoder_Pin1);
	  
	  if(Left_Ecoder_Pin2 == 1)
        left_speed = temp_L;
    else
        left_speed = (1) * temp_L;
    if(Right_Ecoder_Pin2 == 0)
        right_speed = temp_R;
    else
        right_speed = (1) * temp_R;
		
		last_speed = (right_speed+left_speed)/2;// �ٶ�ƽ��ֵ
	  real_speed*=0.8;            //һ�׵�ͨ�˲���
	  real_speed+=last_speed*0.2; //һ�׵�ͨ�˲���  left_last_speed
		
		left_last_speed =left_speed;
	  left_real_speed*=0.8;       //һ�׵�ͨ�˲���
	  left_real_speed+=left_last_speed*0.2; //һ�׵�ͨ�˲���
		
		right_last_speed =right_speed;
	  right_real_speed*=0.8;        //һ�׵�ͨ�˲���
	  right_real_speed+=right_last_speed*0.2; //һ�׵�ͨ�˲���
}
/*******************************���ⶨʱ�򿪸ɻɹܵ�***********************************
������  void timed_task(void)
������  ��
˵����  ������ɶ�ʱ�򿪸ɻɹܵ���Ϊ��־λ��������ֹ�ճ���ͼ�⵽ͣ��
����ֵ���� 
*************************************************************************************/
void timed_task(void)
{
	if(flag_start)
	{
		Open_pack_time=Open_pack_time+10;
	}
	if(flag_end)
	{
		T_inku_wait=T_inku_wait+5;
	}
	if(T_inku_J)
	{
		T_inku_S=T_inku_S+5;
	}
}

/*****************************������*******************************************
������void go_motor (int16 left_PWM,int16 right_PWM)
������  int16 left_PWM,int16 right_PWM
˵����pwm_duty(PWMA_CH1P_P60, duty);
      ��ؽ����ӵĵ���߼��������һ������һ�����������������ڶ������������ҵ��
      ����Ĳ������Ϊ�����������ת����ֵ��ת����������
����ֵ���� 
********************************************************************************/
#define Duty_Max  6000   //�޷����ֵ

void go_motor (int16 left_PWM,int16 right_PWM)
{
  if  (MOTOR_MODE==0)
  {	
//---------------------------------MOS����-----------------------------------------	
    if (left_PWM>0)                   //����
    {
		 left_PWM = left_PWM<=Duty_Max ? left_PWM : Duty_Max;
     pwm_duty(Left_Z_Pin,left_PWM);
     pwm_duty(Left_F_Pin,0);			    //��ת
    } 	
    else 
    {
     left_PWM = left_PWM>=-Duty_Max ? (-left_PWM) : Duty_Max;  
     pwm_duty(Left_Z_Pin,1);	
     pwm_duty(Left_F_Pin,left_PWM);	  //��ת
    } 
    if (right_PWM>0)                  //����
    { 
     right_PWM = right_PWM<=Duty_Max ? right_PWM : Duty_Max; 
	   pwm_duty(Right_Z_Pin,right_PWM);	
	   pwm_duty(Right_F_Pin,0);			    //��ת
    } 
    else 
    {
     right_PWM = right_PWM>=-Duty_Max ? (-right_PWM) : Duty_Max;  
	   pwm_duty(Right_Z_Pin,1);	
	   pwm_duty(Right_F_Pin,right_PWM); //��ת
    }
  }
  else
  {
//-------------------------------------------DRV����-------------------------------------
   if (left_PWM>0)                     //����
   {
		 left_PWM = left_PWM<=Duty_Max ? left_PWM : Duty_Max;
		 Left_DIR_Pin=0;			 
     pwm_duty(Left_PWM_Pin,left_PWM);  //��ת
   } 	
   else 
   {
     left_PWM = left_PWM>=-Duty_Max ? (-left_PWM) : Duty_Max;  
     Left_DIR_Pin=1;	
     pwm_duty(Left_PWM_Pin,left_PWM);  //��ת
   }
   if (right_PWM>0)                    //����
   {
     right_PWM = right_PWM<=Duty_Max ? right_PWM : Duty_Max;
     Right_DIR_Pin=0;			 
	   pwm_duty(Right_PWM_Pin,right_PWM);//��ת		
	 } 
   else 
   {
     right_PWM = right_PWM>=-Duty_Max ? (-right_PWM) : Duty_Max;  
	   Right_DIR_Pin=1;
	   pwm_duty(Right_PWM_Pin,right_PWM); //��ת
   }
  }
}