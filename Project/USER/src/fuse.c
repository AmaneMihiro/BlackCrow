#include "fuse.h"
#include "math.h"

PID SpeedPID = {0};
PID L_SpeedPID = {0};
PID R_SpeedPID = {0};
PID TurnPID = {0};

int16 GORY_Z = 0;
/****************************PID������ʼ��**************************************
������  void PID_int(void)
������  void
˵����  PIDÿ����������ʼ��
����ֵ��void
********************************************************************************/
void PID_int(void)
{
  //	SpeedPID.Kp=50;     //0.6//�ٶȻ�PID������D���ã��ٶȻ�2ms��18
  //	SpeedPID.Ki=10 ;     //0.5                                  2.5
  //	SpeedPID.Kd=0;

  L_SpeedPID.Kp = 360; // 4  //�����ٶȻ�PID�������ٶȻ�20ms����6��4.5
  L_SpeedPID.Ki = 0.1; // 1
  L_SpeedPID.Kd = 0;

  R_SpeedPID.Kp = 360; // ���ٶȻ�PID�������ٶȻ�20ms����6��5.5
  R_SpeedPID.Ki = 0.1;
  R_SpeedPID.Kd = 0;

  TurnPID.Kp = 1.5; // ת��PID����
  TurnPID.Ki = 0;   // 0.08
  TurnPID.Kd = 2.5; // ��ϵ�ѹ7.3v

  //	TurnPID.Kp=0.75 ;       //ת��PID����
  //	TurnPID.Ki=0; //0.08
  //	TurnPID.Kd=0.55;//��ϵ�ѹ7.5v

  TurnPID.K_gory = 0;
}
// void PID_int(void)
//{
//	SpeedPID.Kp=18;     //0.6//�ٶȻ�PID������D���ã��ٶȻ�2ms��18
//	SpeedPID.Ki=3 ;     //0.5                                  2.5
//	SpeedPID.Kd=0;
//
//	L_SpeedPID.Kp=4; //4  //�����ٶȻ�PID�������ٶȻ�20ms����6��4.5
//	L_SpeedPID.Ki=0.15;//1
//	L_SpeedPID.Kd=0;
//
//	R_SpeedPID.Kp=4;   //���ٶȻ�PID�������ٶȻ�20ms����6��5.5
//	R_SpeedPID.Ki=0.15;
//	R_SpeedPID.Kd=0;
//
//	TurnPID.Kp=87;       //ת��PID����
//	TurnPID.Ki=0; //0.08
//	TurnPID.Kd=0.4;
//	TurnPID.K_gory=3.5;
// }
//  ����
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
/****************************���ٶ��ڻ���D���ٶȻ�**************************************
������  void Motor_output_control()
������  void
˵����  ���ٶ��ڻ���D���ٶȻ�(D��/���ֳ��Ż���)
����ֵ��void
***************************************************************************************/
void Motor_output_control()
{
  //	P52=0;//���ж�Ƶ��

  // imu660ra_get_gyro();   //��ȡ660�����ǽ��ٶ�ֵ
  icm20602_get_gyro();
  GORY_Z = icm20602_gyro_transition(icm20602_gyro_z);
  // GORY_Z= imu660ra_gyro_transition(imu660ra_gyro_z);         // ��λΪ��/s
  speed_measure(); // ����������
  Get_deviation(); // ��Ųɼ�����ȡ����ƫ��

  // timed_task(); // ��ʱ����

  ADC_PWM = PID_Turn_DT(&TurnPID, Current_Dir, 0); // ��̬λ��ʽPID  left_real_speed
  //	  ADC_PWM=range_protect(ADC_PWM, -500, 500);
  //	  Speed_pwm_all  += IncPIDCalc(&SpeedPID,aim_speed,real_speed);
  //    Real_Speed_left+=fabs(real_speed)*0.1;
  //	  if(Real_Speed_left>800)
  //		{
  //			aim_speed=0;
  //			Out_protect();         //���籣��
  //		}

  // if (aim_speedb + ADC_PWM < 0)
  // {
  //   aim_speedb = 0;
  // }
  // if (aim_speedb - ADC_PWM < 0)
  // {
  //   aim_speedb = 0;
  // }

  Speed_pwm_left += IncPIDCalc(&L_SpeedPID, aim_speedb + ADC_PWM, left_real_speed);
  Speed_pwm_right += IncPIDCalc(&R_SpeedPID, aim_speedb - ADC_PWM, right_real_speed);

  //	  Speed_pwm_left=range_protect(Speed_pwm_left, -aim_speed, 2*aim_speed);//�����޷�����ֹ���ӷ�ת̫���¼������ٶȣ�
  //	  Speed_pwm_right=range_protect(Speed_pwm_right, -aim_speed,2*aim_speed);//�����޷�����ֹ���ӷ�ת̫���¼������ٶȣ�
  //	  if(Speed_pwm_left<-aim_speed)
  //	  {
  //	  Speed_pwm_left=-aim_speed;
  //	  }
  //	  if(Speed_pwm_right<-aim_speed)
  //	  {
  //	  Speed_pwm_right=-aim_speed;
  //	  }
  //	  All_PWM_left  = Speed_pwm_all+ADC_PWM;
  //	  All_PWM_right = Speed_pwm_all-ADC_PWM;

  go_motor(Speed_pwm_left, Speed_pwm_right); // �������

  //	  pwm_duty(PWMB_CH4_P77, 800);
  //		pwm_duty(PWMB_CH3_P33, 800);
  // go_motor(2000,2000);
  Out_protect(); // ���籣��
  // P52=1;
}
/****************************ת�򻷣�D��ת���⻷��**************************************
������  void Trailing_control()
������  void
˵����  ת�򻷣�D��ת���⻷����C��ת�򻷣�
����ֵ��void
***************************************************************************************/
void Trailing_control()
{
  //	Get_deviation();  //��Ųɼ�����ȡ����ƫ��
  //	speed_measure();       //����������
  //
  //	ADC_PWM = LocP_DCalc(&TurnPID,Current_Dir,0);//λ��ʽPD����ת��
  ////	Out_protect();         //���籣��
  //	ADC_PWM = PlacePID_Control(&TurnPID,Current_Dir,0);//��̬λ��ʽPID����
  //	ADC_PWM = PlacePID_Control(&TurnPID,Current_Dir,0);//��̬λ��ʽPID���� (����ת�����)
  // Steering_Control_Out(ADC_PWM);//(C���ÿ��ƶ��ת��)
}
/****************************�ٶȻ���C���ã�**************************************
������  void Speed_control()
������  void
˵����  �ٶȻ���C���ã�
***************************************************************************************/
void Speed_control()
{
  // timed_task();           //���ⶨʱ�򿪸ɻɹܵ�
  // speed_measure();      //����������
  // aim_speed = 450;      //Ŀ���ٶ�

  // Speed_pwm_all = LocP_DCalc(&SpeedPID,aim_speed ,real_speed); //D���ٶȻ���λ��ʽ��
  // Speed_pwm_all += IncPIDCalc(&SpeedPID,aim_speed,real_speed);//D���ٶȻ�������ʽ��

  // Speed_pwm_left += IncPIDCalc(&L_SpeedPID,aim_speed , left_speed); //C�������ٶȻ���λ��ʽ��
  // Speed_pwm_right += IncPIDCalc(&R_SpeedPID, aim_speed, right_speed); //C�������ٶȻ���λ��ʽ��
  // go_motor(Speed_pwm_left,Speed_pwm_right);                         //�������
}
/***************************************************************************************
��������int16 range_protect(int16 duty, int16 min, int16 max)
��  �ܣ��޷�����
��  ����
����ֵ��duty
**************************************************************************************/
int16 range_protect(int16 duty, int16 min, int16 max) // �޷�����
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
