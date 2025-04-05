#include "park.h"
#include "math.h"

int8   flag_start           = 0;    //出库完成标志位
int8   flag_open_reed       = 0;    //打开干簧管
int8   reed_state           = 0;    //干簧管状态
int8   flag_end             = 0;    //开始停车标志位
uint16 T_outku              = 0;    //出库直走和打角定时
uint16 J_outku              = 0;
uint16 T_inku_wait=0;
uint16 T_inku_J=0;
uint16 T_inku_S=0;
uint16 T_inku=0;

uint16 S_daoku              = 0;    //
uint16 S_daoku_2              = 0;    //
uint16 S_daoku_3              = 0;    //

uint8 Library_selection = 1;

/*****************************************出库函数***************************************
函数：  void Handle_Barn_Out(uint8 type) 
参数：  type-----1为左出库，2为右出库
说明：  出库函数

*注意：调用此函数后执行出库操作，直走的时间和打角时间及占空比按需自己修改调试
返回值：无  
******************************************************************************************/
void Handle_Barn_Out(uint8 type)
{
	//1为左出库，2为右出库
    if(type ==1)
    {
		 if(!flag_start)
		 {
			  //pwm_duty(PWMB_CH1_P74,STEER_MID);              //（C车用）
			 go_motor(1500, 1500);        //开环电池电压对转速右影响，尽量用满电的电池调
			 if(T_outku >= T_OUT_PARK1)   //T_OUT_PARK1----出库直走的时间
			 {
				 //pwm_duty(PWMB_CH1_P74,STEER_MID+STEER_LIM);	//（C车用）
				 go_motor(0, 2000);         //左打角
				 //ips114_showuint16(0,1,J_outku);
			 }
			 if(T_outku > T_OUT_PARK2)  //T_OUT_PARK2----出库打角时间（打角时间为T_OUT_PARK2-T_OUT_PARK1）
			 {
				  flag_start = 1;
				  T_outku=0;
			 }		
		  }	
	  }
		if(type ==2)
    {
			if(!flag_start)
			{
				//pwm_duty(PWMB_CH1_P74,STEER_MID);            //（C车用）
			 go_motor(1500, 1500);
			 if (T_outku >= T_OUT_PARK1) 
			 {
				 //pwm_duty(PWMB_CH1_P74,STEER_MID+STEER_LIM); //（C车用）
				 go_motor(2000, 0);  //右打角
			 }
			 if(T_outku > T_OUT_PARK2)
			 {
				  flag_start = 1;
				  T_outku=0;
			 }															
			}
    }
}
/*****************************************干簧管检测停车***************************************
函数：  void Reed(void) 
参数：  void
说明：  干簧管检测停车

*注意： 干簧管使用方法就和按键类似，通过读取IO口电平即可
返回值：void  
*********************************************************************************************/
void Reed(void)
{
	if(flag_start)//开始时不开启干簧管检测，防止出库时误测
	{
		//走过一段距离后开启干簧管检测
		if(Open_pack_time > START_T)
		{
			flag_open_reed = 1;
			Open_pack_time=0;
		}
	}
	if(flag_open_reed==0)             //干簧管检测标志位成立后才开始检测
	{
		reed_state = Reed_Switch_Pin;//干簧管状态
		if(reed_state==0)
		{
			flag_end += 1;              //识别到停车标志位开启
		}
	 }
}
/*****************************************入库函数***************************************
函数：  void Reed(void) 
参数：  void
说明：  入库函数


返回值：void  
*********************************************************************************************/
void In_park(uint8 type)
{
	if(type ==1)
  {
		if(flag_end==1)
		{
		  go_motor(0,0);
			aim_speed =0;
			while(1)
			{
				speed_measure();      //编码器测量
	      S_daoku += fabs(real_speed)*0.1;
				go_motor(-1500,-1500);
				ips114_showint16(0,3,S_daoku);
				BUZZ_ON;
			  while(S_daoku>450)
			  {
					speed_measure();      //编码器测量
				  go_motor(1500,1500);
					S_daoku_2+= fabs(real_speed)*0.1;
					ips114_showint16(0,4,S_daoku_2);
					while(S_daoku_2>100)
					{
						speed_measure();      //编码器测量
						go_motor(0,2000);
						S_daoku_3+= fabs(real_speed)*0.1;
						ips114_showint16(0,5,S_daoku_3);
					  while(S_daoku_3>150)
						{	
						 go_motor(0,0);
					   BUZZ_OFF;
						}
					}
			  }
			}
		}
 }
	if(type ==2)
  {
		if(flag_end==1)
		{
			 		  go_motor(0,0);
			aim_speed =0;
			while(1)
			{
				speed_measure();      //编码器测量
	      S_daoku += fabs(real_speed)*0.1;
				go_motor(-1500,-1500);
				ips114_showint16(0,3,S_daoku);
				BUZZ_ON;
			  while(S_daoku>450)
			  {
					speed_measure();      //编码器测量
				  go_motor(1500,1500);
					S_daoku_2+= fabs(real_speed)*0.1;
					ips114_showint16(0,4,S_daoku_2);
					while(S_daoku_2>100)
					{
						speed_measure();      //编码器测量
						go_motor(2000,0);
						S_daoku_3+= fabs(real_speed)*0.1;
						ips114_showint16(0,5,S_daoku_3);
					  while(S_daoku_3>150)
						{	
						 go_motor(0,0);
					   BUZZ_OFF;
						}
					}
			  }
			}
		}
 }
	
}


	