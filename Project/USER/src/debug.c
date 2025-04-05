#include "debug.h"
#include "math.h"
/*************************使用说明****************************************
本协议与“Visual Scope”软件协议兼容，用过的可以直接用原来的下位机协议即可
首次使用时：
1.将“outputdata.c”和“outputdata.h”添加到你的工程中
2.在“outputdata.c”中包含你原程序的串口发送函数头文件
3.将uart_putchar(databuf[i]);语句替换为你的串口字节发送函数，如send_char(databuf[i]);
4.在你的程序需要发送波形数据的.c文件中添加包含：#include "outputdata.h"，并在本文件中调用函数OutPut_Data(x,y,z,w);
  其中形参x，y，z，w就是传入四个short int 16位数据，分别对应通道1,2,3,4
************************************************************************/
//此处添加你的串口头文件包含！！！！！！！！！！！！
#include "zf_uart.h"
//****************************移植**************************//

void Data_Send(UARTN_enum uratn,signed short int *pst)
{
		unsigned char _cnt=0; unsigned char sum = 0;
    unsigned char data_to_send[23] = {0};         //发送缓存
    unsigned char i;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=(unsigned char)(pst[0]>>8);  //高8位
    data_to_send[_cnt++]=(unsigned char)pst[0];  //低8位
    data_to_send[_cnt++]=(unsigned char)(pst[1]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[1];
    data_to_send[_cnt++]=(unsigned char)(pst[2]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[2];
    data_to_send[_cnt++]=(unsigned char)(pst[3]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[3];
    data_to_send[_cnt++]=(unsigned char)(pst[4]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[4];
    data_to_send[_cnt++]=(unsigned char)(pst[5]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[5];
    data_to_send[_cnt++]=(unsigned char)(pst[6]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[6];
    data_to_send[_cnt++]=(unsigned char)(pst[7]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[7];
    data_to_send[_cnt++]=(unsigned char)(pst[8]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[8];


    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

        for(i=0;i<_cnt;i++)
    uart_putchar(uratn,data_to_send[i]);
}

//===================================================上位机相关的==========================================================
//======================================================================================================================
/****************数据传输******************
函数：void datasend()
参数：  无
说明： 可以同时传输6个数  icm_gyro_x   icm_acc_x ICM_Real.gyro.y  ICM_Real.acc.z   
常看的变量：icm_acc_x  icm_gyro_y  Angle  adc_date[0] Left_Adc

返回值：无
日期：
作者：  */
void datasend()
{  
   short send_data[6];                      

   send_data[0]= aim_speed;//left_speed; ////////ICM_Start.acc.x 
   send_data[1]= left_real_speed;//right_speed; //////////////////    MpuStart.gyro.x   Angle  
   send_data[2]= aim_speed; //////////
   send_data[3]= right_real_speed;//imu660ra_gyro_z; //
   send_data[4]= 0;//GORY_Z;
   send_data[5]= 0;
	 //Data_Send(UART_4,send_data);
   Data_Send(UART_User,send_data);
}

float StrToDouble(const char *s)
{
	int i = 0;
	int k = 0;
	float j;
	int flag =1;
	float result = 0.0;
	if (s[i] == '+')
	{
		i++;
	}
	if (s[i] == '-')
	{
		i++;
		flag = -1;
	}
	while (s[i] != '\0' && s[i] != '.')
	{
		j = (s[i] - '0')*1.0;
		result = result * 10 + j;
		i++;
	}
	if (s[i] == '.')
	{
		i++;
		while (s[i] != '\0'&&s[i] != ' ')
		{
			k++;
			j = s[i] - '0';
			result = result + (1.0 * j) / pow(10.0, k);   
			i++;
		}
	}
	result = flag * result;
	return result;
}

void extern_iap_write_float(double dat,uint8 num,uint8 pointnum,uint16 addr)
{
  uint8   length;
	int8    buff[34];
	int8    start,end,point;

	if(0>dat)   length = zf_sprintf( &buff[0],"%f",dat);//负数
	else
	{
		length = zf_sprintf( &buff[1],"%f",dat);
		length++;
	}
	point = length - 7;         //计算小数点位置
	start = point - num - 1;    //计算起始位
	end = point + pointnum + 1; //计算结束位
	while(0>start)//整数位不够  末尾应该填充空格
	{
		buff[end] = ' ';
		end++;
		start++;
	}
    
	if(0>dat)   buff[start] = '-';
  else        buff[start] = '+';
    
	buff[end] = '\0';

	extern_iap_write_bytes(addr,(uint8 *)buff,num+pointnum+3);
}

float iap_read_float(uint8 len, uint16 addr)
{
	uint8 buf[34];
	iap_read_bytes(addr, buf, len);
	
	return StrToDouble(buf);
}

void EEROM_CanshuInit() //PID初始化
{
//向eeprom写入数据，在写入的时候记得把擦除eeprom区勾上
//	extern_iap_write_float(30,3,1,0x00);//(30,3,1,0x00)中的30代表要写入的数，3代表整数位，1代表小数位。这句话的意思是把30转换为"+030.0"存入0x00-0x06地址中，因为"+030.0"占六个地址，加上字符串的结束字符'/0'，所以占了七个地址，所以是0x00到0x06,下个数据要存以到0x07为起始地址。
//	extern_iap_write_float(15,3,1,0x07);
//	extern_iap_write_float(11000,5,1,0x0e);
//	extern_iap_write_float(315,4,1,0x17);
//	extern_iap_write_float(2.0,1,1,0x1f);
//	extern_iap_write_float(1.5,1,1,0x24);
//	extern_iap_write_float(680,3,1,0x29);
	
//从eeprom中读取数据，记得不要把擦除eeprom区勾上
	  //SpeedPID.Kp=iap_read_float(7,0x00);
	  //iap_read_bytes(0x00,SpeedPID.Kp,2);
	  //TurnPID.Ki=iap_read_float(7,0x0e);
//	TurnPID.Kp= iap_read_float(7,0x00);//这句话的意思是从0x00地址开始取出7个地址的数据，也就是0x00到0x06。
//  TurnPID.Ki = iap_read_float(7,0x07);
//  TurnPID.Kd = iap_read_float(9,0x0e);
	  //iap_read_bytes(0x00,(uint16 *)"350",2); //aim_speed=
//  MotorPID.I = iap_read_float(8,0x17);
//	Stright=iap_read_float(5,0x1f);
//	Curve=iap_read_float(5,0x24);
//	pout0=iap_read_float(7,0x29);
}
//====================================================屏幕相关的=(已删除)=============================================================
//============================================================================================================================
//sprintf(temp," date20=%d",date);
//TFTSPI_P8X8Str(0,19,temp,BLACK,WHITE);break;

//==========================================================拨码开关及按键相关=========================================================
//====================================================================================================================================

//拨码开关引脚宏定义
#define Switch_Pin_1       P75
#define Switch_Pin_2       P76
#define Switch_Pin_3       P11
#define Switch_Pin_4       P61
#define Switch_Pin_5       P14
#define Switch_Pin_6       P15
//定义按键引脚
#define KEY1    P70      
#define KEY2    P71      
#define KEY3    P72        
#define KEY4    P73       

//***************函数宏定义****(下面这些函数请修改宏定义为对应的GPIO库函数操作)***********
#define KEY_INT(key_x)           gpio_pull_set(key_x,PULLUP)//配置为上拉输出   
#define SWITCH_INT(switch_x)     gpio_pull_set(switch_x,PULLUP)//配置为上拉电阻
#define READ_GPIO(Pin_X)         Pin_X
#define TiaoCan_DelayMs(M_S)     delay_ms(M_S)   //延时

unsigned char TiaoCan=0;////////////////////////调参标志位
unsigned char TFT_SHOW=0;///////////////////////屏幕开关
unsigned char Switch1=0,Switch2=0,Switch3=0,Switch4=0,Switch5=0,Switch6=0;//拨码
char parameter=0;//参数选择

//开关状态变量
uint8 key1_status = 1,key2_status = 1,key3_status = 1, key4_status = 1,key5_status = 1;
//上一次开关状态变量
uint8 key1_last_status, key2_last_status, key3_last_status, key4_last_status,key5_last_status;
//开关标志位
uint8 key1_flag=0,key2_flag=0,key3_flag=0, key4_flag=0,key5_flag=0;
/*****************拨码开关及按键初始化*****************
函数：void Switch_Key_init()
功能：初始化IO
参数：  无
说明： 初始化IO口   gpio_init(D1, GPI, GPIO_HIGH, GPI_PULL_UP); GPO_PUSH_PULL
返回值：无*/
void Switch_Key_init()
{

  //拨码开关初始化  （无需修改，请勿修改）
	SWITCH_INT(Switch_Pin_1) ;
	SWITCH_INT(Switch_Pin_2) ;
	SWITCH_INT(Switch_Pin_3) ;
	SWITCH_INT(Switch_Pin_4) ;
	SWITCH_INT(Switch_Pin_5) ;
	SWITCH_INT(Switch_Pin_6) ;
    
  //按键初始化 （无需修改，请勿修改）
  KEY_INT(KEY1);
	KEY_INT(KEY2);
	KEY_INT(KEY3);
	KEY_INT(KEY4);
}

/*****************拨码开关策略选择*****************
函数：void Strategy_Slect()
功能：通过拨码开关调整策略
参数：  无
说明：  6位拨码开关，如果有增加或者减少可对照修改,如果不足6个也不要删除多余的，多余的你随便引脚改个没用的即可
        使用你定义的就好了，其他没有用到的无需关心
返回值：无*/
void Strategy_Slect()
{
      //读取拨码开关状态
      if(!READ_GPIO(Switch_Pin_1))//用
      {
       Switch1=1;
			 Annulus_selection=1;
//			 //显示电感原始值
//			 ips114_showint16(0,0,adc_value[0]);
//	     ips114_showint16(0,1,adc_value[1]);
//	     ips114_showint16(0,2,adc_value[2]);
//	     ips114_showint16(0,3,adc_value[3]);
//				
//			 //显示电感归一化值
//			 ips114_showuint8(80,0,Left_Adc);
//	     ips114_showuint8(80,1,Left_Shu_Adc);
//	     ips114_showuint8(80,2,Right_Shu_Adc);
//	     ips114_showuint8(80,3,Right_Adc);
//				
//			 ips114_showfloat(0,5,Current_Dir,2,1);//显示浮点数   整数显示2位   小数显示1位
//				
//			 ips114_showint16(130,0,left_speed);
//  	   ips114_showint16(130,1,right_speed);
      }
      if(!READ_GPIO (Switch_Pin_2))//用
      {
       Switch2=1;
			 Library_selection=2;   //出库选择，Library_selection默认为1，左出库，拨下后变成2，右出库
      }
      if(!READ_GPIO (Switch_Pin_3))
      {
       Switch3=1;
			 
      }
      if(!READ_GPIO (Switch_Pin_4))
      {
       Switch4=1;

      }
      if(!READ_GPIO (Switch_Pin_5))
      {
       Switch5=1;

      }
      if(!READ_GPIO (Switch_Pin_6))
      {
       Switch6=1;
 
      }
	
}

/*****************按键扫描读取*****************
函数：void  Key_Scan_Deal ()
功能：读取按键并执行对应操作
参数：  无
说明： 参考逐飞例程 ，5位按键，如果有增加或者减少可对照修改
      // 1号为左移键，2号为上键，3号为右移键，4号为中键盘，5号为下键
     //本次程序没有使用调参，stc单片机下载程序也快，改了烧就可以，如果要加的话自己根据下面的自己加就可以
返回值：无     */
uint8 gogo=0;
void  Key_Scan_Deal ()
{
	while(gogo<=2)
	{
    //使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费
    //保存按键状态
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;
    //读取当前按键状态
    key1_status = READ_GPIO(KEY1);
    key2_status = READ_GPIO(KEY2);
    key3_status = READ_GPIO(KEY3);
    key4_status = READ_GPIO(KEY4);
    //检测到按键按下之后  并放开置位标志位
    if(key1_status && !key1_last_status)    key1_flag = 1;
    if(key2_status && !key2_last_status)    key2_flag = 1;
    if(key3_status && !key3_last_status)    key3_flag = 1;
    if(key4_status && !key4_last_status)    key4_flag = 1;
    if(key5_status && !key5_last_status)    key5_flag = 1;
    //标志位置位之后，可以使用标志位执行自己想要做的事件
 
    if(key1_flag) //S1键，参数加
    {
      key1_flag = 0;//使用按键之后，应该清除标志位
      /*以下为用户任务  */
      switch(parameter)
      {
      //-----------------------调参请修改下面--（注意修改对应的显示）----------------------------------------------------------------
      //第一页显示的东西
        case 0:TurnPID.Kp+=1; break;//extern_iap_write_float(TurnPID.Kp,3,2,0x00);      break;
        case 1:TurnPID.Ki+=0.1;  break;//extern_iap_write_float(TurnPID.Ki,3,2,0x07);			 break;
        case 2:TurnPID.Kd+=0.1;  break;//extern_iap_write_float(TurnPID.Kd,3,2,0x0e);			 break;
        case 3:;TurnPID.K_gory+=1;  break;
        case 4:;  break;
        case 5:;  break;
      /// case 6:  ; break;//这个不能加任何操作在这里了，翻页使用啦 extern_iap_write_bytes(0x00,&aim_speed,2);
      //第二页显示的东西
        case 7:SpeedPID.Kp+=1;   break;
				case 8:SpeedPID.Ki+=0.1; break;
				case 9:SpeedPID.Kd+=0.1; break;
				case 10:aim_speed+=10;   break;
        case 11: break;
        case 12: break;
       //--------------------调参请修改上面------------------------------------------------------------------
       }
     }
     if(key2_flag)//S2键，参数减
     {
       key2_flag = 0;//使用按键之后，应该清除标志位
      /*  以下为用户任务  */
       switch(parameter)
       {
       //----------------------调参请修改下面--（注意修改对应的显示）--------------------------------------------------------------
       //第一页显示的东西
        case 0:TurnPID.Kp-=20;       break;
        case 1:TurnPID.Ki-=5;        break;
        case 2:TurnPID.Kd-=5;        break;
				case 3:;  TurnPID.K_gory-=5;break;
        case 4:;  break;
        case 5:;  break;
        //case 6:ips114_showstr(0,0,"FANYE_ing");  ; break;//这个不能加任何操作在这里了，翻页使用啦
       //第二页显示的东西 
				case 7:SpeedPID.Kp-=1;   break;
				case 8:SpeedPID.Ki-=0.1; break;
				case 9:SpeedPID.Kd-=0.1; break;
				case 10:aim_speed-=50;   break;
				case 11: break;
				case 12: break;
       //--------------------调参请修改上面------------------------------------------------------------------
       } 
      }
      if(key3_flag)//S3键，选择调参的参数
      {
        key3_flag = 0;//使用按键之后，应该清除标志位               
        parameter++;
				ips114_clear(WHITE);
				if(parameter>=11) parameter=0;
      }
      if(key4_flag)//S4键，确认按键，按三下退出调参
      {
        key4_flag = 0;//使用按键之后，应该清除标志位
        gogo++;
      }
    //*******************************屏幕显示第一页***********************

    if(parameter<6)//显示参数0到5，实际显示1到6
    {
			ips114_showstr(0,parameter+1,"->");
			ips114_showstr(20,0,"parameter->");    ips114_showint8(150,0,parameter);//x为int8类型
      //显示调参过程   行号最后一行为9   只能显示这么多，更多重新翻页显示  一页调6个参数
      ips114_showstr(20,1,"TurnPID.Kp=");    ips114_showfloat(150,1,TurnPID.Kp,3,2);//显示浮点数  
      ips114_showstr(20,2,"TurnPID.Ki=");    ips114_showfloat(150,2,TurnPID.Ki,2,2);//显示浮点数   
      ips114_showstr(20,3,"TurnPID.Kd=");    ips114_showfloat(150,3,TurnPID.Kd,2,2);//显示浮点数   
		  ips114_showstr(20,4,"TurnPID.K_gory=");      ips114_showint16(150,4,TurnPID.K_gory);
		        
    }
    //*******************************屏幕显示第二页**************************************************
    if(parameter>6&&parameter<11)//这里行号从4到9   一页调6个参数  //显示参数7到5，实际显示7到12
    {
      ips114_showstr(0,parameter-6,"->");
			ips114_showstr(20,0,"parameter->");    ips114_showint8(150,0,parameter);//x为int8类型
      //显示调参过程   行号最后一行为9   只能显示这么多，更多重新翻页显示  一页调6个参数
      ips114_showstr(20,1,"SpeedPID.Kp=");    ips114_showfloat(150,1,SpeedPID.Kp,2,2);//显示浮点数  
      ips114_showstr(20,2,"SpeedPID.Ki=");    ips114_showfloat(150,2,SpeedPID.Ki,2,2);//显示浮点数   
      ips114_showstr(20,3,"SpeedPID.Kd=");    ips114_showfloat(150,3,SpeedPID.Kd,2,2);//显示浮点数   
		  ips114_showstr(20,4,"aim_speed=");      ips114_showint16(150,4,aim_speed); 
//		  ips114_showstr(20,5,"Turn_NeiPID.Ki=");ips114_showfloat(150,5,Turn_NeiPID.Ki,2,2);//显示浮点数 
//		  ips114_showstr(20,6,"Turn_NeiPID.Kd=");ips114_showfloat(150,6,Turn_NeiPID.Kd,2,2);//显示浮点数    
    }
    //*******************************屏幕显示第三页**************************************************
//    if(parameter>13&&parameter<20)
//    {
//                  
//    }
    //###########还需要更多页仿照着写就可以咯######################这里就不写了 结束
    if(parameter==6||parameter==13||parameter==20)  //翻页准备
    {
			 ips114_showstr(0,0,"Ready to turn the page...");
			 ips114_showstr(0,1,"Press the button to display ");
			 ips114_showstr(0,2,"the second page...");
      // ips114_clear(WHITE);              
    }//清屏
		if(gogo>2)
    {
		  ips114_clear(WHITE); 
    }
	}
}
//===============================================调试环岛屏幕显示=============================================
//============================================================================================================
/************************************************调试环岛屏幕显示*********************************************
函数：void Roundabout_debugshow(void)
功能：调试环岛时显示相应的标志位，方便调试
参数：  无
说明：
返回值：无 
**************************************************************************************************************/
void Roundabout_debugshow(void)
{
	ips114_showuint8(0,0,Left_Adc);      //左横电感值
  ips114_showuint8(0,1,Left_Shu_Adc);  //左竖电感值
  ips114_showuint8(0,2,Right_Shu_Adc); //右竖电感值
  ips114_showuint8(0,3,Right_Adc);     //右横电感值
	ips114_showfloat(0,5,Current_Dir,2,1);//显示浮点数   整数显示2位   小数显示1位	
	
	ips114_showuint8(50,0,road_type.annulus);         //环岛标志位
	ips114_showint16(50,1,annulus_s);                 //过环岛三角区编码器积分距离
  ips114_showuint8(50,2,road_type.in_annulus_left); //左环岛进环打角标志位
	ips114_showint16(50,3,annulus_z);                 //进环打角陀螺仪积分角度值
	ips114_showint16(50,4,annulus_s2);                 
	ips114_showuint8(50,5,road_type.on_annulus_left); //在左环岛标志位（认为小车已经入环）
	ips114_showuint8(50,6,road_type.out_annulus);     //出环岛标志位
	ips114_showint16(50,7,annulus_t);                 //出环岛定时清0时间
	
	
	ips114_showuint8(100,0,road_type.annulus);          //环岛标志位
	ips114_showint16(100,1,annulus_s);                  //过环岛三角区编码器积分距离
	ips114_showuint8(100,2,road_type.in_annulus_right); //左环岛进环打角标志位
	ips114_showint16(100,3,annulus_z);                  //进环打角陀螺仪积分角度值
	ips114_showuint8(100,4,road_type.on_annulus_right); //在右环岛标志位（认为小车已经入环）
	ips114_showuint8(100,5,road_type.out_annulus);      //出环岛标志位
	ips114_showint16(100,6,annulus_t);                  //出环岛定时清0时间
}

//===============================================调试速度环幕显示=============================================
//============================================================================================================
/************************************************调试速度环幕显示*********************************************
函数：Speed_debugshow(void)
功能：调试速度环时显示相应左右编码器的值，方便调试（速度环推荐用上位机看波形调试）
参数：  无
说明：
返回值：无 
**************************************************************************************************************/
void Speed_debugshow(void)
{
	ips114_showint16(0,0,left_real_speed);  //左编码器的值
  ips114_showint16(0,1,right_real_speed); //右编码器的值
  ips114_showint16(0,2,real_speed);       //左右轮平均值
	
	
}
//===============================================蜂鸣器相关=============================================
//=====================================================================================================

//蜂鸣器开和关 被写在对应头文件去了，去H文件查看

/*****************蜂鸣器滴滴滴*****************
函数：void BUZZ_DiDiDi()
功能：蜂鸣器滴滴滴
参数：  无
说明：
返回值：无 */
void BUZZ_DiDiDi(uint16 PinLV)
{
  BUZZ_ON;
  TiaoCan_DelayMs(PinLV);
  BUZZ_OFF;
}


/***************************测试完毕**********************************************
 *  函数名称：Test_Servo(void)
 *  功能说明：舵机PWM初始化，测试标定输出PWM控制SD5/S3010舵机
 *  参数说明：无
 *  函数返回：无
 *  修改时间：
 *  备    注：参考龙邱库提供的！！！！
 【注意事项】注意，一定要对舵机打角进行限制
 使用龙邱母板测试流程：
 1.先使用万用表测量电池电压，务必保证电池电压在7V以上，否则无力不反应！
 2.然后确定舵机供电电压，SD5舵机用5V供电，S3010用6-7V供电，SD012舵机用5V供电！！！
 3.把舵机的舵盘去掉，让舵机可以自由转动；
 4.烧写程序并运行，让舵机转动到中值附近；如果没反应重复1-2步，或者调整舵机的PWM频率计占空比，能受控为准；
 5.舵机受控后用手轻转，舵机会吱吱响，对抗转动，此时可以装上舵盘；
 6.按键K0/K1确定舵机的左右转动极限，并记下来，作为后续限幅防止舵机堵转烧毁！
 *************************************************************************/
void Test_Servo_Hardware (void)
{
    char txt[16] = "X:";
    unsigned int  duty = Steer_Duty_Midle;

    ips114_clear(YELLOW);  //初始清屏
	  ips114_showstr(0, 0, "Test_Servo_Hardware:");
   	pwm_init(Steer_Pin, 50, Steer_Duty_Midle);     //初始化舵机  输出PWM频率200HZ，并设置中值
    pwm_duty(Steer_Pin, Steer_Duty_Midle);
    while (1)
    {
        if (!READ_GPIO(KEY1))
        {
            if (duty > 100)  		//防止duty超
            {
                duty += 10;     //标定的时候，可以把步长改小点，比如10
            }
        }
        if (! READ_GPIO(KEY3))
        {
            duty = Steer_Duty_Midle;
        }
        if (! READ_GPIO(KEY2))
        {
            duty -= 10;
        }
			  pwm_duty(Steer_Pin, duty);
				sprintf(txt, "Servo:%05d ", duty);
				ips114_showstr(1, 2, txt); //显示
				TiaoCan_DelayMs(100);        
    }
}

/****************************测试完毕*********************************************
 *  函数名称：TestMotor(void)
 *  功能说明：测试标定输出PWM控制电机
 *  参数说明：无
 *  函数返回：无
 *  修改时间：
 *  备    注：驱动2个电机
 【注意事项】注意，一定要对电机输出进行限制
 使用龙邱母板测试流程：
 1.先使用万用表测量电池电压，务必保证电池电压在7V以上，否则无力不反应！
 2.接好母板到驱动板的信号线及电源线；
 3.接好驱动板到电机的导线；
 4.烧写程序并运行，确定电机能正常转动后，开启驱动板电源开关；
 5.按键K0/K1确定电机转动速度及方向；
 6.如果出现疯转，按下K1键返回低速模式，或者直接关闭驱动板电源！
 *************************************************************************/
void Test_Motor_Hardware (void)
{
    int16 motor_duty = 3000;
    lcd_clear(YELLOW);  //初始清屏
	  lcd_showstr(2, 0, "Test_Motor_Hardware:");
    init_PWM(MOTOR_MODE_SELECT);
	
    while (1)
    {
    if (!READ_GPIO(KEY1))   //按下KEY1键   左轮单独正转
        {
           go_motor (motor_duty,0);
					 lcd_showstr(0, 4, "Left  Front");   //字符串显示
        }
       if (!READ_GPIO(KEY3)) //按下KEY2键，左右轮同时反转
        {
          	 go_motor (-motor_duty,-motor_duty);
					   lcd_showstr(0, 4, "All  Black");   //字符串显示
           	  	 
        }
       if (!READ_GPIO(KEY2))      //按下KEY3键  右轮单独正转
        {
           	 go_motor (0,motor_duty);
						 lcd_showstr(0, 4, "Right Front");   //字符串显示
        }
				if((READ_GPIO(KEY1))&&(READ_GPIO(KEY2))&&(READ_GPIO(KEY3)))
        go_motor (0,0);
      	TiaoCan_DelayMs(100);  
    }
}

/****************************测试完毕*********************************************
 *  函数名称：void Test_Electric_Hardware (void)
 *  功能说明：测试电磁电感硬件
 *  参数说明：无
 *  函数返回：无
 *  备    注：
 【注意事项】
 *************************************************************************/
void Test_Electric_Hardware (void)
{
	char txt[16];
	ips114_clear(YELLOW);  //初始清屏
	ips114_showstr(2, 0, "Test_Electric_Hardware:");
	ADC_int();
	while(1)
	{
//		    if (!READ_GPIO(KEY1)) //按下KEY1键
//        {
					lcd_showstr(2, 1, "Normalize_Deal....");   //字符串显示
					ADC_Collect();  //电感采值
					
					sprintf(txt,"adc0= %05d",adc_value[0]);
					lcd_showstr(1, 2, txt); //显示
					sprintf(txt,"adc1= %05d",adc_value[1]);
					lcd_showstr(1, 3, txt); //显示
					sprintf(txt,"adc2= %05d",adc_value[2]);
					lcd_showstr(1, 4, txt); //显示
					sprintf(txt,"adc3= %05d",adc_value[3]);
					lcd_showstr(1, 5, txt); //显示				  	  	 
//        }
				if(!READ_GPIO(KEY2)) //按下KEY2键
				{
					lcd_showstr(2, 1, "GYH_Normalize_Deal....");   //字符串显示
					ADC_Collect();  //电感采值
	        Data_current_analyze();  //电感值归一化函数
					Current_Dir=Cha_bi_he(Left_Adc,Right_Adc,100);
					
					sprintf(txt,"adc0= %05d",Left_Adc);
					lcd_showstr(1, 2, txt); //显示
					sprintf(txt,"adc1= %05d",Left_Shu_Adc);
					lcd_showstr(1, 3, txt); //显示
					sprintf(txt,"adc2= %05d",Right_Shu_Adc);
					lcd_showstr(1, 4, txt); //显示
					sprintf(txt,"adc3= %05d",Right_Adc);
					lcd_showstr(1, 5, txt); //显示
					sprintf(txt,"Current_Dir= %05d",Current_Dir);
					lcd_showstr(1, 6, txt); //显示
        }
  }	
}
/****************************测试完毕*********************************************
 *  函数名称：void Test_Encoder(void)
 *  功能说明：测试编码器
 *  参数说明：无
 *  函数返回：无
 *  修改时间：
 *  备    注：
 【注意事项】
 *************************************************************************/
void Test_Encoder(void)
{
	 char txt[16];
   encoder_init();//编码器初始化
	 lcd_clear(YELLOW);  //初始清屏
	 lcd_showstr(2, 0, "Test_Encoder:"); 					
	 while(1)
	 {
		  speed_measure();
		  delay_ms(50);
			sprintf(txt,"Left_Speed  = %05d",left_speed);
		  lcd_showstr(1, 3, txt); //显示
			sprintf(txt,"Right_Speed = %05d",right_speed);
		  lcd_showstr(1, 4, txt); //显示
		  sprintf(txt,"Real_Speed = %05d",real_speed);
		  lcd_showstr(1, 5, txt); //显示
	 }
}


