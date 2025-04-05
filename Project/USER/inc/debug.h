#ifndef _debug_h
#define _debug_h

#include "headfile.h"


#define BUZZ_ON            BUZZPin = 1;          //开蜂鸣器   
#define BUZZ_OFF           BUZZPin = 0;         //关蜂鸣器

//#define BUZZ_ON           pwm_duty(PWMA_CH4N_P67, 9500);          //开蜂鸣器   
//#define BUZZ_OFF          pwm_duty(PWMA_CH4N_P67, 0);         //关蜂鸣器

  

//外部函数声明
void Data_Send(UARTN_enum uratn,signed short int *pst);
void datasend(void);
void display( int hang,int date);
void Switch_Key_init(void);         
void Strategy_Slect(void) ;          
void Key_Scan_Deal (void );          
void BUZZ_DiDiDi(uint16 PinLV);              
void Test_Servo_Hardware(void);
void Test_Motor_Hardware (void);
void Test_Electric_Hardware (void);
void Test_Encoder(void);
float StrToDouble(const char *s);
void extern_iap_write_float(double dat,uint8 num,uint8 pointnum,uint16 addr);
float iap_read_float(uint8 len, uint16 addr);
void EEROM_CanshuInit(); 

void Roundabout_debugshow(void);
void Speed_debugshow(void);

//外部变量申明
extern unsigned char TiaoCan;            
extern unsigned char TFT_SHOW;          
extern unsigned char Switch1,Switch2,Switch3,Switch4,Switch5,Switch6;


#endif  
