#ifndef __HEADFILE_H_
#define __HEADFILE_H_

#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "intrins.h"
//------STC32G SDK等
#include "STC32Gxx.h"
#include "board.h"
#include "common.h"

//------逐飞科技单片机外设驱动头文件
#include "zf_uart.h"
#include "zf_gpio.h"
#include "zf_iic.h"
#include "zf_adc.h"
#include "zf_spi.h"
#include "zf_tim.h"
#include "zf_pwm.h"
#include "zf_nvic.h"
#include "zf_exti.h"
#include "zf_delay.h"
#include "zf_eeprom.h"

//------逐飞科技产品驱动头文件
#include "SEEKFREE_FONT.h"
#include "SEEKFREE_18TFT.h"

#include "SEEKFREE_ICM20602.h"
#include "SEEKFREE_TSL1401.h"
#include "SEEKFREE_IPS114_SPI.h"
#include "SEEKFREE_MPU6050.h"
#include "SEEKFREE_OLED.h"
#include "SEEKFREE_ABSOLUTE_ENCODER.h"
#include "SEEKFREE_WIRELESS.h"
#include "SEEKFREE_PRINTF.h"
#include "SEEKFREE_AT24C02.h"
#include "SEEKFREE_BLUETOOTH_CH9141.h"
#include "SEEKFREE_WIRELESS_CH573.h"
#include "SEEKFREE_CONFIG.h"
#include "SEEKFREE_IMU660RA.h"
#include "SEEKFREE_IMU963RA.h"
#include "SEEKFREE_DL1A.h"

//用户自定义头文件
#include "fuse.h"
#include "ADC.h"
#include "PID.h"
#include  "speed.h" 
#include "debug.h"
#include "park.h"

/*****************使用的硬件引脚定义*********
为了增强程序的兼容性，大部分常用引脚使用宏定义，如果硬件有差别修改下面宏定义即可！
注意修改时，请go to到到对应模块进行粘贴复制，不要自己敲哦！
*********************************************/

//电机引脚 
//MOTOR_MODE  0的话是两路模式MOS驱动，1的话是一路模式DRV驱动
#define MOTOR_MODE_SELECT  1
//=====一个电机需要两路PWM模式=====MOS/btn驱动都行=====
#define Left_Z_Pin     PWMA_CH1P_P60   //PWMA_CH1P_P60 PWMB_CH2_P75
#define Left_F_Pin     PWMA_CH2P_P62  //PWMA_CH2P_P62  PWMB_CH1_P74
#define Right_Z_Pin    PWMA_CH3P_P64  //PWMA_CH3P_P64  PWMB_CH3_P76
#define Right_F_Pin    PWMA_CH4P_P66  //PWMA_CH4P_P66  PWMB_CH4_P77
//=====一个电机只要一路PWM模式，另外一个引脚控制方向=====DRV驱动==
#define Left_PWM_Pin   PWMA_CH4P_P66    //PWMA_CH4P_P66
#define Left_DIR_Pin   P64            //P62  
#define Right_PWM_Pin  PWMA_CH2P_P62    //PWMA_CH3P_P64
#define Right_DIR_Pin  P60            //P60

//舵机引脚   
#define Steer_Pin   PWMB_CH1_P74    //

//编码器引脚  占用了定时器TIM0、TIM3、TIM4  
#define Left_Ecoder_Pin1     CTIM0_P34  //CTIM3_P04   CTIM0_P34    //LSB引脚
#define Left_Ecoder_Pin2     P35        //P05         P30           //Dir方向引脚
#define Right_Ecoder_Pin1    CTIM3_P04   //CTIM4_P06  CTIM3_P04     //LSB引脚
#define Right_Ecoder_Pin2    P53          //Dir                     方向引脚P07

//串口引脚     占用定时器TIM2
#define UART_User    UART_4          //
#define UART_TX_Pin  UART4_TX_P03   //UART4_TX_P03 
#define UART_RX_Pin  UART4_RX_P02  //UART4_RX_P02 

//蜂鸣器引脚
#define BUZZPin     P67

//ADC电磁信号引脚5个电感（实际上都可以，电感，根据需求修改即可）
#define Left_ADC_Pin      ADC_P00  
#define LeftXie_ADC_Pin   ADC_P01
#define RightXie_ADC_Pin  ADC_P05 
#define Right_ADC_Pin     ADC_P06

#define Mid_ADC_Pin       ADC_P15 

//干簧管引脚 (默认使用普通IO读取，外部中断和普通的效果差不多，统一使用普通IO读取)
#define Reed_Switch_Pin P26  //


#endif