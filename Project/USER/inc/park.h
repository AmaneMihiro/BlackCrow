#ifndef _park_h
#define _park_h

#include "headfile.h"

#define START_T         10000
#define T_OUT_PARK1      1900
#define T_OUT_PARK2      2400

//变量声明
extern int8   flag_start;          //出库完毕
extern int8   flag_open_reed;      //屏蔽干簧管
extern int8   reed_state;          //干簧管状态
extern int8   flag_end;            //开始停车标志位
extern uint16 T_outku;    //出库直走和打角定时
extern uint16 J_outku;
extern uint16 T_inku_wait;
extern uint16 T_inku_J;
extern uint16 T_inku_S;
extern uint16 T_inku;

extern uint16 S_daoku;
extern uint8 Library_selection;

//函数申明
extern void Handle_Barn_Out(uint8 type);
extern void Reed(void);
extern void In_park(uint8 type);

#endif