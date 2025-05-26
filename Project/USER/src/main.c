#include "vofa.h"
#include "headfile.h"
#include "fuse.h"
void main()
{
    DisableGlobalIRQ(); // 关闭总中断
    board_init();       // 初始化寄存器,勿删除此句代码
    delay_ms(800);      // 硬件稍微延时一下

    //	lcd_init();
    //	ips114_init_simspi();
    ips114_init(); // 1.14寸液晶屏初始化

    ADC_int();                                                    // ADC采集初始化
    ips114_showstr(0, 0, (unsigned char *)"Electromagnetic-Car"); // 字符串显示
    ips114_showstr(0, 1, "interlize...");
    //	ips114_showstr_simspi(0,0,(unsigned char*)"Electromagnetic-Car");
    //	ips114_showstr_simspi(0,1,"interlize...");
    delay_ms(500);
    //	init_Steer_PWM();                                          //舵机初始化
    init_PWM(MOTOR_MODE_SELECT); // 初始化DRV驱动方式（1-DRV/0-BTN）
    encoder_init();              // 编码器初始化
    //	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_2);//初始化无线串口
    //	pwm_init(PWMB_CH4_P77, 50, 750);                         //初始化无刺电机20%占空比
    //	pwm_init(PWMB_CH3_P33, 50, 750);                         //(1.2ms/20ms * 10000)，10000是PWM的满占空比时候的值） 10000为PWM最大值
    delay_ms(500);                             // 延时让无刷电机先转起来
    gpio_mode(Reed_Switch_Pin, GPI_IMPEDANCE); // 停车识别的干簧管IO初始化
    PID_int();                                 // PID参数初始化
    ips114_showstr(0, 2, "icm20602_int...");
    icm20602_init();

    pit_timer_ms(TIM_1, 5);           // 初始化定时器1作为周期性触发源，5MS进一次中断
    NVIC_SetPriority(TIMER1_IRQn, 3); // 设置定时器1的优先级为 3 级（最高级）

    gpio_mode(P6_7, GPO_PP); // 蜂鸣器初始化（改蜂鸣器引脚也要改这个对应的初始化）
    BUZZ_DiDiDi(250);        // 蜂鸣器滴一下

    ips114_showstr(0, 6, "intall_intok...");
    delay_ms(500);
    ips114_clear(BLUE);

    //	while(adc_valueM<6)
    //	{
    ////			电源电压显示
    //			ips114_showstr(140,0,"Value:");
    //			ips114_showfloat(190,0,adc_valueM,2,2);
    //	}

    //	 Key_Scan_Deal();      //按键调参//
    EnableGlobalIRQ(); // 初始化完毕，开启总中断
    //			串口调试
    uart_init(UART_2, UART2_RX_P10, UART2_TX_P11, 115200, TIM_2);//初始化串口

    /****下面的测试函数只用测试用，测试结束后注释关闭，一次只允许开一个测试函数！！******/
    //	Test_Servo_Hardware();//测试舵机调节
    //	Test_Motor_Hardware();//调试电机使用
    //	Test_Electric_Hardware();//测试电磁电感采集
    //	Test_Encoder();//测试编码器采集//
    /*********************************************************************************/

    while (1)
    {
        //			串口调试

        //			printf("%d,%d,%d\n",aim_speed,left_real_speed,right_real_speed);

        //	 Strategy_Slect();     //拨码开关策略选择 1--显示电感值和赛道偏差 2--选择左右出库 3--选择是否入环
        //	 Handle_Barn_Out(Library_selection);     //执行出库函数后flag_start标志位置一才会进中断

        //			停车显示
        //			Reed();
        //			ips114_showint8(0,6,Reed_Switch_Pin);
        //			ips114_showint8(0,7,flag_end);

        //     Roundabout_debugshow();//环岛调试屏幕显示（调试环岛时打开）
        // Speed_debugshow();     //速度环调试屏幕显示（调试速度环时打开，速度环建议用上位机看波形）
        //		  datasend();          //上位机发送数据

        ////			速度显示
        // ips114_showint16(50, 0, left_real_speed);
        // ips114_showint16(50, 1, right_real_speed);
        // ips114_showint16(50, 2, real_speed);
        // ips114_showint16(50, 4, aim_speed);
        // //
        // //
        // //
        //          电感显示
        // ips114_showint16(0, 0, Left_Adc);
        // ips114_showint16(0, 1, Left_Shu_Adc);
        // ips114_showint16(0, 2, Right_Shu_Adc);
        // ips114_showint16(0, 3, Right_Adc);
        // ips114_showint16(0, 0, adc_value[0]);
        // ips114_showint16(0, 1, adc_value[1]);
        // ips114_showint16(0, 2, adc_value[2]);
        // ips114_showint16(0, 3, adc_value[3]);

        // ////			测距显示（后面注释）
        // //			ips114_showuint16(100,5,dl1a_distance_mm);
        // //			ips114_showuint16(100,1,flag_obstacle);
        // //
        // ////			赛道偏差显示
        // ips114_showfloat(0, 5, Current_Dir, 2, 1); // 显示小点数   整数显示2位   小数显示1位
        // //
        // //
        //			电源电压显示
        // ips114_showstr(140, 0, "Value:");
        // ips114_showfloat(190, 0, adc_valueM, 2, 2);
        vofa_demo2((float)left_real_speed, (float)right_real_speed);
    }
}
