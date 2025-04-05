
#include "headfile.h"
void main()
{
	DisableGlobalIRQ(); // �ر����ж�
	board_init();		// ��ʼ���Ĵ���,��ɾ���˾���롣
	delay_ms(800);		// ������΢��ʱһ��

	//	lcd_init();
	ips114_init(); // 1.14��Һ������ʼ��

	ADC_int();													  // ADC�ɼ���ʼ��
	ips114_showstr(0, 0, (unsigned char *)"Electromagnetic-Car"); // �ַ�����ʾ
	ips114_showstr(0, 1, "interlize...");
	delay_ms(500);
	//	init_Steer_PWM();                                          //�����ʼ��
	init_PWM(MOTOR_MODE_SELECT); // ��ʼ��DRV������ʽ��1-DRV/0-BTN��
	encoder_init();				 // ��������ʼ��
	//	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_2);//��ʼ�����ߴ���
	//	pwm_init(PWMB_CH4_P77, 50, 750);                         //��ʼ����ˢ���20%ռ�ձ�
	//	pwm_init(PWMB_CH3_P33, 50, 750);                         //(1.2ms/20ms * 10000)��10000��PWM����ռ�ձ�ʱ���ֵ�� 10000ΪPWM���ֵ
	delay_ms(500);							   // ��ʱ����ˢ�����ת����
	gpio_mode(Reed_Switch_Pin, GPI_IMPEDANCE); // ͣ��ʶ��ĸɻɹ�IO��ʼ��
	PID_int();								   // PID������ʼ��
	// ips114_showstr(0,2,"icm20602_int...");
	// icm20602_init();
	// ips114_showstr(0,3,"icm20602_intok...");
	// delay_ms(500);
	// ips114_showstr(0,4,"dlla_int...");
	// delay_ms(500);

	//	dl1a_init();                                               //����ģ���ʼ��
	// ips114_showstr(0,5,"dlla_intok...");

	pit_timer_ms(TIM_1, 5);			  // ��ʼ����ʱ��1��Ϊ�����жϴ�������5MS��һ���ж�
	NVIC_SetPriority(TIMER1_IRQn, 3); // ���ö�ʱ��1�ж����ȼ�Ϊ 3 ������߼���

	gpio_mode(P6_7, GPO_PP); // ��������ʼ�����ķ���������ҲҪ�������Ӧ�ĳ�ʼ����
	BUZZ_DiDiDi(250);		 // ��������һ��

	ips114_showstr(0, 6, "intall_intok...");
	delay_ms(500);
	ips114_clear(BLUE);
	//	 Key_Scan_Deal();      //��������//
	EnableGlobalIRQ(); // ��ʼ����ϣ��������ж�

	/****����Ĳ��Ժ���ֻ�ǲ����ã����Խ�����ע�͹رգ�һ��ֻ������һ�����Ժ�������******/
	//	Test_Servo_Hardware();//���Զ����ֵ
	//	Test_Motor_Hardware();//���Ե��ʹ��
	//	Test_Electric_Hardware();//���Ե�ŵ�вɼ�
	//	Test_Encoder();//���Ա������ɼ�//
	/*********************************************************************************/

	while (1)
	{
		//	 Strategy_Slect();     //���뿪�ز���ѡ�� 1--��ʾ���ֵ������ƫ�� 2--ѡ�����ҳ��� 3--ѡ���Ƿ��뻷��
		//	 Handle_Barn_Out(Library_selection);     //ִ�г��⺯����flag_start��־λ��һ�Ż���ж�

		// 断点模块 调试使用
		if ((testflag == 7) || (testflag == 404))
		{
			DisableGlobalIRQ();
			go_motor(0, 0);
			switch (testflag)
			{
			case 7:
				testflag = 100;
				break;

			case 404:
				testflag = 101;
				break;
			}
		}
		//			ͣ����ʾ
		Reed();
		ips114_showint8(0, 6, Reed_Switch_Pin);
		ips114_showint8(0, 7, flag_end);

		//     Roundabout_debugshow();//����������Ļ��ʾ�����Ի���ʱ�򿪣�
		// Speed_debugshow();     //�ٶȻ�������Ļ��ʾ�������ٶȻ�ʱ�򿪣��ٶȻ���������λ�������Σ�
		//		  datasend();          //��λ����������

		//			�ٶ���ʾ
		ips114_showint16(50, 0, left_real_speed);
		ips114_showint16(50, 1, right_real_speed);
		ips114_showint16(50, 2, real_speed);
		ips114_showint16(50, 4, aim_speed);

		//          �����ʾ
		ips114_showint16(0, 0, Left_Adc); // road_type.straight
		ips114_showint16(0, 1, Left_Shu_Adc);
		ips114_showint16(0, 2, Right_Shu_Adc);
		ips114_showint16(0, 3, Right_Adc);
		ips114_showint16(0, 4, testflag);
		//			�����ʾ������ع�ͣ�
		ips114_showuint16(100, 5, dl1a_distance_mm);
		ips114_showuint16(100, 1, flag_obstacle);

		//			����ƫ����ʾ
		ips114_showfloat(0, 5, Current_Dir, 2, 1); // ��ʾ������   ������ʾ2λ   С����ʾ1λ

		//			��Դ��ѹ��ʾ
		ips114_showstr(140, 0, "Value:");
		ips114_showfloat(190, 0, adc_valueM, 2, 2);

		//			���ڵ���
		uart_init(UART_2, UART2_RX_P10, UART2_TX_P11, 115200, TIM_2); // ��ʼ������
		printf("%d,%d,%d\n", aim_speed, left_real_speed, right_real_speed);
	}
}
