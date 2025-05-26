#include "debug.h"
#include "math.h"
/*************************ʹ��˵��****************************************
��Э���롰Visual Scope������Э����ݣ��ù��Ŀ���ֱ����ԭ������λ��Э�鼴��
�״�ʹ��ʱ��
1.����outputdata.c���͡�outputdata.h�����ӵ���Ĺ�����
2.�ڡ�outputdata.c���а�����ԭ����Ĵ��ڷ��ͺ���ͷ�ļ�
3.��uart_putchar(databuf[i]);����滻Ϊ��Ĵ����ֽڷ��ͺ�������send_char(databuf[i]);
4.����ĳ�����Ҫ���Ͳ������ݵ�.c�ļ������Ӱ�����#include "outputdata.h"�����ڱ��ļ��е��ú���OutPut_Data(x,y,z,w);
  �����β�x��y��z��w���Ǵ����ĸ�short int 16λ���ݣ��ֱ��Ӧͨ��1,2,3,4
************************************************************************/
// �˴�������Ĵ���ͷ�ļ�����������������������������
#include "zf_uart.h"
//****************************��ֲ**************************//

void Data_Send(UARTN_enum uratn, signed short int *pst)
{
    unsigned char _cnt = 0;
    unsigned char sum = 0;
    unsigned char data_to_send[23] = {0}; // ���ͻ���
    unsigned char i;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = (unsigned char)(pst[0] >> 8); // ��8λ
    data_to_send[_cnt++] = (unsigned char)pst[0];        // ��8λ
    data_to_send[_cnt++] = (unsigned char)(pst[1] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[1];
    data_to_send[_cnt++] = (unsigned char)(pst[2] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[2];
    data_to_send[_cnt++] = (unsigned char)(pst[3] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[3];
    data_to_send[_cnt++] = (unsigned char)(pst[4] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[4];
    data_to_send[_cnt++] = (unsigned char)(pst[5] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[5];
    data_to_send[_cnt++] = (unsigned char)(pst[6] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[6];
    data_to_send[_cnt++] = (unsigned char)(pst[7] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[7];
    data_to_send[_cnt++] = (unsigned char)(pst[8] >> 8);
    data_to_send[_cnt++] = (unsigned char)pst[8];

    data_to_send[3] = _cnt - 4;

    sum = 0;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    for (i = 0; i < _cnt; i++)
        uart_putchar(uratn, data_to_send[i]);
}

//===================================================��λ����ص�==========================================================
//======================================================================================================================
/****************���ݴ���******************
������void datasend()
������  ��
˵���� ����ͬʱ����6����  icm_gyro_x   icm_acc_x ICM_Real.gyro.y  ICM_Real.acc.z
�����ı�����icm_acc_x  icm_gyro_y  Angle  adc_date[0] Left_Adc

����ֵ����
���ڣ�
���ߣ�  */
void datasend()
{
    short send_data[6];

    send_data[0] = aim_speed;        // left_speed; ////////ICM_Start.acc.x
    send_data[1] = left_real_speed;  // right_speed; //////////////////    MpuStart.gyro.x   Angle
    send_data[2] = aim_speed;        //////////
    send_data[3] = right_real_speed; // imu660ra_gyro_z; //
    send_data[4] = 0;                // GORY_Z;
    send_data[5] = 0;
    // Data_Send(UART_4,send_data);
    Data_Send(UART_User, send_data);
}

float StrToDouble(const char *s)
{
    int i = 0;
    int k = 0;
    float j;
    int flag = 1;
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
        j = (s[i] - '0') * 1.0;
        result = result * 10 + j;
        i++;
    }
    if (s[i] == '.')
    {
        i++;
        while (s[i] != '\0' && s[i] != ' ')
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

void extern_iap_write_float(double dat, uint8 num, uint8 pointnum, uint16 addr)
{
    uint8 length;
    int8 buff[34];
    int8 start, end, point;

    if (0 > dat)
        length = zf_sprintf(&buff[0], "%f", dat); // ����
    else
    {
        length = zf_sprintf(&buff[1], "%f", dat);
        length++;
    }
    point = length - 7;         // ����С����λ��
    start = point - num - 1;    // ������ʼλ
    end = point + pointnum + 1; // �������λ
    while (0 > start)           // ����λ����  ĩβӦ�����ո�
    {
        buff[end] = ' ';
        end++;
        start++;
    }

    if (0 > dat)
        buff[start] = '-';
    else
        buff[start] = '+';

    buff[end] = '\0';

    extern_iap_write_bytes(addr, (uint8 *)buff, num + pointnum + 3);
}

float iap_read_float(uint8 len, uint16 addr)
{
    uint8 buf[34];
    iap_read_bytes(addr, buf, len);

    return StrToDouble(buf);
}

void EEROM_CanshuInit() // PID��ʼ��
{
    // ��eepromд�����ݣ���д���ʱ��ǵðѲ���eeprom������
    //	extern_iap_write_float(30,3,1,0x00);//(30,3,1,0x00)�е�30����Ҫд�������3��������λ��1����С��λ����仰����˼�ǰ�30ת��Ϊ"+030.0"����0x00-0x06��ַ�У���Ϊ"+030.0"ռ������ַ�������ַ����Ľ����ַ�'/0'������ռ���߸���ַ��������0x00��0x06,�¸�����Ҫ���Ե�0x07Ϊ��ʼ��ַ��
    //	extern_iap_write_float(15,3,1,0x07);
    //	extern_iap_write_float(11000,5,1,0x0e);
    //	extern_iap_write_float(315,4,1,0x17);
    //	extern_iap_write_float(2.0,1,1,0x1f);
    //	extern_iap_write_float(1.5,1,1,0x24);
    //	extern_iap_write_float(680,3,1,0x29);

    // ��eeprom�ж�ȡ���ݣ��ǵò�Ҫ�Ѳ���eeprom������
    // SpeedPID.Kp=iap_read_float(7,0x00);
    // iap_read_bytes(0x00,SpeedPID.Kp,2);
    // TurnPID.Ki=iap_read_float(7,0x0e);
    //	TurnPID.Kp= iap_read_float(7,0x00);//��仰����˼�Ǵ�0x00��ַ��ʼȡ��7����ַ�����ݣ�Ҳ����0x00��0x06��
    //  TurnPID.Ki = iap_read_float(7,0x07);
    //  TurnPID.Kd = iap_read_float(9,0x0e);
    // iap_read_bytes(0x00,(uint16 *)"350",2); //aim_speed=
    //  MotorPID.I = iap_read_float(8,0x17);
    //	Stright=iap_read_float(5,0x1f);
    //	Curve=iap_read_float(5,0x24);
    //	pout0=iap_read_float(7,0x29);
}
//====================================================��Ļ��ص�=(��ɾ��)=============================================================
//============================================================================================================================
// sprintf(temp," date20=%d",date);
// TFTSPI_P8X8Str(0,19,temp,BLACK,WHITE);break;

//==========================================================���뿪�ؼ��������=========================================================
//====================================================================================================================================

// ���뿪�����ź궨��
#define Switch_Pin_1 P75
#define Switch_Pin_2 P76
#define Switch_Pin_3 P11
#define Switch_Pin_4 P61
#define Switch_Pin_5 P14
#define Switch_Pin_6 P15
// ���尴������
#define KEY1 P70
#define KEY2 P71
#define KEY3 P72
#define KEY4 P73

//***************�����궨��****(������Щ�������޸ĺ궨��Ϊ��Ӧ��GPIO�⺯������)***********
#define KEY_INT(key_x) gpio_pull_set(key_x, PULLUP)          // ����Ϊ�������
#define SWITCH_INT(switch_x) gpio_pull_set(switch_x, PULLUP) // ����Ϊ��������
#define READ_GPIO(Pin_X) Pin_X
#define TiaoCan_DelayMs(M_S) delay_ms(M_S) // ��ʱ

unsigned char TiaoCan = 0;                                                                  ////////////////////////���α�־λ
unsigned char TFT_SHOW = 0;                                                                 ///////////////////////��Ļ����
unsigned char Switch1 = 0, Switch2 = 0, Switch3 = 0, Switch4 = 0, Switch5 = 0, Switch6 = 0; // ����
char parameter = 0;                                                                         // ����ѡ��

// ����״̬����
uint8 key1_status = 1, key2_status = 1, key3_status = 1, key4_status = 1, key5_status = 1;
// ��һ�ο���״̬����
uint8 key1_last_status, key2_last_status, key3_last_status, key4_last_status, key5_last_status;
// ���ر�־λ
uint8 key1_flag = 0, key2_flag = 0, key3_flag = 0, key4_flag = 0, key5_flag = 0;
/*****************���뿪�ؼ�������ʼ��*****************
������void Switch_Key_init()
���ܣ���ʼ��IO
������  ��
˵���� ��ʼ��IO��   gpio_init(D1, GPI, GPIO_HIGH, GPI_PULL_UP); GPO_PUSH_PULL
����ֵ����*/
void Switch_Key_init()
{

    // ���뿪�س�ʼ��  �������޸ģ������޸ģ�
    SWITCH_INT(Switch_Pin_1);
    SWITCH_INT(Switch_Pin_2);
    SWITCH_INT(Switch_Pin_3);
    SWITCH_INT(Switch_Pin_4);
    SWITCH_INT(Switch_Pin_5);
    SWITCH_INT(Switch_Pin_6);

    // ������ʼ�� �������޸ģ������޸ģ�
    KEY_INT(KEY1);
    KEY_INT(KEY2);
    KEY_INT(KEY3);
    KEY_INT(KEY4);
}

/*****************���뿪�ز���ѡ��*****************
������void Strategy_Slect()
���ܣ�ͨ�����뿪�ص�������
������  ��
˵����  6λ���뿪�أ���������ӻ��߼��ٿɶ����޸�,�������6��Ҳ��Ҫɾ������ģ��������������Ÿĸ�û�õļ���
        ʹ���㶨��ľͺ��ˣ�����û���õ����������
����ֵ����*/
void Strategy_Slect()
{
    // ��ȡ���뿪��״̬
    if (!READ_GPIO(Switch_Pin_1)) // ��
    {
        Switch1 = 1;
        Annulus_selection = 1;
        //			 //��ʾ���ԭʼֵ
        //			 ips114_showint16(0,0,adc_value[0]);
        //	     ips114_showint16(0,1,adc_value[1]);
        //	     ips114_showint16(0,2,adc_value[2]);
        //	     ips114_showint16(0,3,adc_value[3]);
        //
        //			 //��ʾ��й�һ��ֵ
        //			 ips114_showuint8(80,0,Left_Adc);
        //	     ips114_showuint8(80,1,Left_Shu_Adc);
        //	     ips114_showuint8(80,2,Right_Shu_Adc);
        //	     ips114_showuint8(80,3,Right_Adc);
        //
        //			 ips114_showfloat(0,5,Current_Dir,2,1);//��ʾ������   ������ʾ2λ   С����ʾ1λ
        //
        //			 ips114_showint16(130,0,left_speed);
        //  	   ips114_showint16(130,1,right_speed);
    }
    if (!READ_GPIO(Switch_Pin_2)) // ��
    {
        Switch2 = 1;
        Library_selection = 2; // ����ѡ��Library_selectionĬ��Ϊ1������⣬���º���2���ҳ���
    }
    if (!READ_GPIO(Switch_Pin_3))
    {
        Switch3 = 1;
    }
    if (!READ_GPIO(Switch_Pin_4))
    {
        Switch4 = 1;
    }
    if (!READ_GPIO(Switch_Pin_5))
    {
        Switch5 = 1;
    }
    if (!READ_GPIO(Switch_Pin_6))
    {
        Switch6 = 1;
    }
}

/*****************����ɨ���ȡ*****************
������void  Key_Scan_Deal ()
���ܣ���ȡ������ִ�ж�Ӧ����
������  ��
˵���� �ο�������� ��5λ��������������ӻ��߼��ٿɶ����޸�
      // 1��Ϊ���Ƽ���2��Ϊ�ϼ���3��Ϊ���Ƽ���4��Ϊ�м��̣�5��Ϊ�¼�
     //���γ���û��ʹ�õ��Σ�stc��Ƭ�����س���Ҳ�죬�����վͿ��ԣ����Ҫ�ӵĻ��Լ�����������Լ��ӾͿ���
����ֵ����     */
uint8 gogo = 0;
void Key_Scan_Deal()
{
    while (gogo <= 2)
    {
        // ʹ�ô˷����ŵ����ڣ�����Ҫʹ��while(1) �ȴ������⴦������Դ�˷�
        // ���水��״̬
        key1_last_status = key1_status;
        key2_last_status = key2_status;
        key3_last_status = key3_status;
        key4_last_status = key4_status;
        // ��ȡ��ǰ����״̬
        key1_status = READ_GPIO(KEY1);
        key2_status = READ_GPIO(KEY2);
        key3_status = READ_GPIO(KEY3);
        key4_status = READ_GPIO(KEY4);
        // ��⵽��������֮��  ���ſ���λ��־λ
        if (key1_status && !key1_last_status)
            key1_flag = 1;
        if (key2_status && !key2_last_status)
            key2_flag = 1;
        if (key3_status && !key3_last_status)
            key3_flag = 1;
        if (key4_status && !key4_last_status)
            key4_flag = 1;
        if (key5_status && !key5_last_status)
            key5_flag = 1;
        // ��־λ��λ֮�󣬿���ʹ�ñ�־λִ���Լ���Ҫ�����¼�

        if (key1_flag) // S1����������
        {
            key1_flag = 0; // ʹ�ð���֮��Ӧ�������־λ
            /*����Ϊ�û�����  */
            switch (parameter)
            {
                //-----------------------�������޸�����--��ע���޸Ķ�Ӧ����ʾ��----------------------------------------------------------------
                // ��һҳ��ʾ�Ķ���
            case 0:
                TurnPID.Kp += 1;
                break; // extern_iap_write_float(TurnPID.Kp,3,2,0x00);      break;
            case 1:
                TurnPID.Ki += 0.1;
                break; // extern_iap_write_float(TurnPID.Ki,3,2,0x07);			 break;
            case 2:
                TurnPID.Kd += 0.1;
                break; // extern_iap_write_float(TurnPID.Kd,3,2,0x0e);			 break;
            case 3:;
                TurnPID.K_gory += 1;
                break;
            case 4:;
                break;
            case 5:;
                break;
                /// case 6:  ; break;//������ܼ��κβ����������ˣ���ҳʹ���� extern_iap_write_bytes(0x00,&aim_speed,2);
                // �ڶ�ҳ��ʾ�Ķ���
            case 7:
                SpeedPID.Kp += 1;
                break;
            case 8:
                SpeedPID.Ki += 0.1;
                break;
            case 9:
                SpeedPID.Kd += 0.1;
                break;
            case 10:
                aim_speed += 10;
                break;
            case 11:
                break;
            case 12:
                break;
                //--------------------�������޸�����------------------------------------------------------------------
            }
        }
        if (key2_flag) // S2����������
        {
            key2_flag = 0; // ʹ�ð���֮��Ӧ�������־λ
            /*  ����Ϊ�û�����  */
            switch (parameter)
            {
                //----------------------�������޸�����--��ע���޸Ķ�Ӧ����ʾ��--------------------------------------------------------------
                // ��һҳ��ʾ�Ķ���
            case 0:
                TurnPID.Kp -= 20;
                break;
            case 1:
                TurnPID.Ki -= 5;
                break;
            case 2:
                TurnPID.Kd -= 5;
                break;
            case 3:;
                TurnPID.K_gory -= 5;
                break;
            case 4:;
                break;
            case 5:;
                break;
                // case 6:ips114_showstr(0,0,"FANYE_ing");  ; break;//������ܼ��κβ����������ˣ���ҳʹ����
                // �ڶ�ҳ��ʾ�Ķ���
            case 7:
                SpeedPID.Kp -= 1;
                break;
            case 8:
                SpeedPID.Ki -= 0.1;
                break;
            case 9:
                SpeedPID.Kd -= 0.1;
                break;
            case 10:
                aim_speed -= 50;
                break;
            case 11:
                break;
            case 12:
                break;
                //--------------------�������޸�����------------------------------------------------------------------
            }
        }
        if (key3_flag) // S3����ѡ����εĲ���
        {
            key3_flag = 0; // ʹ�ð���֮��Ӧ�������־λ
            parameter++;
            ips114_clear(WHITE);
            if (parameter >= 11)
                parameter = 0;
        }
        if (key4_flag) // S4����ȷ�ϰ������������˳�����
        {
            key4_flag = 0; // ʹ�ð���֮��Ӧ�������־λ
            gogo++;
        }
        //*******************************��Ļ��ʾ��һҳ***********************

        if (parameter < 6) // ��ʾ����0��5��ʵ����ʾ1��6
        {
            ips114_showstr(0, parameter + 1, "->");
            ips114_showstr(20, 0, "parameter->");
            ips114_showint8(150, 0, parameter); // xΪint8����
            // ��ʾ���ι���   �к����һ��Ϊ9   ֻ����ʾ��ô�࣬�������·�ҳ��ʾ  һҳ��6������
            ips114_showstr(20, 1, "TurnPID.Kp=");
            ips114_showfloat(150, 1, TurnPID.Kp, 3, 2); // ��ʾ������
            ips114_showstr(20, 2, "TurnPID.Ki=");
            ips114_showfloat(150, 2, TurnPID.Ki, 2, 2); // ��ʾ������
            ips114_showstr(20, 3, "TurnPID.Kd=");
            ips114_showfloat(150, 3, TurnPID.Kd, 2, 2); // ��ʾ������
            ips114_showstr(20, 4, "TurnPID.K_gory=");
            ips114_showint16(150, 4, TurnPID.K_gory);
        }
        //*******************************��Ļ��ʾ�ڶ�ҳ**************************************************
        if (parameter > 6 && parameter < 11) // �����кŴ�4��9   һҳ��6������  //��ʾ����7��5��ʵ����ʾ7��12
        {
            ips114_showstr(0, parameter - 6, "->");
            ips114_showstr(20, 0, "parameter->");
            ips114_showint8(150, 0, parameter); // xΪint8����
            // ��ʾ���ι���   �к����һ��Ϊ9   ֻ����ʾ��ô�࣬�������·�ҳ��ʾ  һҳ��6������
            ips114_showstr(20, 1, "SpeedPID.Kp=");
            ips114_showfloat(150, 1, SpeedPID.Kp, 2, 2); // ��ʾ������
            ips114_showstr(20, 2, "SpeedPID.Ki=");
            ips114_showfloat(150, 2, SpeedPID.Ki, 2, 2); // ��ʾ������
            ips114_showstr(20, 3, "SpeedPID.Kd=");
            ips114_showfloat(150, 3, SpeedPID.Kd, 2, 2); // ��ʾ������
            ips114_showstr(20, 4, "aim_speed=");
            ips114_showint16(150, 4, aim_speed);
            //		  ips114_showstr(20,5,"Turn_NeiPID.Ki=");ips114_showfloat(150,5,Turn_NeiPID.Ki,2,2);//��ʾ������
            //		  ips114_showstr(20,6,"Turn_NeiPID.Kd=");ips114_showfloat(150,6,Turn_NeiPID.Kd,2,2);//��ʾ������
        }
        //*******************************��Ļ��ʾ����ҳ**************************************************
        //    if(parameter>13&&parameter<20)
        //    {
        //
        //    }
        // ###########����Ҫ����ҳ������д�Ϳ��Կ�######################����Ͳ�д�� ����
        if (parameter == 6 || parameter == 13 || parameter == 20) // ��ҳ׼��
        {
            ips114_showstr(0, 0, "Ready to turn the page...");
            ips114_showstr(0, 1, "Press the button to display ");
            ips114_showstr(0, 2, "the second page...");
            // ips114_clear(WHITE);
        } // ����
        if (gogo > 2)
        {
            ips114_clear(WHITE);
        }
    }
}
//===============================================���Ի�����Ļ��ʾ=============================================
//============================================================================================================
/************************************************���Ի�����Ļ��ʾ*********************************************
������void Roundabout_debugshow(void)
���ܣ����Ի���ʱ��ʾ��Ӧ�ı�־λ���������
������  ��
˵����
����ֵ����
**************************************************************************************************************/
void Roundabout_debugshow(void)
{
    ips114_showuint8(0, 0, Left_Adc);          // �����ֵ
    ips114_showuint8(0, 1, Left_Shu_Adc);      // �������ֵ
    ips114_showuint8(0, 2, Right_Shu_Adc);     // �������ֵ
    ips114_showuint8(0, 3, Right_Adc);         // �Һ���ֵ
    ips114_showfloat(0, 5, Current_Dir, 2, 1); // ��ʾ������   ������ʾ2λ   С����ʾ1λ

    ips114_showuint8(50, 0, road_type.annulus);         // ������־λ
    ips114_showint16(50, 1, annulus_s);                 // ���������������������־���
    ips114_showuint8(50, 2, road_type.in_annulus_left); // �󻷵�������Ǳ�־λ
    ips114_showint16(50, 3, annulus_z);                 // ������������ǻ��ֽǶ�ֵ
    ips114_showint16(50, 4, annulus_s2);
    ips114_showuint8(50, 5, road_type.on_annulus_left); // ���󻷵���־λ����ΪС���Ѿ��뻷��
    ips114_showuint8(50, 6, road_type.out_annulus);     // ��������־λ
    ips114_showint16(50, 7, annulus_t);                 // ��������ʱ��0ʱ��

    ips114_showuint8(100, 0, road_type.annulus);          // ������־λ
    ips114_showint16(100, 1, annulus_s);                  // ���������������������־���
    ips114_showuint8(100, 2, road_type.in_annulus_right); // �󻷵�������Ǳ�־λ
    ips114_showint16(100, 3, annulus_z);                  // ������������ǻ��ֽǶ�ֵ
    ips114_showuint8(100, 4, road_type.on_annulus_right); // ���һ�����־λ����ΪС���Ѿ��뻷��
    ips114_showuint8(100, 5, road_type.out_annulus);      // ��������־λ
    ips114_showint16(100, 6, annulus_t);                  // ��������ʱ��0ʱ��
}

//===============================================�����ٶȻ�Ļ��ʾ=============================================
//============================================================================================================
/************************************************�����ٶȻ�Ļ��ʾ*********************************************
������Speed_debugshow(void)
���ܣ������ٶȻ�ʱ��ʾ��Ӧ���ұ�������ֵ��������ԣ��ٶȻ��Ƽ�����λ�������ε��ԣ�
������  ��
˵����
����ֵ����
**************************************************************************************************************/
void Speed_debugshow(void)
{
    ips114_showint16(0, 0, left_real_speed);  // ���������ֵ
    ips114_showint16(0, 1, right_real_speed); // �ұ�������ֵ
    ips114_showint16(0, 2, real_speed);       // ������ƽ��ֵ
}
//===============================================���������=============================================
//=====================================================================================================

// ���������͹� ��д�ڶ�Ӧͷ�ļ�ȥ�ˣ�ȥH�ļ��鿴

/*****************�������εε�*****************
������void BUZZ_DiDiDi()
���ܣ��������εε�
������  ��
˵����
����ֵ���� */
void BUZZ_DiDiDi(uint16 PinLV)
{
    BUZZ_ON;
    TiaoCan_DelayMs(PinLV);
    BUZZ_OFF;
}

/***************************�������**********************************************
 *  �������ƣ�Test_Servo(void)
 *  ����˵�������PWM��ʼ�������Ա궨���PWM����SD5/S3010���
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺
 *  ��    ע���ο�������ṩ�ģ�������
 ��ע�����ע�⣬һ��Ҫ�Զ����ǽ�������
 ʹ������ĸ��������̣�
 1.��ʹ�����ñ�������ص�ѹ����ر�֤��ص�ѹ��7V���ϣ�������������Ӧ��
 2.Ȼ��ȷ����������ѹ��SD5�����5V���磬S3010��6-7V���磬SD012�����5V���磡����
 3.�Ѷ���Ķ���ȥ�����ö����������ת����
 4.��д�������У��ö��ת������ֵ���������û��Ӧ�ظ�1-2�������ߵ��������PWMƵ�ʼ�ռ�ձȣ����ܿ�Ϊ׼��
 5.����ܿغ�������ת�������֨֨�죬�Կ�ת������ʱ����װ�϶��̣�
 6.����K0/K1ȷ�����������ת�����ޣ�������������Ϊ�����޷���ֹ�����ת�ջ٣�
 *************************************************************************/
void Test_Servo_Hardware(void)
{
    char txt[16] = "X:";
    unsigned int duty = Steer_Duty_Midle;

    ips114_clear(YELLOW); // ��ʼ����
    ips114_showstr(0, 0, "Test_Servo_Hardware:");
    pwm_init(Steer_Pin, 50, Steer_Duty_Midle); // ��ʼ�����  ���PWMƵ��200HZ����������ֵ
    pwm_duty(Steer_Pin, Steer_Duty_Midle);
    while (1)
    {
        if (!READ_GPIO(KEY1))
        {
            if (duty > 100) // ��ֹduty��
            {
                duty += 10; // �궨��ʱ�򣬿��԰Ѳ�����С�㣬����10
            }
        }
        if (!READ_GPIO(KEY3))
        {
            duty = Steer_Duty_Midle;
        }
        if (!READ_GPIO(KEY2))
        {
            duty -= 10;
        }
        pwm_duty(Steer_Pin, duty);
        sprintf(txt, "Servo:%05d ", duty);
        ips114_showstr(1, 2, txt); // ��ʾ
        TiaoCan_DelayMs(100);
    }
}

/****************************�������*********************************************
 *  �������ƣ�TestMotor(void)
 *  ����˵�������Ա궨���PWM���Ƶ��
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺
 *  ��    ע������2�����
 ��ע�����ע�⣬һ��Ҫ�Ե�������������
 ʹ������ĸ��������̣�
 1.��ʹ�����ñ�������ص�ѹ����ر�֤��ص�ѹ��7V���ϣ�������������Ӧ��
 2.�Ӻ�ĸ�嵽��������ź��߼���Դ�ߣ�
 3.�Ӻ������嵽����ĵ��ߣ�
 4.��д�������У�ȷ�����������ת���󣬿����������Դ���أ�
 5.����K0/K1ȷ�����ת���ٶȼ�����
 6.������ַ�ת������K1�����ص���ģʽ������ֱ�ӹر��������Դ��
 *************************************************************************/
void Test_Motor_Hardware(void)
{
    static int16 motor_duty = 3000;
    lcd_clear(YELLOW); // ��ʼ����
    lcd_showstr(2, 0, "Test_Motor_Hardware:");
    init_PWM(MOTOR_MODE_SELECT);

    while (1)
    {
        if (!READ_GPIO(KEY1)) // ����KEY1��   ���ֵ�����ת
        {
            go_motor(motor_duty, 0);
            lcd_showstr(0, 4, "Left  Front"); // �ַ�����ʾ
        }
        if (!READ_GPIO(KEY3)) // ����KEY2����������ͬʱ��ת
        {
            go_motor(-motor_duty, -motor_duty);
            lcd_showstr(0, 4, "All  Black"); // �ַ�����ʾ
        }
        if (!READ_GPIO(KEY2)) // ����KEY3��  ���ֵ�����ת
        {
            go_motor(0, motor_duty);
            lcd_showstr(0, 4, "Right Front"); // �ַ�����ʾ
        }
        if ((READ_GPIO(KEY1)) && (READ_GPIO(KEY2)) && (READ_GPIO(KEY3)))
            go_motor(0, 0);
        TiaoCan_DelayMs(100);
    }
}

/****************************�������*********************************************
 *  �������ƣ�void Test_Electric_Hardware (void)
 *  ����˵�������Ե�ŵ��Ӳ��
 *  ����˵������
 *  �������أ���
 *  ��    ע��
 ��ע�����
 *************************************************************************/
void Test_Electric_Hardware(void)
{
    char txt[16];
    ips114_clear(YELLOW); // ��ʼ����
    ips114_showstr(2, 0, "Test_Electric_Hardware:");
    ADC_int();
    while (1)
    {
        //		    if (!READ_GPIO(KEY1)) //����KEY1��
        //        {
        lcd_showstr(2, 1, "Normalize_Deal...."); // �ַ�����ʾ
        ADC_Collect();                           // ��в�ֵ

        sprintf(txt, "adc0= %05d", adc_value[0]);
        lcd_showstr(1, 2, txt); // ��ʾ
        sprintf(txt, "adc1= %05d", adc_value[1]);
        lcd_showstr(1, 3, txt); // ��ʾ
        sprintf(txt, "adc2= %05d", adc_value[2]);
        lcd_showstr(1, 4, txt); // ��ʾ
        sprintf(txt, "adc3= %05d", adc_value[3]);
        lcd_showstr(1, 5, txt); // ��ʾ
        //        }
        if (!READ_GPIO(KEY2)) // ����KEY2��
        {
            lcd_showstr(2, 1, "GYH_Normalize_Deal...."); // �ַ�����ʾ
            ADC_Collect();                               // ��в�ֵ
            Data_current_analyze();                      // ���ֵ��һ������
            Current_Dir = Cha_bi_he(Left_Adc, Right_Adc, 100);

            sprintf(txt, "adc0= %05d", Left_Adc);
            lcd_showstr(1, 2, txt); // ��ʾ
            sprintf(txt, "adc1= %05d", Left_Shu_Adc);
            lcd_showstr(1, 3, txt); // ��ʾ
            sprintf(txt, "adc2= %05d", Right_Shu_Adc);
            lcd_showstr(1, 4, txt); // ��ʾ
            sprintf(txt, "adc3= %05d", Right_Adc);
            lcd_showstr(1, 5, txt); // ��ʾ
            sprintf(txt, "Current_Dir= %05d", Current_Dir);
            lcd_showstr(1, 6, txt); // ��ʾ
        }
    }
}
/****************************�������*********************************************
 *  �������ƣ�void Test_Encoder(void)
 *  ����˵�������Ա�����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺
 *  ��    ע��
 ��ע�����
 *************************************************************************/
void Test_Encoder(void)
{
    char txt[16];
    encoder_init();    // ��������ʼ��
    lcd_clear(YELLOW); // ��ʼ����
    lcd_showstr(2, 0, "Test_Encoder:");
    while (1)
    {
        speed_measure();
        delay_ms(50);
        sprintf(txt, "Left_Speed  = %05d", left_speed);
        lcd_showstr(1, 3, txt); // ��ʾ
        sprintf(txt, "Right_Speed = %05d", right_speed);
        lcd_showstr(1, 4, txt); // ��ʾ
        sprintf(txt, "Real_Speed = %05d", real_speed);
        lcd_showstr(1, 5, txt); // ��ʾ
    }
}
