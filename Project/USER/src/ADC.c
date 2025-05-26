#include "ADC.h"
#include "math.h"
// int16 aim_speeda        = 730;  //目标速度
int16 aim_speedb;       // 给定速度（动态给定速度）=基础速度*动态调整速度
int16 aim_speedc = 760; // 转弯最小速度
float errorh = 0;
float errors = 0;
float errors1 = 0;

uint8 Annulus_selection = 0; // 环岛选择标志

int16 adc_value[4]; // 所有电感采集值原始值    4个电感
int16 AD_V[4];      // 所有电感采集值归一化值中间量 （调试完成后可以删除）
// int16 adc_max[4]={90,90,90,95}; //电感采值最大值 需要自己采集
int16 adc_max[4] = {200, 200, 200, 200};                // 电感采值最大值 需要自己采集
int16 adc_min[4] = {1, 1, 1, 1};                        // 电感采值最小值
uint8 Left_Adc, Right_Adc, Left_Shu_Adc, Right_Shu_Adc; // 归一值
float adc_valueM;
int8 NM = 4; // 电感个数

// 环岛变量
uint16 annulus_s = 0;  // 环岛进入距离
uint16 annulus_s2 = 0; // 环岛进入距离2
uint16 annulus_s3 = 0;
uint16 annulus_z = 0; // 环岛进入角度
uint16 annulus_t = 0;

struct ROAD_TYPE road_type = {0};

/***当前位置*************/
float Current_Dir = 0;
int16 Set_gyro = 0;
float ADC_PWM = 0;
uint8 flag_obstacle = 0;
uint16 obstacle_time = 0;
uint8 temp = 0;
/***************************电感采集通道初始化****************************
函数名  void ADC_int(void)
功能：  电感采值进行初始化
参数：  void
说明：  电感采集初始化
返回值：无
************************************************************************/
void ADC_int(void)
{
    adc_init(Left_ADC_Pin, ADC_SYSclk_DIV_2);     // 初始化P0.0为ADC通道
    adc_init(LeftXie_ADC_Pin, ADC_SYSclk_DIV_2);  // 初始化P0.1为ADC通道
    adc_init(RightXie_ADC_Pin, ADC_SYSclk_DIV_2); // 初始化P0.5为ADC通道
    adc_init(Right_ADC_Pin, ADC_SYSclk_DIV_2);    // 初始化P0.6为ADC通道

    adc_init(Mid_ADC_Pin, ADC_SYSclk_DIV_2); // 初始化P1.5为ADC通道
}

/***************************中值滤波函数*********************************
函数名：uint16 adc_mid(ADCN_enum adcn,ADCRES_enum ch)
功能： 3次电感采值进行中值滤波
参数： adcn        选择ADC通道       resolution      分辨率
说明： 8位ADC测量值0~255，2的8次方，将5v电压平均分成255份，分辨率为5/255=0.196
返回值：k(uint8)中间那个值
************************************************************************/
uint16 adc_mid(ADCN_enum adcn, ADCRES_enum ch)
{
    uint16 i, j, k, tmp;
    i = adc_once(adcn, ch);
    j = adc_once(adcn, ch);
    k = adc_once(adcn, ch);
    if (i > j)
    {
        tmp = i, i = j, j = tmp;
    }
    if (k > j)
    {
        tmp = j;
    }
    else if (k > i)
    {
        tmp = k;
    }
    else
    {
        tmp = i;
    }
    return (tmp);
}

/***************************均值滤波函数****************************
函数名：  uint16 adc_ave(ADCN_enum adcn,ADCRES_enum ch,uint8 N)
功能：  均值滤波对第5次采集值求平均值
参数：  adcn        选择ADC通道
说明：  该函数调用中值滤波函数，无位移
返回值：tmp
示例：  adc_ave(ADC_P10, ADC_8BIT)-->ADC通道为P-10，分辨率为8bit
*******************************************************************/
uint16 adc_ave(ADCN_enum adcn, ADCRES_enum ch, uint8 N)
{
    uint32 tmp = 0;
    uint8 i;
    for (i = 0; i < N; i++)
    {
        tmp += adc_mid(adcn, ch);
    }
    tmp = tmp / N;
    return (tmp);
}
/***************************电感采值************************************
函数名：  void ADC_Collect()
功能：  电感采值
参数：  void
说明：  8位ADC测量值0~255，2的8次方，将5v电压平均分成255份，分辨率为5/255=0.196
返回值：void
***********************************************************************/
void ADC_Collect()
{
    adc_value[0] = adc_ave(Left_ADC_Pin, ADC_8BIT, 3);       // 左电感
    adc_value[1] = adc_ave(LeftXie_ADC_Pin, ADC_8BIT, 3);    // 左斜电感
    adc_value[2] = adc_ave(RightXie_ADC_Pin, ADC_8BIT, 3);   // 右斜电感
    adc_value[3] = adc_ave(Right_ADC_Pin, ADC_8BIT, 3);      // 右电感
    adc_valueM = adc_ave(Mid_ADC_Pin, ADC_8BIT, 3) * 0.2246; // 电源电压采集
}
/*********************************电感采值********************************
函数名：  void Data_current_analyze()
功能：  电感采值原始值归一化（0~100）
参数：  void
说明：  归一化处理
返回值：void
*************************************************************************/
void Data_current_analyze()
{
    uint8 i;
    for (i = 0; i < NM; i++)
    {
        AD_V[i] = ((adc_value[i] - adc_min[i]) * 100) / (adc_max[i] - adc_min[i]);
        if (AD_V[i] <= 0)
        {
            AD_V[i] = 0;
        }
        else if (AD_V[i] >= 100)
        {
            AD_V[i] = 100;
        }
    }
    Left_Adc = AD_V[0];      // 左电感值
    Left_Shu_Adc = AD_V[1];  // 左斜电感值
    Right_Shu_Adc = AD_V[2]; // 右斜电感值
    Right_Adc = AD_V[3];     // 右电感的值
}

/*********************************差和比函数**********************************
函数名：  float Cha_bi_he(int16 data1, int16 data2,int16 x)
功能：  差和比计算方向偏差
参数：  int16 data1, int16 data2,int16 x
说明：  差和比计算方向偏差
返回值：result
****************************************************************************/
float Cha_bi_he(int16 data1, int16 data2, int16 x)
{
    float cha;
    float he;
    float result;

    cha = (data1) - (data2);
    he = data1 + data2 + 1;
    result = (cha * x) / (1.0 * he);

    return result;
}
// 差和比差
float Cha_bi_he_cha(int16 data1, int16 data2, int16 data3, int16 data4, int16 x, int16 y)
{
    float cha;
    float he;
    float cha1;
    float he1;

    float result;

    cha = (data1) - (data2);
    cha1 = (data3) - (data4);

    he = data1 + data2 + 1;
    he1 = data3 + data4 + 1;

    //    result = (cha*x)/(1.0*he);
    result = ((cha * x) + (cha1 * y)) / ((1.0 * he) + (1.0 * he1));
    return result;
}
float Cha_x_bi_he(int16 data1, int16 data2, int16 data3, int16 data4) // 叉积差和比
{
    float left_value;
    float right_value;
    float ad_sum;
    float ad_diff;
    float error_x;
    left_value = sqrt(data1 * data1 + data2 * data2);

    right_value = sqrt(data3 * data3 + data4 * data4);

    ad_sum = left_value + right_value + 1; // 两电感之和

    // 两电感之差

    ad_diff = (int16)right_value - left_value;
    error_x = ad_diff / ad_sum;

    return error_x;
}
/*****************************************出轨保护函数*************************************
参数：  void Out_protect()
参数：  无
说明：  防止车子出轨撞墙等情况,如果四个电感都检测失败，则停转，关闭中断使能计时器

注意：调试程序平时测试时可以打开，跑了比赛场地则需要关闭此函数，否则有可能无法实现比赛功能，慎用
返回值：无
******************************************************************************************/
void Out_protect(void)
{
    if (Left_Adc < 10 && Right_Adc < 10)
    {
        go_motor(0, 0);
    }
}

/*************************************环岛辅助函数*************************************
参数：  void Annulus_assist(void)
参数：  无
说明：  更新环岛类型变量，环岛标志，环岛变量等

注意： 根据实际的差异来调整，需要自己调试推出去，看看显示屏显示情况记录去修改
返回值：无
******************************************************************************************/
void Annulus_assist(void)
{
    if (road_type.annulus == 1 && road_type.in_annulus_right == 0) //&&road_type.in_annulus_left==0
    {
        annulus_s += fabs(last_speed) * 1;
    }
    if (road_type.in_annulus_right == 1) // road_type.in_annulus_left==1 ||                 && road_type.on_annulus_left==0（右）&& road_type.on_annulus_right==0
    {
        annulus_z += fabs(GORY_Z);
        annulus_s2 += fabs(last_speed) * 1; // 根据环岛距离和标志变量值定的，0.1秒
    }
    if (road_type.on_annulus_right == 1 && road_type.in_annulus_right == 1) // road_type.in_annulus_left==1 ||                 && road_type.on_annulus_left==0（右）&& road_type.on_annulus_right==0
    {
        //        annulus_z += fabs(GORY_Z);
        annulus_s3 += fabs(last_speed) * 1; // 根据环岛距离和标志变量值定的，0.1秒
    }
    if (road_type.out_annulus == 1)
    {
        annulus_t = fabs(last_speed) * 1;
        //		      annulus_t=annulus_t+5;
    }
}

/*****************************************环岛处理***************************************
参数：  void Annulus_handle(void)
参数：  无
// 说明：  环岛处理函数

注意：环岛标志位的处理很重要
返回值：无
******************************************************************************************/
void Annulus_handle(void)
{
    if ((Left_Adc + Right_Adc) > IN_ANNULUS_H_LIMIT && road_type.annulus == 0) //&&Annulus_selection==0
    {
        road_type.annulus = 1;
        //			aim_speed        = 40;
        BUZZ_ON;
    }
    // 左环进入判断
    //		if(annulus_s > DISTANCE_ANNULUS_S&&road_type.annulus==1&&road_type.in_annulus_left==0&&(Left_Shu_Adc>20))
    //		{
    //			road_type.in_annulus_left = 1;
    //			BUZZ_ON;
    //			P52                      = 0;
    //		}
    // 右环进入判断
    if (annulus_s > DISTANCE_ANNULUS_S && road_type.in_annulus_right == 0 && road_type.annulus == 1) //&&(Right_Adc>20)
    {
        road_type.in_annulus_right = 1;
        BUZZ_ON;
        P52 = 0;
        //				while(1)//入环节点1
        //					{
        //						go_motor(0,0);
        //					}
    }
    // 左环处理
    //		if(road_type.in_annulus_left == 1 && annulus_z > DISTANCE_ANNULUS_Z && annulus_s2>350 &&road_type.on_annulus_left==0)
    //		{
    //
    //			  road_type.on_annulus_left = 1;
    //				BUZZ_ON;
    //			  P52                      = 1;
    //		}
    // �һ�����
    if (road_type.in_annulus_right == 1 && annulus_s2 > 1300) //
    {
        road_type.on_annulus_right = 1;
        BUZZ_ON;
        //			  P52                      = 1;
        //			annulus_s2=0;
        //				while(1)//上墙节点3
        //					{
        //						go_motor(0,0);
        //					}
    }
    if (road_type.on_annulus_right == 1 && annulus_s3 > 200 && (Left_Adc + Right_Adc) > OUT_ANNULUS_S_LIMIT) // +Right_Shu_Adc+Left_Shu_Adc
    {

        BUZZ_ON;
        road_type.out_annulus = 1;
        annulus_s = 0;
        annulus_z = 0;
        annulus_s2 = 0;
        //		annulus_s3                 = 0;
//		while(1)//出环节点5
        //					{
        //						go_motor(0,0);
        //					}
    }
    // 环岛变量
    if (annulus_s3 > DISTANCE_ANNULUS_T && road_type.out_annulus == 1 && road_type.annulus == 0)
    {
        road_type.annulus = 0;
        road_type.in_annulus_left = 0;
        road_type.in_annulus_right = 0;
        road_type.on_annulus_left = 0;
        road_type.on_annulus_right = 0;
        road_type.out_annulus = 0;
        annulus_t = 0;
        P52 = 1;
        BUZZ_OFF;
        //					while(1)//完成7
        //					{
        //						go_motor(0,0);
        //					}
    }
}
/*************************根据赛道类型选择不同的方向偏差计算方法*************************
函数名：  int16 Direction_error(void)
功能：  根据赛道类型选择不同的方向偏差
参数：  无
// 说明：  根据赛道类型选择不同的方向偏差
// 返回值：error--最终方向偏差
****************************************************************************************/
float Direction_error(void)
{
    float error = 0;

    // 环岛变量偏差计算
    if (road_type.annulus == 1)
    {
        if (road_type.in_annulus_right == 1 && road_type.on_annulus_right == 0 && road_type.out_annulus == 0)
        {
            error = 0.5;
        }
        // 在环岛上偏差
        if (road_type.on_annulus_right == 1)
        {
            road_type.annulus = 0; // 清除环岛标志位
            road_type.in_annulus_right = 0;
            error = (Cha_bi_he(Right_Adc, Left_Adc, 20));
        }
        // 环岛变量出偏差计算
        if (road_type.out_annulus == 1 && road_type.on_annulus_right == 1)
        {
            error = -3;
            road_type.annulus = 0; // 环岛标志位清零
        }
    }
    else
    {
        error = Cha_x_bi_he(Left_Adc, Left_Shu_Adc * 1, Right_Adc, Right_Shu_Adc * 1) * 5; // 屏幕显示方向偏差值
        errors = Cha_x_bi_he(Left_Adc, Left_Shu_Adc, Right_Adc, Right_Shu_Adc);             // 归一化的标准方向偏差值
    }
    return error;
}

/**********************************方向控制总处理***************************************
函数名  void Get_deviation(void)
功能：  方向控制总处理
参数：  无
// 说明：  在中断调用此函数处理
返回值：无
****************************************************************************************/
void Get_deviation(void)
{

    ADC_Collect();                   // 采集原始值赋值
    Data_current_analyze();          // 采集值归一化处理
    Annulus_handle();                // 环岛变量
    Annulus_assist();                // 环岛变量辅助
    Current_Dir = Direction_error(); // 获取方向偏差
}
