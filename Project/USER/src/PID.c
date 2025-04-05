#include "PID.h"
#include "math.h"
/************************************************
��������IncPIDInit(PID *sptr)
��  �ܣ�PID������ʼ��
��  ����
����ֵ��void
************************************************/
void IncPIDInit(PID *sptr)
{
    sptr->SumError = 0;
    sptr->LastError = 0;
    sptr->LLastError = 0;

    sptr->Kp = 0;
    sptr->Ki = 0;
    sptr->Kd = 0;
}

/************************************************
��������LocP_DCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
��  �ܣ�λ��ʽPID����
��  ����PID *sptr,int16 Setpoint,int16 Turepoint
����ֵ��float
************************************************/
int16 LocP_DCalc(PID *sptr, int16 Setpoint, int16 Turepoint)
{
    int16 iError, dError;
    int16 output;

    iError = Setpoint - Turepoint;                // ƫ��
    sptr->SumError += iError;                     // ����(����ʱ��ܶ�ʱ����һ�ײ�ִ���һ��΢�֣����ۼӴ������)
    dError = (int16)(iError - (sptr->LastError)); // ΢��
    sptr->LastError = iError;
    if (sptr->SumError > 2000)
        sptr->SumError = 2000; // �����޷�
    if (sptr->SumError < -2000)
        sptr->SumError = -2000;
    output = (int16)(sptr->Kp * iError             // ������
                     + (sptr->Ki * sptr->SumError) // ������
                     + sptr->Kd * dError);         // ΢����
    return (output);
}
/************************************************
��������IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
��  �ܣ�����ʽPID����
��  ����PID *sptr,int16 Setpoint,int16 Turepoint
����ֵ��int32 iIncpid
************************************************/
int16 IncPIDCalc(PID *sptr, int16 Setpoint, int16 Turepoint)
{
    int16 iError, iIncpid;
    // ��ǰ���
    iError = Setpoint - Turepoint; // ƫ��

    iIncpid = sptr->Kp * (iError - sptr->LastError) + sptr->Ki * iError;
    //    //�����������´μ���
    // if (iIncpid >= 2000) // ÿ����������޷�
    // {
    //     iIncpid = 2000;
    // }
    // if (iIncpid <= -2000)
    // {
    //     iIncpid = -2000;
    // }
    sptr->LastError = iError;
    return (iIncpid);
}
/************************************************
��������PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont)
��  �ܣ���̬λ��ʽPID���� (һ������ת�����)
��  ����PID *sptr,int16 Setpoint,int16 Turepoint
����ֵ��int32 Actual
************************************************/
int16 PlacePID_Control(PID *sptr, int16 Setpoint, int16 Turepiont)
{
    int16 iError, Actual;
    float KP; // ��̬P��ע����Kp����

    iError = Setpoint - Turepiont;
    KP = (iError)*sptr->Ki + sptr->Kp; // ��̬P�ļ���
    // sptr->SumError+=iError;

    Actual = KP * iError + sptr->Kd * (iError - sptr->LastError);

    sptr->LastError = iError;
    return Actual;
}
/************************************************
��������LocP_DCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
��  �ܣ�D��λ��ʽPID���ƣ�����������
��  ����PID *sptr,int16 Setpoint,int16 Turepoint
����ֵ��float
************************************************/
int16 PID_Turn_DT(PID *sptr, int16 Error, int16 Gory_z)
{
    int16 iError, dError, gory_z;
    float KP; // ��̬P��ע����Kp����
    int16 output;

    iError = Error; // ƫ��
    gory_z = Gory_z;

    // KP = (iError * iError) * sptr->Ki + sptr->Kp; // ��̬P�ļ��/�
    KP = sptr->Kp + fabs(iError) * sptr->Ki; // 线性增加而非二次方增加

    dError = (iError - (sptr->LastError)); // ΢��

    output = KP * iError              // ������(��̬p)
             + sptr->Kd * dError      // ΢����
             + sptr->K_gory * gory_z; // ����������

    sptr->LastError = iError;

    return (output);
}