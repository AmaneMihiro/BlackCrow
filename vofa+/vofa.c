/*
 * @file		vofa.c/h
 * @brief	    vofa+发送数据（调参） --- 奇怪的BUG(vofa发送的内容不能全为0，不然会跳出DEBUG测试(主要原因可能是缓冲数组存储满了))
 * @history
 * 	版本			作者			编写日期			内容
 * 	v1.0		冯俊玮		2024/9/10		向上位机vofa+发送数据
 */
#include <stdio.h>
#include <stdarg.h>
#include "vofa.h"
#include "headfile.h"

#define MAX_BUFFER_SIZE 128

uint8 send_buf[MAX_BUFFER_SIZE];
uint16 cnt = 0;
int head = 0, tail = 0;
/**
***********************************************************************
* @brief:      vofa_transmit(uint8* buf, uint16 len)
* @param:		void
* @retval:     void
* @details:    修改通信工具，USART或者USB
***********************************************************************
**/
void vofa_transmit(uint8 *buf, uint16 len)
{
    uart_putbuff(UART_2, buf, len);
}
/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: 数据编号 data: 数据
* @retval:     void
* @details:    将浮点数据拆分成单字节
***********************************************************************
**/
void vofa_send_data(float adata)
{
    	send_buf[cnt++] = byte3(adata);
    	send_buf[cnt++] = byte2(adata);
    	send_buf[cnt++] = byte1(adata);
    	send_buf[cnt++] = byte0(adata);
}
/**
***********************************************************************
* @brief      vofa_sendframetail(void)
* @param      NULL
* @retval     void
* @details:   给数据包发送帧尾
***********************************************************************
**/
void vofa_sendframetail(void)
{
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x80;
    send_buf[cnt++] = 0x7f;

    /* 将数据和帧尾打包发送 */
    vofa_transmit((uint8 *)send_buf, cnt);
    cnt = 0; // 每次发送完帧尾都需要清零
}
/**
***********************************************************************
* @brief      vofa_demo(void)
* @param      NULL
* @retval     void
* @details:   demo示例
***********************************************************************
**/
void vofa_demo(float data1, float data2, float data3, float data4, float data5, float data6)
{

    // Call the function to store the data in the buffer
    vofa_send_data(data1);
    vofa_send_data(data2);
    vofa_send_data(data3);
    vofa_send_data(data4);
    vofa_send_data(data5);
    vofa_send_data(data6);
    // Call the function to send the frame tail
    vofa_sendframetail();
}

void vofa_demo3(float data1, float data2, float data3)
{

    // Call the function to store the data in the buffer
    vofa_send_data(data1);
    vofa_send_data(data2);
    vofa_send_data(data3);
    // Call the function to send the frame tail
    vofa_sendframetail();
}

void vofa_demo2(float data1, float data2)
{

    // Call the function to store the data in the buffer
    vofa_send_data(data1);
    vofa_send_data(data2);
    // Call the function to send the frame tail
    vofa_sendframetail();
}

void vofa_demo1(float data1)
{

    // Call the function to store the data in the buffer
    vofa_send_data(data1);
    // Call the function to send the frame tail
    vofa_sendframetail();
}
