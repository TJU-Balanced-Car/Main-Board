/*
 * Serial.c
 *
 *  Created on: 2024��9��27��
 *      Author: jacob
 */

#include "debug.h"
#include <stdio.h>
#include <stdarg.h>

char Serial_RxPacket[100];
uint8_t Serial_RxFlag;

//==========================================================
//  �������ƣ�   Serial_Init
//  �������ܣ�   ��ʼ������
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Serial_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // ʹ��GPIOA��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // ʹ��AFIOʱ�ӣ���ʹ����ӳ��

    // ������ӳ�䣬��л�ߺ�΢�������������̣�https://www.wch.cn/bbs/thread-102615-1.html
    AFIO->PCFR1|=(1<<2);
    AFIO->PCFR2|=(1<<26);

    GPIO_InitTypeDef GPIO_InitStructure;
    // USART1_TX
    GPIO_StructInit(&GPIO_InitStructure); // �����������������TX���ʹ��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // USART1_RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // �������������RX����ʹ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 9600; // ������
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// Ӳ�������ƣ���ʹ������
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No; // ��У��λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // ֹͣλ
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // �ֳ�
    USART_Init(USART1, &USART_InitStructure);

    // �����ж�
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

//==========================================================
//  �������ƣ�   USART1_SendByte
//  �������ܣ�   ʹ��USART1�����ֽ�
//  ��ڲ�����   ���ֽڵ���Ϣ
//  ���ز�����   ��
//==========================================================
void USART1_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // ����־λ��ֹ��д
}

//==========================================================
//  �������ƣ�   USART1_SendArray
//  �������ܣ�   ʹ��USART1��������
//  ��ڲ�����   ��������
//  ���ز�����   ��
//==========================================================
void USART1_SendArray(uint8_t *Array)
{
    uint16_t i, Length;
    Length = sizeof(Array) / sizeof(Array[0]);
    for (i = 0; i < Length; i++)
    {
        USART1_SendByte(Array[i]);
    }
}

//==========================================================
//  �������ƣ�   USART1_SendString
//  �������ܣ�   ʹ��USART1�����ַ���
//  ��ڲ�����   �ַ���
//  ���ز�����   ��
//==========================================================
void USART1_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i ++)
    {
        USART1_SendByte(String[i]);
    }
}

//==========================================================
//  �������ƣ�   Pow
//  �������ܣ�   ����X��Y�η�
//  ��ڲ�����   ����X��ָ��Y
//  ���ز�����   X��Y�η����
//==========================================================
uint32_t Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y --) Result *= X;
    return Result;
}

//==========================================================
//  �������ƣ�   USART1_SendNumber
//  �������ܣ�   ʹ��USART1����һ������
//  ��ڲ�����   ��������
//  ���ز�����   ��
//==========================================================
void USART1_SendNumber(uint32_t Number)
{
    uint8_t i, Length = 0;
    uint32_t tmp = Number;
    while (tmp != 0)
    {
        tmp /= 10;
        Length++;
    }
    for (i = 0; i < Length; i++)
    {
        USART1_SendByte(Number / Pow(10, Length - i - 1) % 10 + '0');
    }
}

/****************************************************************************/
//==========================================================
//  �������ƣ�   fputc
//  �������ܣ�   �ض���printf����**��USART1����**
//  ��ڲ�����   Ҫ������ַ����ļ�ָ��
//  ���ز�����   ������ַ�
//==========================================================
int fputc(int ch, FILE *f)
{
    USART1_SendByte(ch);
    return ch;
}
/****************************************************************************/

/******************************************************************************
//==========================================================
//  �������ƣ�   USART1_Printf
//  �������ܣ�   �ض���printf����**��USART1����**
//  ��ڲ�����   Ҫ������ַ�����Ҫ�󳤶�С��500
//  ���ز�����   ��
//==========================================================
void USART1_Printf(char *format, ...)
{
    char String[501];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    USART1_SendString(String);
}
*******************************************************************************/

//==========================================================
//  �������ƣ�   Serial_GetRxFlag
//  �������ܣ�   ��ȡ���ڽ��ձ�־λ�����س�ȥ
//  ��ڲ�����   ��
//  ���ز�����   ���ڽ��ձ�־λ��1Ϊ���յ����ݣ�0Ϊδ���յ�����
//==========================================================
uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        return 1;
    }
    return 0;
}
