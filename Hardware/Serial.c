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
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // ʹ��GPIOA��ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // ʹ��AFIOʱ�ӣ���ʹ����ӳ��
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); // ������ӳ��

    GPIO_InitTypeDef GPIO_InitStructure;
    // USART2_TX
    GPIO_StructInit(&GPIO_InitStructure); // �����������������TX���ʹ��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    // USART2_RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // �������������RX����ʹ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 9600; // ������
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// Ӳ�������ƣ���ʹ������
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No; // ��У��λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // ֹͣλ
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // �ֳ�
    USART_Init(USART2, &USART_InitStructure);

    // �����ж�
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

//==========================================================
//  �������ƣ�   USART2_SendByte
//  �������ܣ�   ʹ��USART2�����ֽ�
//  ��ڲ�����   ���ֽڵ���Ϣ
//  ���ز�����   ��
//==========================================================
void USART2_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // ����־λ��ֹ��д
}

//==========================================================
//  �������ƣ�   USART2_SendArray
//  �������ܣ�   ʹ��USART2��������
//  ��ڲ�����   ��������
//  ���ز�����   ��
//==========================================================
void USART2_SendArray(uint8_t *Array)
{
    uint16_t i, Length;
    Length = sizeof(Array) / sizeof(Array[0]);
    for (i = 0; i < Length; i++)
    {
        USART2_SendByte(Array[i]);
    }
}

//==========================================================
//  �������ƣ�   USART2_SendString
//  �������ܣ�   ʹ��USART2�����ַ���
//  ��ڲ�����   �ַ���
//  ���ز�����   ��
//==========================================================
void USART2_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i ++)
    {
        USART2_SendByte(String[i]);
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
//  �������ƣ�   USART2_SendNumber
//  �������ܣ�   ʹ��USART2����һ������
//  ��ڲ�����   ��������
//  ���ز�����   ��
//==========================================================
void USART2_SendNumber(uint32_t Number)
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
        USART2_SendByte(Number / Pow(10, Length - i - 1) % 10 + '0');
    }
}

/****************************************************************************/
//==========================================================
//  �������ƣ�   fputc
//  �������ܣ�   �ض���printf����**��USART2����**
//  ��ڲ�����   Ҫ������ַ����ļ�ָ��
//  ���ز�����   ������ַ�
//==========================================================
int fputc(int ch, FILE *f)
{
    USART2_SendByte(ch);
    return ch;
}
/****************************************************************************/

/******************************************************************************
//==========================================================
//  �������ƣ�   USART2_Printf
//  �������ܣ�   �ض���printf����**��USART2����**
//  ��ڲ�����   Ҫ������ַ�����Ҫ�󳤶�С��500
//  ���ز�����   ��
//==========================================================
void USART2_Printf(char *format, ...)
{
    char String[501];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    USART2_SendString(String);
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

//==========================================================
//  �������ƣ�   USART2_IRQHandler
//  �������ܣ�   �����жϷ�����
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void USART2_IRQHandler(void)
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;
    if (USART_GetFlagStatus(USART2, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART2);

        if (RxState == 0)
        {
            if (RxData == '@')
            {
                RxData = 1;
                pRxPacket = 0;
            }
        }
        else if (RxState == 1)
        {
            if (RxData == '\r')
            {
                RxState = 2;
            }
            else
            {
                Serial_RxPacket[pRxPacket] = RxData;
                pRxPacket ++;
            }
        }
        else if (RxState == 2)
        {
            if (RxData == '\n')
            {
                RxState = 0;
                Serial_RxPacket[pRxPacket] = '\0';
                Serial_RxFlag = 1;

            }
        }

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
