/*
 * Serial.c
 *
 *  Created on: 2024年9月27日
 *      Author: jacob
 */

#include "debug.h"
#include <stdio.h>
#include <stdarg.h>

char Serial_RxPacket[100];
uint8_t Serial_RxFlag;

//==========================================================
//  函数名称：   Serial_Init
//  函数功能：   初始化串口
//  入口参数：   无
//  返回参数：   无
//==========================================================
void Serial_Init(int BaudRate)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 使能AFIO时钟，以使用重映射
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); // 开启重映射

    // 开启重映射，感谢沁恒微电子社区的例程：https://www.wch.cn/bbs/thread-102615-1.html
//    AFIO->PCFR1|=(1<<2);
//    AFIO->PCFR2|=(1<<26);

    GPIO_InitTypeDef GPIO_InitStructure;
    // USART2_TX
    GPIO_StructInit(&GPIO_InitStructure); // 复用推挽输出，配置TX输出使用
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    // USART2_RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输出，配置RX接收使用
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = BaudRate; // 波特率
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;// 硬件流控制，不使用流控
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No; // 无校验位
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 停止位
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长
    USART_Init(USART2, &USART_InitStructure);

    // 开启中断
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
//  函数名称：   USART1_Init
//  函数功能：   串口初始化，包括蓝牙
//  入口参数：   无
//  返回参数：   成功为0
//==========================================================
uint8_t USART1_Init(void)
{
    Serial_Init(9600);
    Send_AT_Command("AT+BAUD8\r\n");
    Serial_Init(115200);
    return 0;
}

//==========================================================
//  函数名称：   USART2_SendByte
//  函数功能：   使用USART2发送字节
//  入口参数：   单字节的信息
//  返回参数：   无
//==========================================================
void USART2_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // 检测标志位防止覆写
}

//==========================================================
//  函数名称：   USART2_SendArray
//  函数功能：   使用USART2发送数组
//  入口参数：   整型数组
//  返回参数：   无
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
//  函数名称：   USART2_SendString
//  函数功能：   使用USART2发送字符串
//  入口参数：   字符串
//  返回参数：   无
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
//  函数名称：   Pow
//  函数功能：   计算X的Y次方
//  入口参数：   底数X，指数Y
//  返回参数：   X的Y次方结果
//==========================================================
uint32_t Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y --) Result *= X;
    return Result;
}

//==========================================================
//  函数名称：   USART2_SendNumber
//  函数功能：   使用USART2发送一串数字
//  入口参数：   整型数字
//  返回参数：   无
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
//  函数名称：   fputc
//  函数功能：   重定向printf函数**至USART2串口**
//  入口参数：   要输出的字符，文件指针
//  返回参数：   输出的字符
//==========================================================
int fputc(int ch, FILE *f)
{
    USART2_SendByte(ch);
    return ch;
}
/****************************************************************************/

/******************************************************************************
//==========================================================
//  函数名称：   USART2_Printf
//  函数功能：   重定向printf函数**至USART2串口**
//  入口参数：   要输出的字符串，要求长度小于500
//  返回参数：   无
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
//  函数名称：   Serial_GetRxFlag
//  函数功能：   获取串口接收标志位，返回出去
//  入口参数：   无
//  返回参数：   串口接收标志位，1为接收到数据，0为未接收到数据
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
//  函数名称：   USART_SendDataPacket
//  函数功能：   发送数据包
//  入口参数：   使用串口，数据包
//  返回参数：   无
//==========================================================
void USART_SendDataPacket(USART_TypeDef* USARTx, DataPacket* packet) {
    uint8_t* data = (uint8_t*)packet;  // 将数据包转换为字节流
    for (int i = 0; i < sizeof(DataPacket); ++i) {
        // 等待发送缓冲区为空
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
        // 发送一个字节
        USART_SendData(USARTx, data[i]);
    }
}

//==========================================================
//  函数名称：   Wait_BLE_Response
//  函数功能：   等待蓝牙响应AT指令
//  入口参数：   使用的串口
//  返回参数：   无
//==========================================================
void Wait_BLE_Response(USART_TypeDef* USARTx) {
    char buffer[100];
    int i = 0;
    while (1) {
        if (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) != RESET) {
            char ch = USART_ReceiveData(USARTx);
            buffer[i++] = ch;
            if (ch == '\n') {  // 如果蓝牙模块返回的结束符为 '\n'?
                buffer[i] = '\0';
                break;
            }
        }
    }
}

//==========================================================
//  函数名称：   Send_AT_Command
//  函数功能：   发送AT指令
//  入口参数：   指令内容
//  返回参数：   无
//==========================================================
void Send_AT_Command(const char *cmd) {
    while (*cmd) {
        USART_SendData(USART1, *cmd++);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}
