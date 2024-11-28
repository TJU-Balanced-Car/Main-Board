/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/03/06
* Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v30x_it.h"
#include "debug.h"

extern char Serial_RxPacket[100];
extern uint8_t Serial_RxFlag;
extern int32_t Motor1_is_there_speed; // ��־λ��ָʾ�ٶ��Ƿ�Ϊ��
extern int32_t Motor2_is_there_speed; // ��־λ��ָʾ�ٶ��Ƿ�Ϊ��
extern int32_t Motor1_lastCapture; // ��־λ��ָʾ�Ƿ���²���
extern int32_t Motor2_lastCapture; // ��־λ��ָʾ�Ƿ���²���
extern int Servo_Angle;

extern pid_param_t vel_pid;   // �ٶȻ�
extern pid_param_t angle_pid; // �ǶȻ�
extern pid_param_t acc_pid;   // ���ٶȻ�

extern DataPacket packet;

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI9_5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

//==========================================================
//  �������ƣ�   USART1_IRQHandler
//  �������ܣ�   �����жϷ�����
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void USART1_IRQHandler(void)
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;
    if (USART_GetFlagStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART1);

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

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

//==========================================================
//  �������ƣ�   TIM5_IRQHandler
//  �������ܣ�   ��ʱ���жϷ�������10ms��һ��
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

        // ���ͣ���ж�
        Motor1_is_there_speed = (Motor1_lastCapture == TIM_GetCapture2(TIM1)) ? 0 : 1;
        Motor2_is_there_speed = (Motor2_lastCapture == TIM_GetCapture2(TIM2)) ? 0 : 1;
        Motor1_lastCapture = (TIM_GetCapture2(TIM1) != 0) ? TIM_GetCapture2(TIM1) : Motor1_lastCapture;
        Motor2_lastCapture = (TIM_GetCapture2(TIM2) != 0) ? TIM_GetCapture2(TIM2) : Motor2_lastCapture;

        // ƽ��PID����
        PID_Control();
        packet.Motor1Speed = Motor1_GetFreq();
        packet.Motor2Speed = Motor2_GetFreq();
        packet.ServoAngle = Servo_Angle;

    }
}

// CCW2
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) == SET)
    {
        acc_pid.ki += 0.01;
//        angle_pid.ki += 5;
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
// CW2
void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) == SET)
    {
        acc_pid.ki -= 0.01;
//        angle_pid.ki -= 5;
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}
// PRESS2
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) == SET)
    {
//        TIM_Cmd(TIM1, DISABLE);
        acc_pid.kd += 0.1;
//        angle_pid.kd += 0.1;
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
// PRESS1
void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) == SET)
    {
//        TIM_Cmd(TIM1, ENABLE);
        acc_pid.kd -= 0.1;
//        angle_pid.kd -= 0.1;
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}
// CCW1
void EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) == SET)
    {
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) == 0)
        {
            acc_pid.kp -= 5;
//            angle_pid.kp -= 5;
        }
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}
// CW1
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line5) == SET)
    {
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 0)
        {
            acc_pid.kp += 5;
//            angle_pid.kp += 5;
        }
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}
