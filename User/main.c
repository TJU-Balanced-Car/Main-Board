/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 USART Print debugging routine:
 USART1_Tx(PA9).
 This example demonstrates using USART1(PA9) as a print debug port output.

*/

#include "debug.h"
#include "Servo.h"
#include "Buzzer.h"
#include "Serial.h"


/* Global typedef */

/* Global define */

/* Global Variable */
uint8_t RxData;

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	//USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
	printf("Run Successfully!\r\n");

	Servo_PWM_Init();
	Servo_SetAngle(90);
	Buzzer_Init();
	Buzzer_Stop();
	Serial_Init();


	while(1)
    {
	    if (Serial_GetRxFlag() == 1)
	    {
	        RxData = Serial_GetRxData();
	        printf(RxData);
	    }
	}
}

