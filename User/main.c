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
#include "MPU6050.h"
#include "TestLED.h"


/* Global typedef */

/* Global define */

/* Global Variable */
uint8_t RxData;
uint8_t ID;
int16_t AX, AY, AZ, GX, GY, GZ;

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    //  Servo_PWM_Init();
    //  Servo_SetAngle(90);
      Buzzer_Init();
      Buzzer_Stop();
      Test_LED_Init();
//        Serial_Init();
    //  MPU6050_Init();

	SystemCoreClockUpdate();
	Delay_Init();
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
	printf("Run Successfully!\r\n");

//	Test_LED_On();

	while(1)
    {
//	    USART1_SendString("1");
	    Test_LED_On();
//	    Delay_Ms(1000);
	    //Test_LED_Off();
//	    MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
//	    ID = MPU6050_GetID();
//	    printf("ID:%d, AX:%d, AY:%d, AZ:%d, GX:%d, GY:%d, GZ:%d", ID, AX, AY, AZ, GX, GY, GZ);

//	    if (Serial_GetRxFlag() == 1)
//	    {
//	        RxData = Serial_GetRxData();
//	        printf(RxData);
//	    }
//	    if (Serial_GetRxFlag() == 1)
//	    {
//	        printf(Serial_RxPacket[0], Serial_RxPacket[1], Serial_RxPacket[2]);
//	    }
	}
}

