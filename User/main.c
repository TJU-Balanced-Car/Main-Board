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

#include <debug.h>
#include "Servo.h"
#include "Buzzer.h"
#include "Serial.h"
#include "MPU6050.h"
#include "TestLED.h"
#include "Encoder.h"
#include "Motor.h"


/* Global typedef */

/* Global define */

/* Global Variable */
uint8_t RxData;
uint8_t ID;
MPU6050_t MPU6050_Data;
volatile uint16_t CNT, Num;
volatile int32_t Motor1_is_there_speed = 1; // 标志位，指示速度是否为零
volatile int32_t Motor2_is_there_speed = 1; // 标志位，指示速度是否为零
volatile int32_t Motor1_lastCapture = 1; // 标志位，指示是否更新捕获
volatile int32_t Motor2_lastCapture = 1; // 标志位，指示是否更新捕获
//uint32_t TIM2_rpm = 0;
//uint8_t TIM2_direction = 0;

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    Servo_PWM_Init();
    Buzzer_Init();
    Buzzer_Stop();
    Test_LED_Init();
    Serial_Init();
    Encoder_Init();
    MPU6050_Init();
    Motor_Init();

	SystemCoreClockUpdate();
	Delay_Init();
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
	printf("Run Successfully!\r\n");


//	Motor1_SetSpeed(0);
//    Motor1_SetDir(1);
//    Motor2_SetSpeed(0);
//    Motor2_SetDir(1);

    Servo_SetAngle(90);
	while(1)
    {
//	    printf("duty1: %d, dir1: %d, duty2: %d, dir2: %d\n", Motor1_GetFreq(), Motor1_GetDir(), Motor2_GetFreq(), Motor2_GetDir());
	    Motor1_is_there_speed = (Motor1_lastCapture == TIM_GetCapture2(TIM1)) ? 0 : 1;
        Motor2_is_there_speed = (Motor2_lastCapture == TIM_GetCapture2(TIM2)) ? 0 : 1;
        Motor1_lastCapture = (TIM_GetCapture2(TIM1) != 0) ? TIM_GetCapture2(TIM1) : Motor1_lastCapture;
        Motor2_lastCapture = (TIM_GetCapture2(TIM2) != 0) ? TIM_GetCapture2(TIM2) : Motor2_lastCapture;

	    //        Motor1_SetSpeed(70);
//        Motor1_SetDir(1);
//	    CNT = TIM_GetCounter(TIM5);
//	    printf("CNT: %d", CNT);
//        Delay_Ms(10);
//        USART_SendData(USART2, '1');
//	    USART2_SendString("dsahgbksfjvk");
//	    Motor1_SetDir(1);
//	    Test_LED_Off();
//	    Delay_Ms(10);
//	    Motor1_SetDir(0);
//	    Test_LED_On();
//	    Delay_Ms(10);
//        Motor2_SetSpeed(70);
//        Motor2_SetDir(1);
	    //Test_LED_Off();

        MPU6050_Read_All(&MPU6050_Data);
	    ID = MPU6050_GetID();
//	    printf("ID:%d, AX:%d, AY:%d, AZ:%d, GX:%d, GY:%d, GZ:%d\n", ID,
//                MPU6050_Data.Accel_X_RAW, MPU6050_Data.Accel_Y_RAW, MPU6050_Data.Accel_Z_RAW,
//                MPU6050_Data.Gyro_X_RAW, MPU6050_Data.Gyro_Y_RAW, MPU6050_Data.Gyro_Z_RAW);
	    printf("KalmanAngleX:%f, KalmanAngleY:%f\n",
	            MPU6050_Data.KalmanAngleX, MPU6050_Data.KalmanAngleY);

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
