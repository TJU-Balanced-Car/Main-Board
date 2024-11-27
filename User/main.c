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


/* Global typedef */

/* Global define */

/* Global Variable */
uint8_t RxData;
uint8_t ID;
volatile int32_t Motor1_is_there_speed = 1; // 标志位，指示速度是否为零
volatile int32_t Motor2_is_there_speed = 1; // 标志位，指示速度是否为零
volatile int32_t Motor1_lastCapture = 1; // 标志位，指示是否更新捕获
volatile int32_t Motor2_lastCapture = 1; // 标志位，指示是否更新捕获
volatile int Servo_Angle = 0;
float Vertical_Kp=315, Vertical_Ki=35, Vertical_Kd=-1.17;               //直立环KP、KD 675 52 -1.35
//float Vertical_Kp=425, Vertical_Ki=35, Vertical_Kd=-1.22;
float Velocity_Kp=-0, Velocity_Ki=-0.00;                               //速度环KP、KI-1.6 -0.007
//uint32_t TIM2_rpm = 0;
//uint8_t TIM2_direction = 0;
DataPacket packet = {
    .Roll = 1.23f,
    .Motor1Speed = 100,
    .Motor2Speed = 200,
    .ServoAngle = 45,
    .VerticalKp = 1.0f,
    .VerticalKi = 0.5f,
    .VerticalKd = 0.2f,
    .VerticalOut = 10,
    .VelocityKp = 1.5f,
    .VelocityKi = 0.7f,
    .VelocityOut = 20
};

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
//    while (USART1_Init());
    Serial_Init(115200);
    Motor_Encoder_Init();
    Encoder_Init();
    while (mpu_dmp_init()){printf("MPU Init Failed\n");};
    printf("MPU Init Succeed\n");
    Motor_Init();

	SystemCoreClockUpdate();
	Delay_Init();
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
	printf("Run Successfully!\r\n");


	Motor1_SetSpeed(0);
    Motor1_SetDir(1);
    Motor2_SetSpeed(10);
    Motor2_SetDir(1);

    Servo_SetAngle(Servo_Angle);
    Timer_IC_Init();
	while(1)
    {
        Delay_Ms(300);
        printf("R:%f,M1:%d,M2:%d,S:%d,Lp:%f,Li:%f,Ld:%f,Lo:%d,Yp:%f,Yi:%f,Yo:%d\n",
                packet.Roll, packet.Motor1Speed, packet.Motor2Speed, packet.ServoAngle,
                packet.VerticalKp, packet.VerticalKi, packet.VerticalKd, packet.VerticalOut,
                packet.VelocityKp, packet.VelocityKi, packet.VelocityOut);
//	    printf("duty1: %d, dir1: %d, duty2: %d, dir2: %d\n", Motor1_GetFreq(), Motor1_GetDir(), Motor2_GetFreq(), Motor2_GetDir());

//	    printf("ID:%d, AX:%d, AY:%d, AZ:%d, GX:%d, GY:%d, GZ:%d\n", ID,
//                MPU6050_Data.Accel_X_RAW, MPU6050_Data.Accel_Y_RAW, MPU6050_Data.Accel_Z_RAW,
//                MPU6050_Data.Gyro_X_RAW, MPU6050_Data.Gyro_Y_RAW, MPU6050_Data.Gyro_Z_RAW);

//	    if (Serial_GetRxFlag() == 1)
//	    {
//	        printf("RXXXXXXXXXXXXXXXXXXXXXXX\n");
//	        if(Serial_RxPacket[0] == 'p')
//	        {
//	            Vertical_Kp = (float)(*(Serial_RxPacket + 1));
//	            printf("Vertical_Kp:%f OK!\n", Vertical_Kp);
//	        }
//	        else if(Serial_RxPacket[0] == 'd')
//            {
//                Vertical_Kd = (float)(*(Serial_RxPacket + 1));
//                printf("Vertical_Kd:%f OK!\n", Vertical_Kd);
//            }
//	    }

	    //printf("clock:%f, Vertical_Kp:%f, Vertical_Kd:%f", clock(), Vertical_Kp, Vertical_Kd);
	}
}
