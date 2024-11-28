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
extern short gyrox,gyroy,gyroz;
uint8_t RxData;
uint8_t ID;
volatile int32_t Motor1_is_there_speed = 1; // 标志位，指示速度是否为零
volatile int32_t Motor2_is_there_speed = 1; // 标志位，指示速度是否为零
volatile int32_t Motor1_lastCapture = 1; // 标志位，指示是否更新捕获
volatile int32_t Motor2_lastCapture = 1; // 标志位，指示是否更新捕获
volatile int Servo_Angle = 0;
//这里进行参数的初始化，结构体定义的数值按照顺序定义
//依次为  kp  ki  kd  积分限幅
pid_param_t vel_pid =   {2.3   ,  -0.00  ,  0     ,  0      , 0,0,0,0,0,0,0}; // 速度环
pid_param_t angle_pid = {-315.0  ,  -0.0    ,  0     ,  200    , 0,0,0,0,0,0,0}; // 角度环
pid_param_t acc_pid =   {0.13  ,  0.000  ,  0 ,  100.00 , 0,0,0,0,0,0,0}; // 角速度环
//uint32_t TIM2_rpm = 0;
//uint8_t TIM2_direction = 0;
DataPacket packet = {
    .Roll = 1.23f,
    .Motor1Speed = 100,
    .Motor2Speed = 200,
    .ServoAngle = 45,
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
        Delay_Ms(100);
        printf("R:%f,M1:%d,M2:%d,S:%d,"
                "Vp:%f,Vi:%f,Vd:%f,Vo:%f,"      // 速度环
                "Ap:%f,Ai:%f,Ad:%f,Ao:%f"       // 角度环
                "Cp:%f,Ci:%f,Cd:%f,Co:%f\n",    // 角速度环
                packet.Roll,  packet.Motor1Speed, packet.Motor2Speed, packet.ServoAngle,
                vel_pid.kp,   vel_pid.ki,         vel_pid.kd,         vel_pid.out,    // 速度环
                angle_pid.kp, angle_pid.ki,       angle_pid.kd,       angle_pid.out,  // 角度环
                acc_pid.kp,   acc_pid.ki,         acc_pid.kd,         acc_pid.out);   // 角速度环

        printf(gyroy);

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
