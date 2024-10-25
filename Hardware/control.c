#include "control.h" 
#include <debug.h>
#include "Encoder.h"
#include "Motor.h"
#include "MPU6050.h"

float Pitch,Roll,Yaw;						        //�Ƕ�
short aacx,aacy,aacz;		                        //�Ǽ��ٶ�
short gyrox,gyroy,gyroz;	                        //���ٶ�

int Vertical_out,Velocity_out;                      //ֱ����&�ٶȻ����������
int PWM1;                                           //�������
int Encoder_Motor;	                                //����������������������ݣ��ٶȣ�


float Med_Angle=0.6;	                                //��е��ֵ---�������޸���Ļ�е��ֵ���ɡ�
extern float Vertical_Kp, Vertical_Kd, Velocity_Kp, Velocity_Ki;                 //�ٶȻ�KP��KI


void PID_Control(void)
{
    Encoder_Motor=Motor1_GetFreq();          //1.�ɼ�����������&MPU6050�Ƕ���Ϣ��

    mpu_dmp_get_data(&Pitch,&Roll,&Yaw);        //�Ƕ�
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);    //�����ǽ��ٶ�
    MPU_Get_Accelerometer(&aacx,&aacy,&aacz);   //�Ǽ��ٶ�

    Vertical_out=Vertical(Med_Angle,Roll,gyrox);//ֱ����
    Velocity_out=Velocity(Encoder_Motor);	    //�ٶȻ�

    //2.�ѿ�����������ص�����ϣ�������յĵĿ��ơ�
    PWM1=Vertical_out;
//            +Velocity_out;            //�������
    PWM_Limit(&PWM1);
    printf("Angle:%f, Vertical_out:%d, PWM1:%d\n", Roll, Vertical_out, PWM1);
    Motor1_SetSpeed(PWM1);
    Motor_Stop(&Med_Angle, &Roll);
}

/*********************
ֱ����PD��������Kp*Ek+Kd*Ek_D
��ڣ������Ƕȡ���ʵ�Ƕȡ���ʵ���ٶ�
���ڣ�ֱ�������
*********************/
int Vertical(float Med,float Angle,float gyro_x)
{
	int PWM_out1;
	PWM_out1=Vertical_Kp*(Med-Angle)+Vertical_Kd*(gyro_x);
	return PWM_out1;
}



/*********************
�ٶȻ�PI��Kp*Ek+Ki*Ek_S
*********************/
int Velocity (int Encoder_motor)
{
	static int Encoder_S,EnC_Err_Lowout_last,PWM_out2,Encoder_Err,EnC_Err_Lowout;
	float a=0.7;
	
	Encoder_Err=Encoder_motor;	                             //1.�����ٶ�ƫ��
    
	//2.���ٶ�ƫ����е�ͨ�˲�:low_out=(1-a)*Ek+a*low_out_last
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last; //ʹ�ò��θ���ƽ�����˳���Ƶ���ţ���ֹ�ٶ�ͻ�䡣
	EnC_Err_Lowout_last=EnC_Err_Lowout;                     //��ֹ�ٶȹ����Ӱ��ֱ����������������
    
	Encoder_S+=EnC_Err_Lowout;                                               //3.���ٶ�ƫ����֣����ֳ�λ��
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);//4.�����޷�
    
	PWM_out2=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;	            //5.�ٶȻ������������
	return PWM_out2;
}
