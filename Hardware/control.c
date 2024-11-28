#include "control.h" 
#include <debug.h>
#include "Encoder.h"
#include "Motor.h"
#include "MPU6050.h"

float Pitch,Roll,Yaw;						        //�Ƕ�
short gyrox,gyroy,gyroz;	                        //���ٶ�
int PWM1;                                           //�������

int16_t ECPULSE1;

extern pid_param_t vel_pid;   // �ٶȻ�
extern pid_param_t angle_pid; // �ǶȻ�
extern pid_param_t acc_pid;   // ���ٶȻ�


float Med_Angle=-0.4;	                                //��е��ֵ---�������޸���Ļ�е��ֵ���ɡ�
extern DataPacket packet;

//==========================================================
//  �������ƣ�   PID_Control
//  �������ܣ�  PID���ƺ���
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void PID_Control(void)
{
    PWM1=Cascade_Pid_Control(Med_Angle);
    PWM_Limit(&PWM1);
    Motor1_SetSpeed(PWM1);
    Motor_Stop(&Med_Angle, &Roll);
    packet.Roll = Roll;
}

//==========================================================
//  �������ƣ�   constrain_float
//  �������ܣ�   �޷�����
//  ��ڲ�����   value������     min�����ֵ     max�����ֵ
//  ���ز�����   ��
//==========================================================
float constrain_float(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

//==========================================================
//  �������ƣ�   PidLocCtrl
//  �������ܣ�   pidλ��ʽ���������
//  ��ڲ�����   pid: pid����                  error: pid�������
//  ���ز�����   PID������
//==========================================================
float PidLocCtrl(pid_param_t * pid, float error)
{
    // �ۻ��������
    pid->integrator += error;
    // �����޷�
    pid->integrator = constrain_float(pid->integrator, -pid->imax, pid->imax);
    // ����PID����
    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);
    pid->last_error = error;
    // �����������
    pid->out = pid->out_p + pid->out_i + pid->out_d;
    return pid->out;
}

//==========================================================
//  �������ƣ�   Cascade_Pid_Control
//  �������ܣ�   �������ƺ���
//  ��ڲ�����   ��е��ֵ
//  ���ز�����   ���ٶȻ����
//==========================================================
float Cascade_Pid_Control(float Med_Angle)
{
    static int16_t Pid_t;
    Pid_t += 2;
    mpu_dmp_get_data(&Pitch,&Roll,&Yaw);        //�Ƕ�
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);    //�����ǽ��ٶ�
    if (Pid_t % 20 == 0)                       // �ǶȻ�
    {
        Pid_t = 0;
//        PidLocCtrl(&angle_pid, Med_Angle - Roll);
    }
    if(Pid_t % 10 == 0)                         // ���ٶȻ�
    {
        PidLocCtrl(&acc_pid, angle_pid.out - gyroy);
        printf(gyroy);
    }
//    PidLocCtrl(&vel_pid, acc_pid.out - Motor1_GetFreq()); // �ٶȻ�

//    return vel_pid.out;
return acc_pid.out;
}
