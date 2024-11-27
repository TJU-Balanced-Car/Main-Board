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

void PidInit(pid_param_t * pid)
{
  pid->kp        = 0;
  pid->ki        = 0;
  pid->kd        = 0;
  pid->imax      = 0;
  pid->out_p     = 0;
  pid->out_i     = 0;
  pid->out_d     = 0;
  pid->out       = 0;
  pid->integrator= 0;
  pid->last_error= 0;
  pid->last_derivative   = 0;
  pid->last_t    = 0;
}

void PID_Control(void)
{
    PWM1=Cascade_Pid_Control(Med_Angle);            //�������
    PWM_Limit(&PWM1);
//    printf("Angle:%f, Kp:%f, Kd:%f, Vertical_out:%d\n", Roll, Vertical_Kp, Vertical_Kd, Vertical_out);
//    printf("Speed:%d, Kp:%f, Ki:%f, Velocity_out:%d, PWM:%d\n", Encoder_Motor, Velocity_Kp, Velocity_Ki, Velocity_out, PWM1);
    Motor1_SetSpeed(PWM1);
    Motor_Stop(&Med_Angle, &Roll);
    packet.Roll = Roll;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�float constrain_float(float amt, float low, float high)
@����˵�����޷�����
@����˵����amt������ low�����ֵ high�����ֵ
@�������أ���
@�޸�ʱ�䣺2022/03/15
@���÷�����constrain_float(pid->integrator, -pid->imax, pid->imax);
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
float constrain_float(float amt, float low, float high)
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�float constrain_float(float amt, float low, float high)
@����˵����pidλ��ʽ���������
@����˵����pid     pid����  error   pid�������
@�������أ�PID������
@�޸�ʱ�䣺2022/03/15
@���÷�����
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
float PidLocCtrl(pid_param_t * pid, float error)
{
  /* �ۻ���� */
  pid->integrator += error;

  /* ����޷� */
  pid->integrator = constrain_float(pid->integrator, -pid->imax, pid->imax);


  pid->out_p = pid->kp * error;
  pid->out_i = pid->ki * pid->integrator;
  pid->out_d = pid->kd * (error - pid->last_error);

  pid->last_error = error;

  pid->out = pid->out_p + pid->out_i + pid->out_d;

  return pid->out;
}

float Cascade_Pid_Control(float Med_Angle)
{
    static int16_t Pid_t;
    Pid_t += 2;
    mpu_dmp_get_data(&Pitch,&Roll,&Yaw);        //�Ƕ�
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);    //�����ǽ��ٶ�
    if (Pid_t % 100 == 0)                       // �ٶȻ�
    {
        Pid_t = 0;
        ECPULSE1 = Motor1_GetFreq();
        PidLocCtrl(&vel_pid, 0-ECPULSE1);
    }
    if(Pid_t % 10 == 0)                         // �ǶȻ�
    {
        PidLocCtrl(&angle_pid, vel_pid.out - Pitch + Med_Angle);
    }
    PidLocCtrl(&acc_pid, -gyrox + angle_pid.out); // ���ٶȻ�

    return acc_pid.out;
}
