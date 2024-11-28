#include "control.h" 
#include <debug.h>
#include "Encoder.h"
#include "Motor.h"
#include "MPU6050.h"

float Pitch,Roll,Yaw;						        //角度
short gyrox,gyroy,gyroz;	                        //角速度
int PWM1;                                           //最终输出

int16_t ECPULSE1;

extern pid_param_t vel_pid;   // 速度环
extern pid_param_t angle_pid; // 角度环
extern pid_param_t acc_pid;   // 角速度环


float Med_Angle=-0.4;	                                //机械中值---在这里修改你的机械中值即可。
extern DataPacket packet;

//==========================================================
//  函数名称：   PID_Control
//  函数功能：  PID控制函数
//  入口参数：   无
//  返回参数：   无
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
//  函数名称：   constrain_float
//  函数功能：   限幅函数
//  入口参数：   value：参数     min：最低值     max：最高值
//  返回参数：   无
//==========================================================
float constrain_float(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

//==========================================================
//  函数名称：   PidLocCtrl
//  函数功能：   pid位置式控制器输出
//  入口参数：   pid: pid参数                  error: pid输入误差
//  返回参数：   PID输出结果
//==========================================================
float PidLocCtrl(pid_param_t * pid, float error)
{
    // 累积误差（积分项）
    pid->integrator += error;
    // 积分限幅
    pid->integrator = constrain_float(pid->integrator, -pid->imax, pid->imax);
    // 计算PID各项
    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);
    pid->last_error = error;
    // 计算最终输出
    pid->out = pid->out_p + pid->out_i + pid->out_d;
    return pid->out;
}

//==========================================================
//  函数名称：   Cascade_Pid_Control
//  函数功能：   串级控制函数
//  入口参数：   机械中值
//  返回参数：   角速度环输出
//==========================================================
float Cascade_Pid_Control(float Med_Angle)
{
    static int16_t Pid_t;
    Pid_t += 2;
    mpu_dmp_get_data(&Pitch,&Roll,&Yaw);        //角度
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);    //陀螺仪角速度
    if (Pid_t % 20 == 0)                       // 角度环
    {
        Pid_t = 0;
//        PidLocCtrl(&angle_pid, Med_Angle - Roll);
    }
    if(Pid_t % 10 == 0)                         // 角速度环
    {
        PidLocCtrl(&acc_pid, angle_pid.out - gyroy);
        printf(gyroy);
    }
//    PidLocCtrl(&vel_pid, acc_pid.out - Motor1_GetFreq()); // 速度环

//    return vel_pid.out;
return acc_pid.out;
}
