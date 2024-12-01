#ifndef __CONTROL_H 
#define __CONTROL_H 

typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //�����޷�

    float                out_p;  //KP���
    float                out_i;  //KI���
    float                out_d;  //KD���
    float                out;    //pid���

    float                integrator; //< ����ֵ
    float                last_error; //< �ϴ����
    float                last_derivative;//< �ϴ���������ϴ����֮��
    unsigned long        last_t;     //< �ϴ�ʱ��
}pid_param_t;

void PID_Control(void);
float constrain_float(float amt, float low, float high);
float PidLocCtrl(pid_param_t * pid, float error);
float Cascade_Pid_Control(float Med_Angle);
#endif
