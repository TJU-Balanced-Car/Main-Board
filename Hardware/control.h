#ifndef __CONTROL_H 
#define __CONTROL_H 

void PID_Control(void);
int Vertical(float Med,float Angle,float gyro_x);
int Velocity(int encoder_motor);
#endif
