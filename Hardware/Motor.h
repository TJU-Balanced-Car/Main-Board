/*
 * Motor.h
 *
 *  Created on: 2024��10��11��
 *      Author: mdfucker
 */

#ifndef HARDWARE_MOTOR_H_
#define HARDWARE_MOTOR_H_

void Motor_PWM_Init(void);
void Motor1_SetSpeed(float Angle);
void Motor2_SetSpeed(float Angle);
void Motor1_SetDir(uint8_t Dir);
void Motor2_SetDir(uint8_t Dir);

#endif /* HARDWARE_MOTOR_H_ */
