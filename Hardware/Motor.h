/*
 * Motor.h
 *
 *  Created on: 2024��10��11��
 *      Author: mdfucker
 */

#ifndef HARDWARE_MOTOR_H_
#define HARDWARE_MOTOR_H_

void Motor_PWM_Init(void);
void Motor_PWM_SetCompare(uint16_t Compare);
void Motor_SetAngle(float Angle);

#endif /* HARDWARE_MOTOR_H_ */
