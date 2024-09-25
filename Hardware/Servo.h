/*
 * Servo.h
 *
 *  Created on: 2024��9��25��
 *      Author: jacob
 */

#ifndef HARDWARE_Servo_H_
#define HARDWARE_Servo_H_

void Servo_PWM_Init(void);
void Servo_PWM_SetCompare(uint16_t Compare);
void Servo_SetAngle(float Angle);

#endif /* HARDWARE_Servo_H_ */
