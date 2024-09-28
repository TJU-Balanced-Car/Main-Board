/*
 * Serial.h
 *
 *  Created on: 2024Äê9ÔÂ27ÈÕ
 *      Author: jacob
 */

#ifndef HARDWARE_SERIAL_H_
#define HARDWARE_SERIAL_H_

//#include <stdio.h>

void Serial_Init(void);
void USART1_SendByte(uint8_t Byte);
void USART1_SendArray(uint8_t *Array);
void USART1_SendString(char *String);
void USART1_SendNumber(uint32_t Number);
//void USART1_Printf(char *format, ...);
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

#endif /* HARDWARE_SERIAL_H_ */
