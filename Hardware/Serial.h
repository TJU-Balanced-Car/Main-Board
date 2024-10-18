/*
 * Serial.h
 *
 *  Created on: 2024Äê9ÔÂ27ÈÕ
 *      Author: jacob
 */

#ifndef HARDWARE_SERIAL_H_
#define HARDWARE_SERIAL_H_

#include <stdio.h>

extern uint8_t Serial_TxPacket[];
extern uint8_t Serial_RxPacket[];

void Serial_Init(void);
void USART2_SendByte(uint8_t Byte);
void USART2_SendArray(uint8_t *Array);
void USART2_SendString(char *String);
void USART2_SendNumber(uint32_t Number);
//void USART2_Printf(char *format, ...);
void USART2_SendPacket(void);
uint8_t Serial_GetRxFlag(void);


#endif /* HARDWARE_SERIAL_H_ */
