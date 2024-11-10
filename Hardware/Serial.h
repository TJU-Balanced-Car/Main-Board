/*
 * Serial.h
 *
 *  Created on: 2024Äê9ÔÂ27ÈÕ
 *      Author: jacob
 */

#ifndef HARDWARE_SERIAL_H_
#define HARDWARE_SERIAL_H_

#include <stdio.h>

typedef struct {
    float Roll;
    int Motor1Speed;
    int Motor2Speed;
    int ServoAngle;
    float VerticalKp;
    float VerticalKi;
    float VerticalKd;
    int VerticalOut;
    float VelocityKp;
    float VelocityKi;
    int VelocityOut;
} DataPacket;

extern char Serial_RxPacket[];

void Serial_Init(void);
void USART1_SendByte(uint8_t Byte);
void USART1_SendArray(uint8_t *Array);
void USART1_SendString(char *String);
void USART1_SendNumber(uint32_t Number);
//void USART1_Printf(char *format, ...);
uint8_t Serial_GetRxFlag(void);
void USART_SendDataPacket(USART_TypeDef* USARTx, DataPacket* packet);


#endif /* HARDWARE_SERIAL_H_ */
