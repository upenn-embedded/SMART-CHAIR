#ifndef UART_H
#define UART_H

#include <avr/io.h>
#include <stdbool.h>


extern void UART0_Init(void);

extern bool UART_isRxMsgReady(void);

extern void UART_ResetRxMsgReadyFlag(void);
extern void UART_ResetRxMsgIndex(void);
extern void UART0_SendString(const char *s);

extern void UART1_Init(void);

extern void UART1_SendChar(char c);

extern void UART1_SendString(const char *s);

#endif
