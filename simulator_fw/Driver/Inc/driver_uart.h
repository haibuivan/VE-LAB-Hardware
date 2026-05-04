#ifndef DRIVER_UART_H
#define DRIVER_UART_H

#include "stm32f10x.h"
#include <stdint.h>

void uart_init(USART_TypeDef *USARTx, uint32_t baudrate);
void uart_write(USART_TypeDef *USARTx, uint8_t data);
uint8_t uart_read(USART_TypeDef *USARTx);
void uart_stop(USART_TypeDef *USARTx);

#endif /* DRIVER_UART_H */
