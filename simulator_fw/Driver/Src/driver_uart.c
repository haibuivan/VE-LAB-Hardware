#include "driver_uart.h"

void uart_init(USART_TypeDef *USARTx, uint32_t baudrate)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    if (USARTx == USART1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
        
        /* Configure USART1 Tx (PA9) as alternate function push-pull */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        /* Configure USART1 Rx (PA10) as input floating */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
    else if (USARTx == USART2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USARTx, &USART_InitStructure);
    USART_Cmd(USARTx, ENABLE);
}

void uart_write(USART_TypeDef *USARTx, uint8_t data)
{
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USARTx, data);
}

uint8_t uart_read(USART_TypeDef *USARTx)
{
    while (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET)
    {
    }
    return (uint8_t)(USART_ReceiveData(USARTx) & 0xFF);
}

void uart_stop(USART_TypeDef *USARTx)
{
    USART_Cmd(USARTx, DISABLE);
    
    if (USARTx == USART1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
    }
    else if (USARTx == USART2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);
    }
}
