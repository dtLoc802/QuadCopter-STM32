/*
 * Ringbuffer.h
 *
 *  Created on: Mar 5, 2026
 *      Author: tanlo
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include "stm32f4xx_hal.h"
#include "usart.h"

#define UART_BUFFER_SIZE 64

typedef struct
{
  unsigned char buffer[UART_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
} ring_buffer;

typedef struct
{
	UART_HandleTypeDef *huart;
	ring_buffer rx;
	ring_buffer tx;
	uint8_t rx_byte;
	uint8_t tx_byte;
} UART_RingBuffer;
extern uint8_t int_uart6_flag;
extern UART_RingBuffer HC05;
extern UART_RingBuffer MTF01P;

void RINGBUF_INIT(UART_RingBuffer *rb, UART_HandleTypeDef *huart);
void UART_WRITE(UART_RingBuffer *rb, int c);
int UART_READ(UART_RingBuffer *rb);
int AVAILABLEDATA(UART_RingBuffer *rb);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
#endif /* INC_RINGBUFFER_H_ */
