/*
 * Ringbuffer.c
 *
 *  Created on: Mar 5, 2026
 *      Author: tanlo
 */
#include "Ringbuffer.h"
UART_RingBuffer HC05;
UART_RingBuffer MTF01P;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UART_RingBuffer *rb = NULL;
	if (huart->Instance == USART6) rb = &HC05;
	else if (huart->Instance == USART1) rb = &MTF01P;
	else return;

    int i = (rb->rx.head + 1) % UART_BUFFER_SIZE;
    if (i != rb->rx.tail)
    {
    	rb->rx.buffer[rb->rx.head] = rb->rx_byte;
        rb->rx.head = i;
    }
    if (huart->Instance == USART6)
    {
    	if (rb->rx_byte == 'I' || rb->rx_byte == 'O' || rb->rx_byte == 'S' || rb->rx_byte == 'k' || rb->rx_byte == '+' || rb->rx_byte == '-' || rb->rx_byte == 'M')
        {
        	int_uart6_flag = 1;
        }
    }
    HAL_UART_Receive_IT(rb->huart, &rb->rx_byte, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	UART_RingBuffer *rb = NULL;
	if (huart->Instance == USART6) rb = &HC05;
	else if (huart->Instance == USART1) rb = &MTF01P;
	else return;
    if (rb->tx.head != rb->tx.tail)
    {
    	rb->tx_byte = rb->tx.buffer[rb->tx.tail];
    	rb->tx.tail = (rb->tx.tail + 1) % UART_BUFFER_SIZE;
        HAL_UART_Transmit_IT(rb->huart, &rb->tx_byte, 1);
    }
}
void RINGBUF_INIT(UART_RingBuffer *rb, UART_HandleTypeDef *huart)
{
  rb->huart = huart;
  rb->rx.head = 0;
  rb->rx.tail = 0;
  rb->tx.head = 0;
  rb->tx.tail = 0;
  rb->rx_byte = 0;
  rb->tx_byte = 0;
  HAL_UART_Receive_IT(rb->huart, &rb->rx_byte, 1);
  /* Enable the UART Data Register not empty Interrupt */
  //HAL_UART_Receive_IT(uart, &rx_byte, 1);
}

void UART_WRITE(UART_RingBuffer *rb, int c)
{
	if (c<0) return;

	int i = (rb->tx.head + 1) % UART_BUFFER_SIZE;
	while (i == rb->tx.tail);

	rb->tx.buffer[rb->tx.head] = (uint8_t)c;
	rb->tx.head = i;
	if (rb->huart->gState == HAL_UART_STATE_READY)
	{
	   	rb->tx_byte = rb->tx.buffer[rb->tx.tail];
	    rb->tx.tail = (rb->tx.tail + 1) % UART_BUFFER_SIZE;
	    HAL_UART_Transmit_IT(rb->huart, &rb->tx_byte, 1);
	} // Enable UART transmission interrupt
}

int UART_READ(UART_RingBuffer *rb)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if(rb->rx.head == rb->rx.tail) return -1;
    uint8_t c = rb->rx.buffer[rb->rx.tail];
    rb->rx.tail = (rb->rx.tail + 1) % UART_BUFFER_SIZE;
    return (int)c;
}
int AVAILABLEDATA(UART_RingBuffer *rb)
{
	 return (int)((UART_BUFFER_SIZE + rb->rx.head - rb->rx.tail) % UART_BUFFER_SIZE);
}
