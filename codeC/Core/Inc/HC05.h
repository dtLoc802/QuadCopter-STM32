/*
 * HC05.h
 *
 *  Created on: Mar 5, 2026
 *      Author: tanlo
 */

#ifndef INC_HC05_H_
#define INC_HC05_H_

#include "stm32f4xx_hal.h"
#include "PIDControl.h"
#include "usart.h"
#include "Ringbuffer.h"

void HC05_PID_SETTINGS(UART_HandleTypeDef *huart, char *rx_buffer, uint8_t *rxbyte, uint8_t rx_index,
					   PIDsingle* PitchAngle, PIDsingle* RollAngle, PIDsingle* YawAngle, PIDsingle* PitchRate,
					   PIDsingle* RollRate,PIDsingle* YawRate , PIDsingle* Height, PIDsingle* Velocity);

#endif /* INC_HC05_H_ */
