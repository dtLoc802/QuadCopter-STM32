/*
 * RC.h
 *
 *  Created on: Mar 3, 2026
 *      Author: tanlo
 */

#ifndef INC_RC_H_
#define INC_RC_H_
#include "stm32f4xx_hal.h"

enum RX_CHANNEL
{
	ROLL,
	PITCH,
	THRUST,
	YAW
};
void RC_CAPTURE_PWM(TIM_HandleTypeDef *htim, uint16_t*CHANNEL);
#endif /* INC_RC_H_ */
