/*
 * RC.c
 *
 *  Created on: Mar 3, 2026
 *      Author: tanlo
 */
#include "RC.h"
void RC_CAPTURE_PWM(TIM_HandleTypeDef *htim, uint16_t*CHANNEL)
{
	if (htim->Instance == TIM1)
		{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			CHANNEL[ROLL] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
	}
	else if (htim->Instance == TIM3) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			CHANNEL[PITCH] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
	}
	else if (htim->Instance == TIM4)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			CHANNEL[THRUST] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
	}
}
