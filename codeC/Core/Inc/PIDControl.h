/*
 * PIDControl.h
 *
 *  Created on: Oct 12, 2025
 *      Author: tanlo
 */

#ifndef __PID_CONTROL_H_
#define __PID_CONTROL_H_

#include "stm32f4xx_hal.h"
#include "Filters.h"
typedef struct _PIDparameter
{
	float kp;
	float ki;
	float kd;
	float Fce_kd;
	float dt;
	float kt;
}PIDparameter;
typedef struct _PIDSingle
{
	float reference;
	float measure_value;
	float measure_value_previous;
	float error;
	float error_derivation;
	float temp_i_result;
	float p_result;
	float i_result;
	float d_result;
	float pid_result;
	float pid_result_constrain;
	float backCalc;
	LowPassFilter LPF_Kd;
	PIDparameter par;
}PIDsingle;


#ifndef SIGN
#define SIGN(a) ((a >= 0) ? 1 : -1)
#endif


float PID_Caculate(PIDsingle* axis, float setpoint, float measure, float dt, float min, float max, uint8_t yaw);
void CalculatekT(PIDsingle* axis, float kp, float ki, float kd);
void PID_Init(PIDsingle* axis);
void PID_Reset(PIDsingle* axis);
#endif /*__PID_CONTROL_H */
