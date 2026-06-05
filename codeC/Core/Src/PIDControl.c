/*
 * PIDControl.c
 *
 *  Created on: Oct 12, 2025
 *      Author: tanloc
 */

#include "PIDControl.h"

float constrainf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}
void CalculatekT(PIDsingle* axis, float kp, float ki, float kd)
{
	if (ki > 1e-6f && kp > 1e-6f)
		{
			float Ti = kp / ki;
		    float Td = kd/ kp;
		    axis->par.kt = 2.0f / (Ti + Td);
		}
		else if (ki > 1e-6f)
		{
		    axis->par.kt = ki;
		    axis->par.kt = 0.0f;
		}
		else
		{
		    axis->par.ki = 0.0;
		    axis->par.kt = 0.0;
		}
}
void PID_Init(PIDsingle* axis)
{
	float kP = axis->par.kp;
	float kI = axis->par.ki;
	float kD = axis->par.kd;
	CalculatekT(axis, kP, kI, kD);
	PID_Reset(axis);
}
void PID_Reset(PIDsingle* axis)
{
	axis->reference = 0;
	axis->p_result = 0;
	axis->i_result = 0;
	axis->d_result = 0;
	axis->temp_i_result = 0;
	axis->measure_value = 0;
	axis->measure_value_previous = 0;
	axis->error = 0;
	axis->error_derivation = 0;
	axis->pid_result = 0;
	axis->pid_result_constrain = 0;
}
float PID_Caculate(PIDsingle* axis, float setpoint, float measure, float dt, float min, float max, uint8_t yaw)
 {
	 /**********Double PID Outer Begin (Roll and Pitch Position)**********/
	axis->par.dt = dt;
	 axis->reference = setpoint;
	 axis->measure_value = measure;

	 axis->error = axis->reference - axis->measure_value;
	 if (yaw)
	 {
		 if (axis->error < -180.f) axis->error += 360.f;
		 if (axis->error > 180) axis->error -= 360.f;
	 }
	 axis->p_result = axis->error * axis->par.kp;

	 axis->temp_i_result = axis->i_result + (axis->error * axis->par.ki + axis->backCalc * axis->par.kt) * axis->par.dt;
	 axis->i_result = constrainf(axis->temp_i_result, min, max);


	 axis->error_derivation = -(axis->measure_value - axis->measure_value_previous) / axis->par.dt;
	 axis->measure_value_previous = axis->measure_value;
	 if (axis->par.Fce_kd > 0)
	 axis->d_result = LowpassFilterKd(&axis->LPF_Kd, axis->error_derivation, axis->par.Fce_kd, axis->par.dt) * axis->par.kd;
	 else
	 axis->d_result = axis->error_derivation * axis->par.kd;


	 axis->pid_result = axis->p_result + axis->i_result + axis->d_result;
	 axis->pid_result_constrain = constrainf(axis->pid_result, min, max);
	 axis->backCalc = axis ->pid_result - axis ->pid_result_constrain;
	 if(SIGN(axis->backCalc) == SIGN(axis->i_result))
		 axis->backCalc = 0;
	 return axis->pid_result_constrain;
	 /*************************************************************/
}



