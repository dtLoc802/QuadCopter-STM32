/*
 * HC05.c
 *
 *  Created on: Mar 5, 2026
 *      Author: tanlo
 */
#include "HC05.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern uint8_t TurnOnHeight;
extern uint8_t FirstARM;
extern uint8_t SendHeight;

uint8_t int_uart6_flag = 0;

static void HC05_Send(const char *msg)
{
    while (*msg)
    {
        UART_WRITE(&HC05, (int)(*msg++));
    }
}
void HC05_PID_SETTINGS(UART_HandleTypeDef *huart, char *rx_buffer, uint8_t *rxbyte, uint8_t rx_index,
					   PIDsingle* PitchAngle, PIDsingle* RollAngle, PIDsingle* YawAngle, PIDsingle* PitchRate,
					   PIDsingle* RollRate,PIDsingle* YawRate , PIDsingle* Height, PIDsingle* Velocity)
{
int len = strlen((char*)rx_buffer);

			// eliminate \r \n
			while (len > 0 && (rx_buffer[len-1] == '\r' || rx_buffer[len-1] == '\n')) {
			    rx_buffer[--len] = '\0';
			}
			char mode = rx_buffer[len - 1];   // 'O' 'I' 'S'
			// Khi gặp 'K' mới thì reset buffer (bắt đầu lệnh mới)
			if (*rxbyte == 'K' || *rxbyte == 'c') {
				rx_index = 0;
			}
			if (mode == 'O') {
				if (strncmp((char*) rx_buffer, "KpPitch:", 8) == 0) {
					float value = atof((char*) &rx_buffer[8]);
					PitchAngle->par.kp = value;

					char msg[40];
					sprintf(msg, "Angle_KpPitch = %f\r\n", PitchAngle->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KdPitch:", 8) == 0) {
					float value = atof((char*) &rx_buffer[8]);
					PitchAngle->par.kd = value;

					char msg[40];
					sprintf(msg, "Angle_KdPitch = %f\r\n", PitchAngle->par.kd);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KiPitch:", 8) == 0) {
					float value = atof((char*) &rx_buffer[8]);
					PitchAngle->par.ki = value;

					char msg[40];
					sprintf(msg, "Angle_KiPitch = %f\r\n", PitchAngle->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KpRoll:", 7) == 0) {
					float value = atof((char*) &rx_buffer[7]);
					RollAngle->par.kp = value;

					char msg[40];
					sprintf(msg, "Angle_KpRoll = %f\r\n", RollAngle->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KiRoll:", 7) == 0) {
					float value = atof((char*) &rx_buffer[7]);
					RollAngle->par.ki = value;

					char msg[40];
					sprintf(msg, "Angle_KiRoll = %f\r\n", RollAngle->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KdRoll:", 7) == 0) {
					float value = atof((char*) &rx_buffer[7]);
					RollAngle->par.kd = value;

					char msg[40];
					sprintf(msg, "Angle_KdRoll = %f\r\n", RollAngle->par.kd);
					HC05_Send(msg);
				}

				else if (strncmp((char*) rx_buffer, "KpYaw:", 6) == 0) {
					float value = atof((char*) &rx_buffer[6]);
					YawAngle->par.kp = value;

					char msg[40];
					sprintf(msg, "Angle_KpYaw = %f\r\n", YawAngle->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KiYaw:", 6) == 0) {
					float value = atof((char*) &rx_buffer[6]);
					YawAngle->par.ki = value;

					char msg[40];
					sprintf(msg, "Angle_KiYaw = %f\r\n", YawAngle->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KdYaw:", 6) == 0) {
					float value = atof((char*) &rx_buffer[6]);
					YawAngle->par.kd = value;

					char msg[40];
					sprintf(msg, "Angle_KdYaw = %f\r\n", YawAngle->par.kd);
					HC05_Send(msg);
				}

				else if (strncmp((char*) rx_buffer, "KpHeight:", 9) == 0) {
					float value = atof((char*) &rx_buffer[9]);
					Height->par.kp = value;

					char msg[40];
					sprintf(msg, "Height_Kp = %f\r\n", Height->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KiHeight:", 9) == 0) {
					float value = atof((char*) &rx_buffer[9]);
					Height->par.ki = value;

					char msg[40];
					sprintf(msg, "Height_Ki = %f\r\n", Height->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KdHeight:", 9) == 0) {
					float value = atof((char*) &rx_buffer[9]);
					Height->par.kd = value;

					char msg[40];
					sprintf(msg, "Height_Kd = %f\r\n", Height->par.kd);
					HC05_Send(msg);
				}

			}

			if (mode == 'I')
			{
				if (strncmp((char*) rx_buffer, "KpPitch:", 8) == 0)
				{
					float value = atof((char*) &rx_buffer[8]);
					PitchRate->par.kp = value;

					char msg[40];
					sprintf(msg, "Rate_KpPitch = %f\r\n", PitchRate->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KdPitch:", 8) == 0)
				{
					float value = atof((char*) &rx_buffer[8]);
					PitchRate->par.kd = value;

					char msg[40];
					sprintf(msg, "Rate_KdPitch = %f\r\n", PitchRate->par.kd);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KiPitch:", 8) == 0)
				{
					float value = atof((char*) &rx_buffer[8]);
					PitchRate->par.ki = value;

					char msg[40];
					sprintf(msg, "Rate_KiPitch = %f\r\n", PitchRate->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KpRoll:", 7) == 0)
				{
					float value = atof((char*) &rx_buffer[7]);
					RollRate->par.kp = value;

					char msg[40];
					sprintf(msg, "Rate_KpRoll = %f\r\n", RollRate->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KiRoll:", 7) == 0)
				{
					float value = atof((char*) &rx_buffer[7]);
					RollRate->par.ki = value;

					char msg[40];
					sprintf(msg, "Rate_KiRoll = %f\r\n", RollRate->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KdRoll:", 7) == 0)
				{
					float value = atof((char*) &rx_buffer[7]);
					RollRate->par.kd = value;

					char msg[40];
					sprintf(msg, "Rate_KdRoll = %f\r\n", RollRate->par.kd);
					HC05_Send(msg);
				}

				else if (strncmp((char*) rx_buffer, "KpYaw:", 6) == 0)
				{
					float value = atof((char*) &rx_buffer[6]);
					YawRate->par.kp = value;

					char msg[40];
					sprintf(msg, "Rate_KpYaw = %f\r\n", YawRate->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KiYaw:", 6) == 0)
				{
					float value = atof((char*) &rx_buffer[6]);
					YawRate->par.ki = value;

					char msg[40];
					sprintf(msg, "Rate_KiYaw = %f\r\n", YawRate->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KdYaw:", 6) == 0)
				{
					float value = atof((char*) &rx_buffer[6]);
					YawRate->par.kd = value;

					char msg[40];
					sprintf(msg, "Rate_KdYaw = %f\r\n", YawRate->par.kd);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KpVelocity:", 11) == 0)
				{
					float value = atof((char*) &rx_buffer[11]);
					Velocity->par.kp = value;

					char msg[40];
					sprintf(msg, "Velocity_Kp = %f\r\n", Velocity->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KiVelocity:", 11) == 0)
				{
					float value = atof((char*) &rx_buffer[11]);
					Velocity->par.ki = value;

					char msg[40];
					sprintf(msg, "Velocity_Ki = %f\r\n", Velocity->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "KdVelocity:", 11) == 0)
				{
					float value = atof((char*) &rx_buffer[11]);
					Velocity->par.kd = value;

					char msg[40];
					sprintf(msg, "Velocity_Kd = %f\r\n", Velocity->par.kd);
					HC05_Send(msg);
				}

			}
			if (mode == 'k')
			{
				char msg[500];
				sprintf(msg,
						"--ROLL--\r\n"
						"Kp = %.3f Ki = %.3f Kd = %.3f\r\n"
						"Kp = %.3f Ki = %.3f Kd = %.3f\r\n"
						"--PITCH--\r\n"
						"Kp = %.3f Ki = %.3f Kd = %.3f\r\n"
						"Kp = %.3f Ki = %.3f Kd = %.3f\r\n"
						"--YAW--\r\n"
						"Kp = %.3f Ki = %.3f Kd = %.3f\r\n"
						"Kp = %.3f Ki = %.3f Kd = %.3f\r\n"
						"--HEIGHT--\r\n"
						"Kp = %.3f Ki = %.3f Kd = %.3f\r\n"
						"Kp = %.3f Ki = %.3f Kd = %.3f\r\n",
						 RollAngle->par.kp, RollAngle->par.ki, RollAngle->par.kd,
						 RollRate->par.kp, RollRate->par.ki, RollRate->par.kd,
						 PitchAngle->par.kp, PitchAngle->par.ki, PitchAngle->par.kd,
						 PitchRate->par.kp, PitchRate->par.ki, PitchRate->par.kd,
						 YawAngle->par.kp, YawAngle->par.ki, YawAngle->par.kd,
						 YawRate->par.kp, YawRate->par.ki, YawRate->par.kd,
						 Height->par.kp,  Height->par.ki,  Height->par.kd,
						 Velocity->par.kp, Velocity->par.ki, Velocity->par.kd);
				HC05_Send(msg);
			}
			if (mode == '+')
			{
				//- Pitch Rate
				if (strncmp((char*) rx_buffer, "ppr", 3) == 0)
				{
					PitchRate->par.kp += 0.001;
					char msg[40];
					sprintf(msg, "Rate_KpPitch = %f\r\n", PitchRate->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "ipr", 3) == 0)
				{
					PitchRate->par.ki += 0.001;
					char msg[40];
					sprintf(msg, "Rate_KiPitch = %f\r\n", PitchRate->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "dpr", 3) == 0)
				{
					PitchRate->par.kd += 0.001;
					char msg[40];
					sprintf(msg, "Rate_KdPitch = %f\r\n", PitchRate->par.kd);
					HC05_Send(msg);
				}
				//- Roll Rate
				else if (strncmp((char*) rx_buffer, "prr", 3) == 0)
				{
					RollRate->par.kp += 0.001;
					char msg[40];
					sprintf(msg, "Rate_KpRoll = %f\r\n", RollRate->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "irr", 3) == 0)
				{
					RollRate->par.ki += 0.001;
					char msg[40];
					sprintf(msg, "Rate_KiRoll = %f\r\n", RollRate->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "drr", 3) == 0)
				{
					RollRate->par.kd += 0.001;
					char msg[40];
					sprintf(msg, "Rate_KdRoll = %f\r\n", RollRate->par.kd);
					HC05_Send(msg);
				}
				//- Pitch Angle
				else if (strncmp((char*) rx_buffer, "ppa", 3) == 0)
				{
					PitchAngle->par.kp += 0.001;
					char msg[40];
					sprintf(msg, "Angle_KpPitch = %f\r\n", PitchAngle->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "ipa", 3) == 0)
				{
					PitchAngle->par.ki += 0.001;
					char msg[40];
					sprintf(msg, "Angle_KiPitch = %f\r\n", PitchAngle->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "dpa", 3) == 0)
				{
					PitchAngle->par.kd += 0.001;
					char msg[40];
					sprintf(msg, "Angle_KdPitch = %f\r\n", PitchAngle->par.kd);
					HC05_Send(msg);
				}
				//- Roll Angle
				else if (strncmp((char*) rx_buffer, "pra", 3) == 0)
				{
					RollAngle->par.kp += 0.001;
					char msg[40];
					sprintf(msg, "Angle_KpRoll = %f\r\n", RollAngle->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "ira", 3) == 0)
				{
					RollAngle->par.ki += 0.001;
					char msg[40];
					sprintf(msg, "Angle_KiRoll = %f\r\n", RollAngle->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "dra", 3) == 0)
				{
					RollAngle->par.kd += 0.001;
					char msg[40];
					sprintf(msg, "Angle_KdRoll = %f\r\n", RollAngle->par.kd);
					HC05_Send(msg);
				}
				//- Yaw Angle
				else if (strncmp((char*) rx_buffer, "pya", 3) == 0)
				{
					YawAngle->par.kp += 0.001;
					char msg[40];
					sprintf(msg, "Angle_KpYaw = %f\r\n", YawAngle->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "iya", 3) == 0)
				{
					YawAngle->par.ki += 0.001;
					char msg[40];
					sprintf(msg, "Angle_KiYaw = %f\r\n", YawAngle->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "dya", 3) == 0)
				{
					YawAngle->par.kd += 0.001;
					char msg[40];
					sprintf(msg, "Angle_KdYaw = %f\r\n", YawAngle->par.kd);
					HC05_Send(msg);
				}
			}
			if (mode == '-')
			{
				//- Pitch Rate
				if (strncmp((char*) rx_buffer, "ppr", 3) == 0)
				{
					PitchRate->par.kp -= 0.001;
					char msg[40];
					sprintf(msg, "Rate_KpPitch = %f\r\n", PitchRate->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "ipr", 3) == 0)
				{
					PitchRate->par.ki -= 0.001;
					char msg[40];
					sprintf(msg, "Rate_KiPitch = %f\r\n", PitchRate->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "dpr", 3) == 0)
				{
					PitchRate->par.kd -= 0.001;
					char msg[40];
					sprintf(msg, "Rate_KdPitch = %f\r\n", PitchRate->par.kd);
					HC05_Send(msg);
				}
				//- Roll Rate
				else if (strncmp((char*) rx_buffer, "prr", 3) == 0)
				{
					RollRate->par.kp -= 0.001;
					char msg[40];
					sprintf(msg, "Rate_KpRoll = %f\r\n", RollRate->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "irr", 3) == 0)
				{
					RollRate->par.ki -= 0.001;
					char msg[40];
					sprintf(msg, "Rate_KiRoll = %f\r\n", RollRate->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "drr", 3) == 0)
				{
					RollRate->par.kd -= 0.001;
					char msg[40];
					sprintf(msg, "Rate_KdRoll = %f\r\n", RollRate->par.kd);
					HC05_Send(msg);
				}
				//- Pitch Angle
				else if (strncmp((char*) rx_buffer, "ppa", 3) == 0)
				{
					PitchAngle->par.kp -= 0.001;
					char msg[40];
					sprintf(msg, "Angle_KpPitch = %f\r\n", PitchAngle->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "ipa", 3) == 0)
				{
					PitchAngle->par.ki -= 0.001;
					char msg[40];
					sprintf(msg, "Angle_KiPitch = %f\r\n", PitchAngle->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "dpa", 3) == 0)
				{
					PitchAngle->par.kd -= 0.001;
					char msg[40];
					sprintf(msg, "Angle_KdPitch = %f\r\n", PitchAngle->par.kd);
					HC05_Send(msg);
				}
				//- Roll Angle
				else if (strncmp((char*) rx_buffer, "pra", 3) == 0)
				{
					RollAngle->par.kp -= 0.001;
					char msg[40];
					sprintf(msg, "Angle_KpRoll = %f\r\n", RollAngle->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "ira", 3) == 0)
				{
					RollAngle->par.ki -= 0.001;
					char msg[40];
					sprintf(msg, "Angle_KiRoll = %f\r\n", RollAngle->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "dra", 3) == 0)
				{
					RollAngle->par.kd -= 0.001;
					char msg[40];
					sprintf(msg, "Angle_KdRoll = %f\r\n", RollAngle->par.kd);
					HC05_Send(msg);
				}
				//- Yaw Angle
				else if (strncmp((char*) rx_buffer, "pya", 3) == 0)
				{
					YawAngle->par.kp -= 0.001;
					char msg[40];
					sprintf(msg, "Angle_KpYaw = %f\r\n", YawAngle->par.kp);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "iya", 3) == 0)
				{
					YawAngle->par.ki -= 0.001;
					char msg[40];
					sprintf(msg, "Angle_KiYaw = %f\r\n", YawAngle->par.ki);
					HC05_Send(msg);
				}
				else if (strncmp((char*) rx_buffer, "dya", 3) == 0)
				{
					YawAngle->par.kd -= 0.001;
					char msg[40];
					sprintf(msg, "Angle_KdYaw = %f\r\n", YawAngle->par.kd);
					HC05_Send(msg);
				}

			}
			if (mode == 'M')
			{
				TurnOnHeight++;
				char msg[40];
				if (TurnOnHeight == 1)
				{
					SendHeight = 1;
					sprintf(msg, "---------HEIGHT ON---------\r\n");
					HC05_Send(msg);
				}
				if (TurnOnHeight == 2)
				{
					TurnOnHeight = 0;
					SendHeight = 0;
					sprintf(msg, "---------HEIGHT OFF---------\r\n");
					HC05_Send(msg);
				}
			}
			memset(rx_buffer, 0, rx_index);
			rx_index = 0;
			int_uart6_flag = 0;
}
