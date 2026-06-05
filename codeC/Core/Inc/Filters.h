/*
 * Filters.h
 *
 *  Created on: Apr 2, 2026
 *      Author: Tanloc
 */
#include "stm32f4xx_hal.h"
#include "math.h"
#include "arm_math.h"
#include "ICM20948_I2C.h"
#include "MS5611_SPI.h"

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

#define RAD_TO_DEG 57.2957795130
#define DEG_TO_RAD 0.01745329252
#define Pi 3.1415926535f



typedef struct
{
	float omega;
	float alpha;
	float Q;
	float fc;
	float fs;
	float fce;
	float dt;
	float BiquadType;
	float a1, a2, b0, b1, b2;
	float x, y, w1, w2;
	float input;
	float output;
}BiquadFilter;
typedef struct
{
	float ConstantTime;
	float alpha;
	float fce;
	float fs;
	float dt;
	float output;
	float input;
}LowPassFilter;
typedef struct
{
	float beta, beta_default;
	float dt;
	float w[3];
	float a[3];
	float qdot[4];
	float q[4];
}MagdwitchFilter;
typedef struct
{
	float a, a1;
	float v, v1;
	float x, x1, xb;
	float ow, ov;
	float h;
	float dt, dx;
}ComplementaryFilterH;
typedef struct
{
	float height, velocity;
	float dt;
	float P00, P01, P10, P11;
	float K0, K1;
	float Differ;
	float R;
	float Q_height, Q_velocity, Q_heightSe, Q_velocitySe;
	float zn;
	float un;
	float L;
}KalmanFilter;
void LowpassFilterInit(LowPassFilter* filter, float fcenter, float dt);
float LowpassFilterProcess(LowPassFilter* filter, float input);
float LowpassFilterKd(LowPassFilter* filter, float input, float fcenter, float dt);

void BiquadFilterNotchInit(BiquadFilter* filter, float fcenter, float fcut, float dt);
void CalculateNotchQ(BiquadFilter* filter, float fcut, float fcenter);
void BiquadFilterInit(BiquadFilter* filter, float fcenter, float dt);
float BiquadFilterProcessTDF2(BiquadFilter* filter, float input);

void MadgwickFilter(MagdwitchFilter *filter, ICM20948_FILTER *IMU, float dt);
void MadgwickFilter_Update(MagdwitchFilter *filter, ICM20948_FILTER *IMU, AK09916_DATA *MAG, float dt);
//void BiquadFilterUpdate(BiquadFilter* filter, float fcenter, float dt, float Q);
//float BiquadFilterReset(BiquadFilter* filter, float value);
void ComplementaryHeightProcess(ComplementaryFilterH* filter, ICM20948_FILTER* imu, MS5611_SPI* bar, float dt);
void KalmanFilterProcess(KalmanFilter* K, ICM20948_FILTER* imu, MS5611_SPI* bar, float dt, uint8_t baro_enable);
#endif /* INC_FILTERS_H_ */
