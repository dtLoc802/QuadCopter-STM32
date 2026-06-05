/*
 * Filters.c
 *
 *  Created on: Apr 2, 2026
 *      Author: Windows
 */
#include "Filters.h"
extern uint32_t MFcount;
extern uint32_t KFcount;

void LowpassFilterInit(LowPassFilter* filter, float fcenter, float dt)
{
	filter->dt = dt;
	filter->fce = fcenter;

	filter->ConstantTime = 1.0f / ( 2.0f * Pi * filter->fce );
	filter->alpha = filter->dt / ( filter->dt + filter->ConstantTime );
}
float LowpassFilterProcess(LowPassFilter* filter, float input)
{
	filter->input = input;
	filter->output = filter->output + filter->alpha*(filter->input - filter->output);
	return filter->output;
}
float LowpassFilterKd(LowPassFilter* filter, float input, float fcenter, float dt)
{
	filter->dt = dt;
	filter->fce = fcenter;

	filter->ConstantTime = 1.0f / ( 2.0f * Pi * filter->fce );
	filter->alpha = filter->dt / ( filter->dt + filter->ConstantTime );

	filter->input = input;
	filter->output = filter->output + filter->alpha*(filter->input - filter->output);
	return filter->output;
}
void BiquadFilterNotchInit(BiquadFilter* filter, float fcenter, float fcut, float dt)
{
	CalculateNotchQ(filter, fcut, fcenter);
	BiquadFilterInit(filter, fcenter, dt);
}
void CalculateNotchQ(BiquadFilter* filter, float fcut, float fcenter)
{
	filter->fce = fcenter;
	filter->fc = fcut;
	filter->Q = ( filter->fce * filter->fc ) / ( filter->fce * filter->fce - filter->fc * filter->fc );
}
void BiquadFilterInit(BiquadFilter* filter, float fcenter, float dt)
{
	filter->fce = fcenter;
	filter->dt = dt;
	if (filter->fce < (1000000 / filter->dt / 2.0f ))
	{
		filter->fs = 1.0f / ( (dt * 0.000001f ));

		filter->omega = 2.0f * Pi * filter->fce / filter->fs;
		filter->alpha = sinf(filter->omega )/ (2*filter->Q);

		float a0, a1, a2, b0, b1, b2;
		//NOTCH
		b0 = 1;
		b1 = -2 * cosf(filter->omega);
		b2 = 1;

		a0 = 1 + filter->alpha;
		a1 = -2 * cosf(filter->omega);
		a2 = 1 - filter->alpha;

		filter->b0 = b0/a0;
		filter->b1 = b1/a0;
		filter->b2 = b2/a0;
		filter->a1 = a1/a0;
		filter->a2 = a2/a0;
	}
	else
	{
		filter->b0 = 1.0f;
		filter->b1 = 0.0f;
		filter->b2 = 0.0f;
		filter->a1 = 0.0f;
		filter->a2 = 0.0f;
	}
}
float BiquadFilterProcessTDF2(BiquadFilter* filter, float input)
{
	filter->input = input;

	filter->output = filter->input * filter->b0 + filter->w1;
	filter->w1 = filter->input * filter->b1 -filter->a1 * filter->output + filter->w2;
	filter->w2 = filter->b2 * filter->input - filter->a2 * filter->output;
	return filter->output;
}
//- For catching the real-time Fcenter
//void BiquadFilterUpdate(BiquadFilter* filter, float fcenter, float dt, float Q)
//{
//
//    float w1 = filter->w1;
//    float w2 = filter->w2;
//
//    BiquadFilterInit(filter, fcenter, dt);
//
//
//    filter->w1 = w1;
//    filter->w2 = w2;
//
//}
//float BiquadFilterReset(BiquadFilter* filter, float value)
//{
//    filter->w1 = value - (value * filter->b0);
//    filter->w2 = (filter->b2 - filter->a2) * value;
//    return value;
//}
static float32_t divSqrt(float32_t i)
{
	float32_t sqrt;
	arm_sqrt_f32(i, &sqrt);
	return 1.0f/sqrt;
}
void MadgwickFilter_Update(MagdwitchFilter *filter, ICM20948_FILTER *IMU, AK09916_DATA *MAG, float dt)
{
	if (MFcount <= 9000)
	{
		filter->beta = filter->beta_default*50;
	}
	else
		filter->beta = filter->beta_default;
	float32_t DF1, DF2, DF3, DF4;
	float32_t qdot1, qdot2, qdot3, qdot4;
	float32_t Norm;
	float32_t hx, hy, hz;
	float32_t bx, bz;
	float32_t q1 = filter->q[0];
	float32_t q2 = filter->q[1];
	float32_t q3 = filter->q[2];
	float32_t q4 = filter->q[3];

	float32_t wx = IMU->gyro_filter[0] * DEG_TO_RAD;
	float32_t wy = IMU->gyro_filter[1] * DEG_TO_RAD;
	float32_t wz = -IMU->gyro_filter[2] * DEG_TO_RAD;

	float32_t ax = IMU->acc_filter[0];
	float32_t ay = IMU->acc_filter[1];
	float32_t az = IMU->acc_filter[2];

	float32_t mx = MAG->MAG_DATA[0];
	float32_t my = MAG->MAG_DATA[1];
	float32_t mz = MAG->MAG_DATA[2];
	filter->dt = dt;
	//- Turn to 6DOF algorithm if there isn't any magnetometer data
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		 MadgwickFilter(filter, IMU, dt);
		 return;
	}
	//- Calculate Quaternion derivative to present the angular rate
	qdot1 = 0.5f * (-q2*wx - q3*wy - q4*wz);
	qdot2 = 0.5f * (q1*wx + q3*wz - q4*wy);
	qdot3 = 0.5f * (q1*wy - q2*wz + q4*wx);
	qdot4 = 0.5f * (q1*wz + q2*wy - q3*wx);
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
		{

			//- Normalize accelerometer data
			Norm = divSqrt(ax*ax+ay*ay+az*az);
			ax *= Norm;
			ay *= Norm;
			az *= Norm;
			//- Normalize magnetometer data
			Norm = divSqrt(mx*mx+my*my+mz*mz);
			mx *= Norm;
			my *= Norm;
			mz *= Norm;
			//- Reference direction of Earth's magnetic field
			hx = mx*q1*q1 + mx*q2*q2 - mx*q3*q3 - mx*q4*q4 - 2.0f*my*q1*q4 + 2.0f*my*q2*q3 + 2.0f*mz*q1*q3 + 2.0f*mz*q2*q4;
			hy = my*q1*q1 - my*q2*q2 + my*q3*q3 - my*q4*q4 + 2.0f*mx*q1*q4 + 2.0f*mx*q2*q3 - 2.0f*mz*q1*q2 + 2.0f*mz*q3*q4;
			hz = mz*q1*q1 - mz*q2*q2 - mz*q3*q3 + mz*q4*q4 - 2.0f*mx*q1*q3 + 2.0f*mx*q2*q4 + 2.0f*my*q1*q2 + 2.0f*my*q3*q4;

			bx = sqrt(hx*hx + hy*hy);
			bz = hz;
			//Gradient decent algorithm corrective step
			DF1 = ax*q3*2.0f-ay*q2*2.0f+q1*(q2*q2)*4.0f+q1*(q3*q3)*4.0f+(bx*bx)*q1*(q3*q3)*4.0f+(bx*bx)*q1*(q4*q4)*4.0f+(bz*bz)*q1*(q2*q2)*4.0f+(bz*bz)*q1*(q3*q3)*4.0f+bx*my*q4*2.0f-bx*mz*q3*2.0f+bz*mx*q3*2.0f-bz*my*q2*2.0f-bx*bz*q1*q2*q4*8.0f;
			DF2 = ax*q4*-2.0f-ay*q1*2.0f+az*q2*4.0f+q2*(q3*q3)*4.0f+(q2*q2*q2)*4.0f+(bz*bz)*(q2*q2*q2)*4.0f+(bx*bx)*q2*(q3*q3)*4.0f+(bx*bx)*q2*(q4*q4)*4.0f+(bz*bz)*q2*(q3*q3)*4.0f-bx*my*q3*2.0f-bx*mz*q4*2.0f-bz*mx*q4*2.0f-bz*my*q1*2.0f+bz*mz*q2*4.0f-bx*bz*(q2*q2)*q4*8.0f;
			DF3 = ax*q1*2.0f-ay*q4*2.0f+az*q3*4.0f+(q2*q2)*q3*4.0f+(q3*q3*q3)*4.0f+(bx*bx)*(q3*q3*q3)*4.0f+(bz*bz)*(q3*q3*q3)*4.0f+(bx*bx)*q3*(q4*q4)*4.0f+(bz*bz)*(q2*q2)*q3*4.0f+bx*mx*q3*4.0f-bx*my*q2*2.0f-bx*mz*q1*2.0f+bz*mx*q1*2.0f-bz*my*q4*2.0f+bz*mz*q3*4.0f-bx*bz*q2*q3*q4*8.0f;
			DF4 = ax*q2*-2.0f-ay*q3*2.0f+(q2*q2)*q4*4.0f+(q3*q3)*q4*4.0f+(bx*bx)*(q4*q4*q4)*4.0f+(bx*bx)*(q3*q3)*q4*4.0f+(bz*bz)*(q2*q2)*q4*4.0f+(bz*bz)*(q3*q3)*q4*4.0f+bx*mx*q4*4.0f+bx*my*q1*2.0f-bx*mz*q2*2.0f-bz*mx*q2*2.0f-bz*my*q3*2.0f-bx*bz*q2*(q4*q4)*8.0f;
			//- Normalize DeltaF data
			Norm = divSqrt(DF1*DF1 + DF2*DF2 +DF3*DF3 + DF4*DF4);
			DF1 *= Norm;
			DF2 *= Norm;
			DF3 *= Norm;
			DF4 *= Norm;
			//- Calculate Quaternion dot
			qdot1 -= filter->beta*DF1;
			qdot2 -= filter->beta*DF2;
			qdot3 -= filter->beta*DF3;
			qdot4 -= filter->beta*DF4;
		}
		//- Integral
		q1 += qdot1*filter->dt;
		q2 += qdot2*filter->dt;
		q3 += qdot3*filter->dt;
		q4 += qdot4*filter->dt;
		//- Normalize Quaternion
		Norm = divSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
		q1 *= Norm;
		q2 *= Norm;
		q3 *= Norm;
		q4 *= Norm;

		filter->q[0] = q1;
		filter->q[1] = q2;
		filter->q[2] = q3;
		filter->q[3] = q4;

		//- Quaternion to Euler Angle
		IMU->angle_rad[0] = atan2f(2.0f*(q1*q2 + q3*q4), q1*q1 - q2*q2 - q3*q3 + q4*q4);
		IMU->angle_rad[1] = asinf(-2.0f*(q2*q4 - q1*q3));
		IMU->angle_rad[2] = atan2f(2.0f*(q2*q3 + q1*q4),q1*q1 + q2*q2 - q3*q3 - q4*q4);
		//- Radiant to Angle
		for (int i = 0; i < 3; i++)
		{
			IMU->angle_deg[i] = IMU->angle_rad[i] * RAD_TO_DEG;
		}
		//-Remove gravity
		IMU->acc_filter_linear[0] =	IMU->acc_filter[0]*(1.0f - 2.0f*q3*q3 - 2.0f*q4*q4) + IMU->acc_filter[1]*(2.0f*q2*q3 + 2.0f*q1*q4) + IMU->acc_filter[2]*(2.0f*q2*q4 - 2.0f*q1*q3);
		IMU->acc_filter_linear[1] = IMU->acc_filter[0]*(2.0f*q2*q3 - 2.0f*q1*q4) + IMU->acc_filter[1]*(1.0f - 2.0f*q2*q2 - 2.0f*q4*q4) + IMU->acc_filter[2]*(2.0f*q1*q2 + 2.0f*q3*q4);
		IMU->acc_filter_linear[2] = IMU->acc_filter[0]*(2.0f*q1*q3 + 2.0f*q2*q4) + IMU->acc_filter[1]*(2.0f*q3*q4 - 2.0f*q1*q2) + IMU->acc_filter[2]*(1.0f - 2.0f*q2*q2 - 2.0f*q3*q3) - g;

		for (int i = 0; i<3; i++)
		{
			IMU->gyro_filter_linear[i] += IMU->acc_filter_linear[i] * dt;
		}
		MFcount++;
}
void MadgwickFilter(MagdwitchFilter *filter, ICM20948_FILTER *IMU, float dt)
{
	if (MFcount <= 9000)
	{
		filter->beta = filter->beta_default*50;
	}
	else
		filter->beta = filter->beta_default;
	float32_t DF1, DF2, DF3, DF4;
	float32_t qdot1, qdot2, qdot3, qdot4;
	float32_t Norm;
	float32_t q1 = filter->q[0];
	float32_t q2 = filter->q[1];
	float32_t q3 = filter->q[2];
	float32_t q4 = filter->q[3];

	float32_t wx = IMU->gyro_filter[0] * DEG_TO_RAD;
	float32_t wy = IMU->gyro_filter[1] * DEG_TO_RAD;
	float32_t wz = -IMU->gyro_filter[2] * DEG_TO_RAD;

	float32_t ax = IMU->acc_filter[0];
	float32_t ay = IMU->acc_filter[1];
	float32_t az = IMU->acc_filter[2];
	filter->dt = dt;

	//- Calculate Quaternion derivative to present the angular rate
	qdot1 = 0.5f * (-q2*wx - q3*wy - q4*wz);
	qdot2 = 0.5f * (q1*wx + q3*wz - q4*wy);
	qdot3 = 0.5f * (q1*wy - q2*wz + q4*wx);
	qdot4 = 0.5f * (q1*wz + q2*wy - q3*wx);
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		//- Normalize accelerometer data
		Norm = divSqrt(ax*ax+ay*ay+az*az);
		ax *= Norm;
		ay *= Norm;
		az *= Norm;
		//- Gradient descent
		DF1 = 2*q3*ax - 2*q2*ay + 4*q1*q3*q3 + 4*q1*q2*q2;
		DF2 = - 2*q4*ax - 2*q1*ay + 4*q2*az + 4*q1*q1*q2 + 8*q2*q2*q2 + 8*q2*q3*q3 + 4*q2*q4*q4 - 4*q2;
		DF3 = 2*q1*ax - 2*q4*ay + 4*q3*az + 4*q1*q1*q3 + 8*q2*q2*q3 + 8*q3*q3*q3 + 4*q3*q4*q4 - 4*q3;
		DF4 = - 2*q2*ax - 2*q3*ay + 4*q2*q2*q4 + 4*q3*q3*q4;
		//- Normalize DeltaF data
		Norm = divSqrt(DF1*DF1 + DF2*DF2 +DF3*DF3 + DF4*DF4);
		DF1 *= Norm;
		DF2 *= Norm;
		DF3 *= Norm;
		DF4 *= Norm;
		//- Calculate Quaternion dot
		qdot1 -= filter->beta*DF1;
		qdot2 -= filter->beta*DF2;
		qdot3 -= filter->beta*DF3;
		qdot4 -= filter->beta*DF4;
	}
	//- Integral
	q1 += qdot1*filter->dt;
	q2 += qdot2*filter->dt;
	q3 += qdot3*filter->dt;
	q4 += qdot4*filter->dt;
	//- Normalize Quaternion
	Norm = divSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
	q1 *= Norm;
	q2 *= Norm;
	q3 *= Norm;
	q4 *= Norm;

	filter->q[0] = q1;
	filter->q[1] = q2;
	filter->q[2] = q3;
	filter->q[3] = q4;

	//- Quaternion to Euler Angle
	IMU->angle_rad[0] = atan2f(2*(q1*q2 + q3*q4), q1*q1 - q2*q2 - q3*q3 + q4*q4);
	IMU->angle_rad[1] = asinf(-2*(q2*q4 - q1*q3));
	IMU->angle_rad[2] = atan2f(2*(q2*q3 + q1*q4),q1*q1 + q2*q2 - q3*q3 - q4*q4);
	//- Radiant to Angle
	for (int i = 0; i < 3; i++)
	{
		IMU->angle_deg[i] = IMU->angle_rad[i] * RAD_TO_DEG;
	}
	//-Remove gravity
	IMU->acc_filter_linear[0] =	IMU->acc_filter[0]*(1 - 2*q3*q3 - 2*q4*q4) + IMU->acc_filter[1]*(2*q2*q3 + 2*q1*q4) + IMU->acc_filter[2]*(2*q2*q4 - 2*q1*q3);
	IMU->acc_filter_linear[1] = IMU->acc_filter[0]*(2*q2*q3 - 2*q1*q4) + IMU->acc_filter[1]*(1 - 2*q2*q2 - 2*q4*q4) + IMU->acc_filter[2]*(2*q1*q2 + 2*q3*q4);
	IMU->acc_filter_linear[2] = IMU->acc_filter[0]*(2*q1*q3 + 2*q2*q4) + IMU->acc_filter[1]*(2*q3*q4 - 2*q1*q2) + IMU->acc_filter[2]*(1 - 2*q2*q2 - 2*q3*q3) - g;

	for (int i = 0; i<3; i++)
	{
		IMU->gyro_filter_linear[i] += IMU->acc_filter_linear[i] * dt;
	}
	MFcount++;
}
void ComplementaryHeightProcess(ComplementaryFilterH* filter, ICM20948_FILTER* imu, MS5611_SPI* bar, float dt)
{
    if (dt <= 0.005f || dt >= 0.5f) return;
    if (MFcount < 7000) return;

	filter->dt = dt;
	filter->a = imu->acc_filter_linear[2];
	filter->xb = bar->Distance;
	//if (fabsf(filter->a) < 0.05f) filter->a = 0.0f;
    if (fabsf(filter->a) < 0.05f && fabsf(filter->v) < 0.05f && bar->Ground != 0.0f)
    {
        bar->Ground  = bar->Ground * 0.999f + bar->Height * 0.001f;
        bar->Distance = bar->Height - bar->Ground;
    }
	filter->dx = filter->xb - filter->x1;
	if (filter->dx >  0.5f) filter->dx =  0.5f;
	if (filter->dx < -0.5f) filter->dx = -0.5f;

	filter->x = filter->x1 + filter->dt * filter->v1 + (filter->dt * filter->dt / 2.0f) * filter->a1 - filter->dt * (sqrt(2.0f * filter->ow/filter->ov) + filter->dt/2.0f * filter->ow/filter->ov)*filter->dx;
	filter->v = filter->v1 + filter->dt * filter->a1 - filter->dt * filter->ow / filter->ov * filter->dx;

   if (filter->v >  5.0f) filter->v =  5.0f;
   if (filter->v < -5.0f) filter->v = -5.0f;

	filter->x1 = filter->x;
	filter->v1 = filter->v;
	filter->a1 = filter->a;
}
void KalmanFilterProcess(KalmanFilter* K, ICM20948_FILTER* imu, MS5611_SPI* bar, float dt, uint8_t baro_enable)
{
	float h_predict;
	float v_predict;
	float P00_predict, P01_predict, P10_predict, P11_predict;
	if (KFcount <= 4500)
	{
		K->Q_height = K->Q_heightSe * 100;
		K->Q_velocity = K->Q_velocitySe * 100;
	}
	else
	{
		K->Q_height = K->Q_heightSe;
		K->Q_velocity = K->Q_velocitySe;
	}

	K->dt = dt;
	K->zn = bar->Distance;
	K->un = imu->acc_filter_linear[2];
    if (fabsf(K->un) < 0.05f)  K->un = 0.0f;
	//__PREDICT__//

	//- Extrapolate the state
	h_predict = K->height + K->dt * K->velocity + 0.5f * K->dt * K->dt * K->un;
	v_predict = K->velocity + K->dt * imu->acc_filter_linear[2];

	//- Extrapolate uncertainly
	P00_predict = K->P00 + K->dt * K->P10 + K->dt * (K->P01 + K->dt * K->P11) + K->Q_height;
	P01_predict = K->P01 + K->dt * K->P11;
	P10_predict = K->P10 + K->dt * K->P11;
	P11_predict = K->P11 + K->Q_velocity;
	//__UPDATE__//
	if (baro_enable)
	{
		//- Compute the Kalman Gain
		K->L = P00_predict + K->R;
		K->K0 = P00_predict / K->L;
		K->K1 = P10_predict / K->L;

		//- Update estimate with measurement
		K->Differ = K->zn - h_predict;
		K->height = h_predict + K->K0 * K->Differ;
		K->velocity = v_predict + K->K1 * K->Differ;

		//- Update the estimate uncertainly
		K->P00 = (1 - K->K0) * P00_predict;
		K->P01 = (1 - K->K0) * P01_predict;
		K->P10 = - K->K1 * P00_predict + P10_predict;
		K->P11 = - K->K1 * P01_predict + P11_predict;
	}
	else
	{
		K->height = h_predict;
		K->velocity = v_predict;
		K->P00 = P00_predict;
		K->P01 = P01_predict;
		K->P10 = P10_predict;
		K->P11 = P11_predict;
	}
	KFcount++;
}
