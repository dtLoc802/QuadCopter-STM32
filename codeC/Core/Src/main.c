/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MS5611_SPI.h"
#include "ICM20948_I2C.h"
#include "RC.h"
#include "Ringbuffer.h"
#include "HC05.h"
#include "PIDControl.h"
#include "Filters.h"
#include "MTF01P.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t CHANNEL[4];
float GYRO_NOTCH[3];
float GYRO_LPF[3];
float ACC_NOTCH[3];
float ACC_LPF[3];
enum XYZ
{
	X,
	Y,
	Z,
};
//- Variables for pitch, roll, yaw
ICM20948_ACCEL_DATA acc;
ICM20948_GYRO_DATA gyro;
AK09916_DATA mag = { 0 };
ICM20948_FILTER imu;
uint8_t ICM20948_status = 0;
LowPassFilter gyro_LPF[3];
LowPassFilter acc_LPF[3];
LowPassFilter mag_LPF;
BiquadFilter mag_NOTCH;
BiquadFilter gyro_NOTCH[3];
MagdwitchFilter MF = {
		.beta_default = 0.05,
		.q[0] = 1,
		.q[1] = 0,
		.q[2] = 0,
		.q[3] = 0,
};
float f_ICM20948;
float overhead;
volatile uint8_t DATA_ICM20948_READY = 0;
float temp_yaw;
uint32_t ICM20948_Count = 0;
uint32_t ICM20948_dt = 0;
uint32_t ICM20948_Start = 0;
uint32_t ICM20948_PreCount = 0;
int16_t stick;
//- Variables for thrust
uint8_t MS5611_status = 0;
uint8_t MS5611_flag = 0;
uint32_t MS5611_count = 0;
uint32_t KF_Start = 0;
uint32_t Pre_KF_Start = 0;
uint32_t PRES_time;
uint32_t PRES_timepre;
float dt_PRES;
uint32_t TEMP_time;
uint32_t TEMP_timepre;
float dt_TEMP;
float dt_KF = 0;
float f_KF = 0;
MS5611_SPI MS5611;
MS5611_State_t State = MS5611_IDLE;
uint16_t pres_reference;
uint16_t pres_ground;
ring_buffer HC05_data;
uint8_t count = 0;
uint8_t SendHeight = 0;
char temp_hc05[64];
uint32_t StartTick = 0, EndTick = 0, StartTick_MS = 0, EndTick_MS = 0;
float dt_ICM20948 = 0, dt_MS5611 = 0;
float f_MS5611;
float sample[1000];
//- Variables for XYZ
float mean;
float variance, sigma;
ComplementaryFilterH CFH = {
		.ow = 0.010,
		.ov = 0.30,
		.a = 0,
		.v = 0,
		.x = 0,
		.dt = 0.018
};
KalmanFilter KF = {
		.Q_positionSe = 0.001,
		.Q_velocitySe = 0.0001,
		.Rbar = 0.06,
		.RoptP = 0.00003,
		.RoptV = 0.000005
};
uint8_t temp_TMF01P;
uint32_t MFcount = 0;
uint32_t KFcount = 0;
uint8_t ENABLE_FLY = 0;
uint32_t HC05_Start = 0;
uint32_t MTF01P_current_time;
uint32_t MTF01P_previous_time;
uint16_t MTF01P_dt_desired = 10000;
float dt_MTF01P;
float f_MTF01P;
//-Safe
uint8_t Safety_flag = 0;
uint16_t PWM1, PWM2, PWM3, PWM4;
uint8_t SetYaw_flag = 0;
float yaw_reference;
float thrust_value;

uint8_t TurnOnHeight= 0;
// -PID various
uint32_t current_time_pid_pos;
uint32_t previous_time_pid_pos;
uint32_t current_time_pid_vel;
uint32_t previous_time_pid_vel;
uint32_t current_time_pid_yaw;
uint32_t previous_time_pid_yaw;
uint32_t current_time_pid_angle;
uint32_t previous_time_pid_angle;
uint32_t current_time_pid_rate;
uint32_t previous_time_pid_rate;
uint32_t current_time_pid_yaw_rate;
uint32_t previous_time_pid_yaw_rate;
uint32_t current_time_pid_height;
uint32_t previous_time_pid_height;
uint32_t current_time_pid_velH;
uint32_t previous_time_pid_velH;
uint32_t catch_yaw;
uint32_t catch_angle;
uint32_t catch_rate;
float dt_pid_yaw;
float dt_pid_yaw_rate;
float dt_pid_pos;
float dt_pid_vel;
float dt_pid_angle;
float dt_pid_rate;
float dt_pid_height;
float dt_pid_velH;
float f_pid_yaw;
float f_pid_yaw_rate;
float f_pid_pos;
float f_pid_vel;
float f_pid_angle;
float f_pid_rate;
float f_pid_height;
float f_pid_velH;
uint32_t dt_pid_pos_desire = 100000;
uint16_t dt_pid_vel_desire = 50000;
uint16_t dt_pid_yaw_angle_desire = 10000; // us
uint16_t dt_pid_yaw_rate_desire = 5000;
uint16_t dt_pid_angle_desire = 5000; // us
uint16_t dt_pid_rate_desire = 2500; // us
uint32_t dt_pid_height_desire = 100000; // us
uint16_t dt_pid_velH_desire = 20000; // us
float PID_velocity_setpoint[4];
float PID_angle_setpoint[4];
float PID_rate_setpoint[4];
float height_setpoint;
float pid_velH_setpoint;
PIDsingle PitchRate = {
		.par.kp = 1.1,
		.par.ki = 1.5,
		.par.kd = 0.0035,
};
PIDsingle RollRate = {
		.par.kp = 1.1,
		.par.ki = 1.5,
		.par.kd = 0.0035,
};
PIDsingle YawRate = {
		.par.kp = 1.1,
		.par.ki = 0.3,
		.par.kd = 0.0025,
};
PIDsingle PitchAngle = {
		.par.kp = 1.35,
		.par.ki = 0.0,
		.par.kd = 0.001,
};
PIDsingle RollAngle = {
		.par.kp = 1.35,
		.par.ki = 0.0,
		.par.kd = 0.001,
};
PIDsingle YawAngle = {
		.par.kp = 1.35,
		.par.ki = 0.0,
		.par.kd = 0.0,
};
PIDsingle Height = {
		.par.kp = 2.5,
		.par.ki = 0.0,
		.par.kd = 0.0,
};
PIDsingle VelH = {
		.par.kp = 40.0,
		.par.ki = 0.5,
		.par.kd = 2.0,
};
PIDsingle Position;
PIDsingle Velocitydx = {
		.par.kp = 0,
		.par.ki = 0,
		.par.kd = 2.0,
};
PIDsingle Velocitydy = {
		.par.kp = 0,
		.par.ki = 0,
		.par.kd = 2.0,
};
uint8_t missout = 0;
//-
char last;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Filter_init(void);
void ICM20948_IMU_init(void);
void MS5611_BAR_init(void);
void PID_init(void);
void HC05_PROCESS(void);
void MTF01P_PROCESS(void);
void MTF01P_s(void);
void ICM20948_IMU(void);
void ICM20948_s(void);
void MS5611_BAR(void);
void MS5611_s(void);
void PID_YAW_PROCESS(void);
void PID_POSITION_PROCESS(void);
void PID_VELOCITY_PROCESS(void);
void PID_ANGLE_PROCESS(void);
void PID_RATE_PROCESS(void);
void PID_HEIGHT_PROCESS(void);
void PID_VELH_PROCESS(void);
void PID_CONTROLLER(void);
void MOTOR_CONTROLLER(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  __enable_irq();
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
//- Initialize Filters
  Filter_init();
//- Initialize RingBuffer
  RINGBUF_INIT(&HC05, &huart6);
  RINGBUF_INIT(&MTF01P, &huart1);
//- Initialize MS5611
  MS5611_BAR_init();
//- Initialize ICM-20948
  ICM20948_IMU_init();
//- Initialize PID
  PID_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HC05_PROCESS();
	  MTF01P_PROCESS();
	  //MTF01P_s();
	  //ak09916_calibrate(&hi2c1, &mag);
//- Receive and calculate data from MS5611 barometric pressure sensor
	  MS5611_BAR();
	  //MS5611_s();
//- Receive and calculate data from ICM-20948 9-Axits IMU
	  ICM20948_IMU();
	  //ICM20948_s();
//- Control motors via PID controller
	  PID_CONTROLLER();
	  MOTOR_CONTROLLER();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	RC_CAPTURE_PWM(htim, CHANNEL);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == ICM20948_INT_Pin)
	{
		DATA_ICM20948_READY = 1;
		ICM20948_Count++;
	}
}
void MS5611_BAR_init(void)
{
	if (MS5611_SPI_Init(&hspi1, &MS5611) == OK)
	MS5611_status = 1;
	else
	{
		while (1)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_Delay(1000);
		}
	}
}
void ICM20948_IMU_init(void)
{

	if (ICM20948_I2C_Init(&hi2c1) == HAL_OK)
	{
		ICM20948_calirate_accel_gyro(&hi2c1, &gyro, &acc);
		mag.xOffset = 73;
		mag.yOffset = 205;
		mag.zOffset = 116.5;

		mag.xScale = 0.00388349523;
		mag.yScale = 0.00392156886;
		mag.zScale = 0.00858369097;
		ICM20948_status = 1;
	}
	if ((MS5611_status == 1)&&(ICM20948_status == 1)) HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	else
	  {
		  	  while (1)
	  		{
	  			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  			HAL_Delay(1000);
	  		}
	  }
}
void Filter_init(void)
{
	LowpassFilterInit(&PitchAngle.LPF_Kd, 100, 0.00264599989);
	LowpassFilterInit(&RollRate.LPF_Kd, 100, 0.00264599989);
	LowpassFilterInit(&YawRate.LPF_Kd, 100, 0.00501499977);

	for (int i = 0; i <3; i++)
	{
		BiquadFilterNotchInit(&gyro_NOTCH[i], 150, 50, 887.8);
		LowpassFilterInit(&gyro_LPF[i], 100, 0.0008878);
		LowpassFilterInit(&acc_LPF[i], 100, 0.0008878);
		LowpassFilterInit(&mag_LPF, 10, 0.0008878);
	}
}
void PID_init(void)
{
	PID_Init(&PitchRate);
	PID_Init(&RollRate);
	PID_Init(&YawRate);
	PID_Init(&RollAngle);
	PID_Init(&PitchAngle);
	PID_Init(&YawAngle);
	PID_Init(&Height);
	PID_Init(&VelH);
	PID_Init(&Position);
	PID_Init(&Velocitydx);
	PID_Init(&Velocitydy);
}
void HC05_PROCESS(void)
{
		  if (int_uart6_flag == 1)
		  {
			  int_uart6_flag = 0;
			  int idx = 0;
			  while (AVAILABLEDATA(&HC05) > 0)
			  {
				  temp_hc05[idx++] = UART_READ(&HC05);
			  }
			  char *rx_buffer_hc05 = temp_hc05;
			  while ((*rx_buffer_hc05 == '\r') || (*rx_buffer_hc05 == '\n'))
				  rx_buffer_hc05++;
	//
	//		  last = rx_buffer_hc05[idx-2];
	//		  if (last == 'I'  || last == 'O' || last == 'K')
			  HC05_PID_SETTINGS(&huart6, rx_buffer_hc05, &HC05.rx_byte, idx, &PitchAngle, &RollAngle, &YawAngle, &PitchRate, &RollRate, &YawRate, &Height, &VelH);
			  PitchRate.par.kp = RollRate.par.kp;
			  PitchRate.par.ki = RollRate.par.ki;
			  PitchRate.par.kd = RollRate.par.kd;
			  RollRate.par.kp = PitchRate.par.kp;
			  RollRate.par.ki = PitchRate.par.ki;
			  RollRate.par.kd = PitchRate.par.kd;

			  PitchAngle.par.kp = RollAngle.par.kp;
			  PitchAngle.par.ki = RollAngle.par.ki;
			  PitchAngle.par.kd = RollAngle.par.kd;
			  RollAngle.par.kp = PitchAngle.par.kp;
			  RollAngle.par.ki = PitchAngle.par.ki;
			  RollAngle.par.kd = PitchAngle.par.kd;
		  }
		  if (SendHeight)
		  {
			  if (TIM2->CNT - HC05_Start >= 500000)
			  {
				HC05_Start = TIM2->CNT;
				char msg[50];
				sprintf(msg, "HeightSP = %.2f\r\n HeightRV = %.2f\r\n", Height.reference, Height.measure_value);
				const char *msg1 = msg;
				while (*msg1)
				{
					UART_WRITE(&HC05, (int)(*msg1++));
				}
			  }
		  }
}
void MTF01P_PROCESS(void)
{
	if (TIM2->CNT - MTF01P_current_time >= MTF01P_dt_desired)
	{
		MTF01P_previous_time = MTF01P_current_time;
		MTF01P_current_time = TIM2->CNT;
		dt_MTF01P = (MTF01P_current_time - MTF01P_previous_time)*0.000001;
		f_MTF01P = 1 / dt_MTF01P;

		while (AVAILABLEDATA(&MTF01P) > 0)
		{
		temp_TMF01P = (uint8_t)UART_READ(&MTF01P);
		micolink_decode(temp_TMF01P);
		}
	}
}
void ICM20948_IMU(void)
{

	if (TIM2->CNT - ICM20948_Start >= 1000000)
	{
		ICM20948_Start = TIM2->CNT;
		if (ICM20948_Count - ICM20948_PreCount < 1000)
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,  GPIO_PIN_RESET);
		}
		ICM20948_PreCount = ICM20948_Count;
	}
	if (DATA_ICM20948_READY)
	{
		DATA_ICM20948_READY = 0;
		//dma_done = 0;
		ICM20948_READ_ACCEL_GYRO_MAG_DMA(&hi2c1, &acc, &gyro, &mag);
		temp_yaw = atan2(-mag.MAG_DATA[1], mag.MAG_DATA[0])*57.29577951;
//		ak09916_calibrate(&hi2c1, &mag);
//		if (temp_yaw < 0) temp_yaw += 360;
	}
	if (data_ready)
	{
		data_ready = 0;
		EndTick = StartTick;
		StartTick = TIM2->CNT;
		dt_ICM20948 = (float)(StartTick - EndTick) * 0.000001f;
		imu.yaw = LowpassFilterProcess(&mag_LPF, temp_yaw);
		for (int i = 0; i < 3; i++)
		{
			GYRO_NOTCH[i] = BiquadFilterProcessTDF2(&gyro_NOTCH[i], gyro.GYRO_DATA[i]);
			imu.gyro_filter[i] = LowpassFilterProcess(&gyro_LPF[i], GYRO_NOTCH[i]);
			imu.acc_filter[i] = LowpassFilterProcess(&acc_LPF[i], acc.ACC_DATA[i]);
		}

		MadgwickFilter_Update(&MF, &imu, &mag, dt_ICM20948);
		//MadgwickFilter(&MF, &imu, dt_ICM20948);
		if(MFcount > 9000 && SetYaw_flag == 2)
				{
					KalmanFilterProcess(&KF, &imu, &MS5611, &payload, dt_ICM20948, MS5611_flag, MTF01P_flag);
					MS5611_flag = 0;
					if (KFcount <= 4500 && (KFcount % 125 == 0))
						HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					else if (KFcount > 4500 && abs(KF.velocity[2]) < 0.1 && abs(KF.position[2]) < 0.1)
						ENABLE_FLY = 1;
				}
		else if (MFcount <= 9000 && (MFcount % 72 == 0))
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		f_ICM20948 = 1 / dt_ICM20948;
	}

}
void MS5611_BAR(void)
{
//	HAL_Delay(MS5611_SPI_TRIGGER_TEMP(&hspi1, TEMP_OSR_4096));
//	MS5611_SPI_READ_TEMP(&hspi1, &MS5611);
//	HAL_Delay(MS5611_SPI_TRIGGER_PRES(&hspi1, PRES_OSR_4096));
//	MS5611_SPI_READ_PRES(&hspi1, &MS5611);
//
//	MS5611_SPI_CALCULATE(&MS5611);
	static uint32_t now = 0;
	static uint32_t wait_time = 0;

	switch (State)
	{
	case (MS5611_IDLE):
		count++;
		if (count >= 5)
		{
			count = 0;
			State = MS5611_TRIGGER_TEMP;
		}
		break;
	case (MS5611_TRIGGER_TEMP):
		wait_time = MS5611_SPI_TRIGGER_TEMP(&hspi1, TEMP_OSR_4096)*1000+500;
		now = TIM2->CNT;
		State = MS5611_READ_TEMP;
		break;
	case (MS5611_READ_TEMP):
		if ((TIM2->CNT - now) >= wait_time)
		{
			TEMP_timepre = TEMP_time;
			TEMP_time = TIM2->CNT;

			dt_TEMP = (TEMP_time - TEMP_timepre)*0.000001;
			MS5611_SPI_READ_TEMP(&hspi1, &MS5611);
			State = MS5611_TRIGGER_PRES;
		}
		break;
	case (MS5611_TRIGGER_PRES):
		wait_time = MS5611_SPI_TRIGGER_PRES(&hspi1, PRES_OSR_4096)*1000+500;
		now = TIM2->CNT;
		State = MS5611_READ_PRES;
		break;
	case (MS5611_READ_PRES):
		if ((TIM2->CNT - now) >= wait_time)
		{
			PRES_timepre = PRES_time;
			PRES_time = TIM2->CNT;

			dt_PRES = (PRES_time - PRES_timepre)*0.000001;
			MS5611_SPI_READ_PRES(&hspi1, &MS5611);
			State = MS5611_READ_PRES_CALCULATE;
		}
	break;
	case (MS5611_READ_PRES_CALCULATE):
			MS5611_SPI_CALCULATE(&MS5611);
			MS5611.Height = (float) 44330 * (1 - pow((float)MS5611.P/101352,0.19));
			if (MS5611.Ground == 0 && MS5611_count > 50)
				MS5611.Ground = MS5611.Height;
			if (ENABLE_FLY == 0 && MS5611.Ground != 0)
			    MS5611.Ground = MS5611.Height;
			else if (ENABLE_FLY == 1 && fabsf(KF.velocity[2]) < 0.1f && fabsf(MS5611.Distance) > 0.1f && CHANNEL[THRUST] == 1000)
			    MS5611.Ground = MS5611.Ground * 0.9f + MS5611.Height * 0.1f;
			MS5611.Distance = MS5611.Height - MS5611.Ground;
			MS5611_flag = 1;
			MS5611_count++;
			EndTick_MS = StartTick_MS;
			StartTick_MS = TIM2->CNT;
			dt_MS5611 = (float) (StartTick_MS - EndTick_MS) * 0.000001f;
			f_MS5611 = 1 / dt_MS5611;

			State = MS5611_IDLE;
			break;
	}
}
void PID_CONTROLLER(void)
{
//- Set yaw reference for YAWPID
	while (SetYaw_flag < 2)
	{

		ICM20948_IMU();

		if (CHANNEL[ROLL] == 1000 && CHANNEL[THRUST] == 1000)
			SetYaw_flag = 1;
		if (SetYaw_flag)
		{
			if (dma_done)
				yaw_reference = imu.angle_deg[2];
			if (CHANNEL[ROLL] == 2000 && MFcount > 9000)
			{
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				SetYaw_flag = 2;
			}
		}
	}

//- If kp haven't set yet, it automatically change to RATE control
	if (PitchAngle.par.kp == 0 && RollAngle.par.kp == 0 && Height.par.kp == 0 && Position.par.kp == 0)
	// RATE PID
	{
		if (CHANNEL[THRUST] == 1000)
		{
			PID_Reset(&YawRate);
			PID_Reset(&PitchRate);
			PID_Reset(&RollRate);
			if (CHANNEL[PITCH] == 1000)
				yaw_reference = imu.angle_deg[2];
		}
		else
		{
			PID_rate_setpoint[PITCH] = -(CHANNEL[PITCH]-1500)*0.1;
			PID_rate_setpoint[ROLL] = (CHANNEL[ROLL]-1500)*0.1;
			PID_rate_setpoint[YAW] = 0;
			PID_RATE_PROCESS();
		}
	}
	// ANGLE PID
	else if (Height.par.kp == 0 && Velocitydx.par.kp == 0 && Velocitydy.par.kp == 0)
	{
		if (CHANNEL[THRUST] == 1000)
		{
			PID_Reset(&YawAngle);
			PID_Reset(&PitchAngle);
			PID_Reset(&RollAngle);
			PID_Reset(&YawRate);
			PID_Reset(&PitchRate);
			PID_Reset(&RollRate);
			if (CHANNEL[PITCH] == 1000)
				yaw_reference = imu.angle_deg[2];
		}
		else
		{
			PID_YAW_PROCESS();
			PID_ANGLE_PROCESS();
			PID_RATE_PROCESS();
			//pid_velH_PROCESS();
		}
	}
	else
	{
		if (CHANNEL[THRUST] == 1000)
		{
			PID_Reset(&Position);
			PID_Reset(&Velocitydx);
			PID_Reset(&Velocitydy);
			PID_Reset(&YawAngle);
			PID_Reset(&PitchAngle);
			PID_Reset(&RollAngle);
			PID_Reset(&YawRate);
			PID_Reset(&PitchRate);
			PID_Reset(&RollRate);
			PID_Reset(&Height);
			PID_Reset(&VelH);
			current_time_pid_pos	   = TIM2->CNT;
			previous_time_pid_pos	   = TIM2->CNT;
			current_time_pid_vel	   = TIM2->CNT;
			previous_time_pid_vel	   = TIM2->CNT;
			current_time_pid_rate      = TIM2->CNT;
			previous_time_pid_rate     = TIM2->CNT;
			current_time_pid_yaw_rate  = TIM2->CNT;
			previous_time_pid_yaw_rate = TIM2->CNT;
			current_time_pid_angle     = TIM2->CNT;
			previous_time_pid_angle    = TIM2->CNT;
			current_time_pid_yaw       = TIM2->CNT;
			previous_time_pid_yaw      = TIM2->CNT;
			current_time_pid_height   = TIM2->CNT;
			previous_time_pid_height  = TIM2->CNT;
			current_time_pid_velH = TIM2->CNT;
			previous_time_pid_velH= TIM2->CNT;
			if (CHANNEL[PITCH] == 1000)
				yaw_reference = imu.angle_deg[2];
			if (CHANNEL[PITCH] == 2000)
				MS5611.Ground = MS5611.Height;
			}
			else
			{
				//PID_POSITION_PROCESS();
				PID_VELOCITY_PROCESS();
				PID_YAW_PROCESS();
				PID_ANGLE_PROCESS();
				PID_RATE_PROCESS();
				PID_HEIGHT_PROCESS();
				PID_VELH_PROCESS();
			}
		}
}


void PID_POSITION_PROCESS(void)
{
	if((TIM2->CNT - current_time_pid_pos) >= dt_pid_pos_desire)
	{
		previous_time_pid_pos = current_time_pid_pos;
		current_time_pid_pos = TIM2->CNT;
		dt_pid_pos = (current_time_pid_pos - previous_time_pid_pos)*0.000001;
		f_pid_pos = 1/dt_pid_pos;
		//PID_velocity_setpoint[PITCH] = PID_Caculate(&Position, (CHANNEL[PITCH]-1500)*0.1, 0, dt_pid_pos, -50, 50, 0);
		//PID_velocity_setpoint[ROLL] = PID_Caculate(&Position, (CHANNEL[ROLL]-1500)*0.1, 0, dt_pid_pos, -50, 50, 0);

	}
}
void PID_VELOCITY_PROCESS(void)
{
	if((TIM2->CNT - current_time_pid_vel) >= dt_pid_vel_desire)
	{
		previous_time_pid_vel = current_time_pid_vel;
		current_time_pid_vel = TIM2->CNT;
		dt_pid_vel = (current_time_pid_vel - previous_time_pid_vel)*0.000001;
		f_pid_vel = 1/dt_pid_vel;
		PID_angle_setpoint[PITCH] = PID_Caculate(&Velocitydy, -(CHANNEL[PITCH]-1500)*0.01, KF.velocity[1], dt_pid_vel, -50, 50, 0);
		PID_angle_setpoint[ROLL] = PID_Caculate(&Velocitydx, (CHANNEL[ROLL]-1500)*0.01, KF.velocity[0], dt_pid_vel, -50, 50, 0);
	}
}
void PID_YAW_PROCESS(void)
{
	if((TIM2->CNT - current_time_pid_yaw) >= (dt_pid_yaw_angle_desire))
	{
		previous_time_pid_yaw = current_time_pid_yaw;
		current_time_pid_yaw = TIM2->CNT;
		dt_pid_yaw = (current_time_pid_yaw - previous_time_pid_yaw)*0.000001;
		f_pid_yaw = 1 / dt_pid_yaw;
		PID_rate_setpoint[YAW] = PID_Caculate(&YawAngle, yaw_reference, imu.angle_deg[2], dt_pid_yaw, -200, 200, 1);
	}
}
void PID_ANGLE_PROCESS(void)
{
	if((TIM2->CNT - current_time_pid_angle) >= (dt_pid_angle_desire))
	{
		previous_time_pid_angle = current_time_pid_angle;
		current_time_pid_angle = TIM2->CNT;
		dt_pid_angle = (current_time_pid_angle - previous_time_pid_angle)*0.000001;
		f_pid_angle = 1 / dt_pid_angle;
		if (Velocitydx.par.kp > 0 && Velocitydy.par.kp > 0 )
		{
			PID_rate_setpoint[PITCH] = PID_Caculate(&PitchAngle, PID_angle_setpoint[PITCH], imu.angle_deg[0], dt_pid_angle, -1000, 1000, 0);
			PID_rate_setpoint[ROLL] = PID_Caculate(&RollAngle, PID_angle_setpoint[ROLL], -imu.angle_deg[1], dt_pid_angle, -1000, 1000, 0);
		}
		else
		{
			PID_rate_setpoint[PITCH] = PID_Caculate(&PitchAngle, -(CHANNEL[PITCH]-1500)*0.1, imu.angle_deg[0], dt_pid_angle, -1000, 1000, 0);
			PID_rate_setpoint[ROLL] = PID_Caculate(&RollAngle, (CHANNEL[ROLL]-1500)*0.1, -imu.angle_deg[1], dt_pid_angle, -1000, 1000, 0);
		}
	}
}
void PID_RATE_PROCESS(void)
{
	if ((TIM2->CNT - current_time_pid_rate) >= dt_pid_rate_desire)
	{
		previous_time_pid_rate = current_time_pid_rate;
		current_time_pid_rate = TIM2->CNT;
		dt_pid_rate = (current_time_pid_rate - previous_time_pid_rate)*0.000001;
		f_pid_rate = 1 / dt_pid_rate;
		PID_Caculate(&PitchRate, PID_rate_setpoint[PITCH], imu.gyro_filter[0], dt_pid_rate, -1000, 1000, 0);
		PID_Caculate(&RollRate, PID_rate_setpoint[ROLL], -imu.gyro_filter[1], dt_pid_rate, -1000, 1000, 0);
	}
	if ((TIM2->CNT - current_time_pid_yaw_rate) >= dt_pid_yaw_rate_desire)
	{
		previous_time_pid_yaw_rate = current_time_pid_yaw_rate;
		current_time_pid_yaw_rate = TIM2->CNT;
		dt_pid_yaw_rate = (current_time_pid_yaw_rate - previous_time_pid_yaw_rate)*0.000001;
		f_pid_yaw_rate = 1 / dt_pid_yaw_rate;
		PID_Caculate(&YawRate, PID_rate_setpoint[YAW], - imu.gyro_filter[2], dt_pid_yaw_rate, -1000, 1000, 0);
	}
}
void PID_HEIGHT_PROCESS(void)
{
	stick = CHANNEL[THRUST] - 1500;
	if ((TIM2->CNT - current_time_pid_height) >= dt_pid_height_desire)
	{
		previous_time_pid_height = current_time_pid_height;
		current_time_pid_height = TIM2->CNT;
		dt_pid_height = (current_time_pid_height - previous_time_pid_height)*0.000001;
        if (dt_pid_height > 0.2f)  dt_pid_height = 0.2f;
        if (dt_pid_height < 0.0f)  dt_pid_height = 0.01f;
		f_pid_height = 1 / dt_pid_height;
		if (abs(stick)>50)
		{
			height_setpoint += (float)stick*0.001*dt_pid_height;
			height_setpoint = height_setpoint < 0.15 ? 0.15 : height_setpoint > 4.0 ? 4.0 : height_setpoint;
		}
		else
		{

		}
		pid_velH_setpoint = PID_Caculate(&Height, height_setpoint, KF.position[2], dt_pid_height, -3, 3, 0);
	}
}
void PID_VELH_PROCESS(void)
{
	if ((TIM2->CNT - current_time_pid_velH) >= dt_pid_velH_desire)
	{
		previous_time_pid_velH = current_time_pid_velH;
		current_time_pid_velH = TIM2->CNT;
		dt_pid_velH = (current_time_pid_velH - previous_time_pid_velH)*0.000001;
        if (dt_pid_velH > 0.1f)  dt_pid_velH = 0.1f;
        if (dt_pid_velH < 0.001f) dt_pid_velH = 0.02f;
		f_pid_velH = 1 / dt_pid_velH;
		PID_Caculate(&VelH, pid_velH_setpoint, KF.velocity[2], dt_pid_velH, -300, 300, 0);
	}
}
void MOTOR_CONTROLLER(void)
{
	if (ENABLE_FLY)
	{
		thrust_value = 0.0000038*CHANNEL[THRUST]*CHANNEL[THRUST]*CHANNEL[THRUST]-0.0171*CHANNEL[THRUST]*CHANNEL[THRUST]+25.7*CHANNEL[THRUST]-12400;
		//thrust_value = 0.0000038*(CHANNEL[THRUST]-1000)*(CHANNEL[THRUST]-1400)*(CHANNEL[THRUST]-2000)+CHANNEL[THRUST]-1000;
		PWM1 = 1000 + thrust_value + VelH.pid_result_constrain - RollRate.pid_result_constrain + PitchRate.pid_result_constrain - YawRate.pid_result_constrain;
		PWM2 = 1000 + thrust_value + VelH.pid_result_constrain + RollRate.pid_result_constrain + PitchRate.pid_result_constrain + YawRate.pid_result_constrain;
		PWM3 = 1000 + thrust_value + VelH.pid_result_constrain + RollRate.pid_result_constrain - PitchRate.pid_result_constrain - YawRate.pid_result_constrain;
		PWM4 = 1000 + thrust_value + VelH.pid_result_constrain - RollRate.pid_result_constrain - PitchRate.pid_result_constrain + YawRate.pid_result_constrain;

		PWM1 = PWM1 < 1000 ? 1000 : PWM1 > 2000 ? 2000 : PWM1;
		PWM2 = PWM2 < 1000 ? 1000 : PWM2 > 2000 ? 2000 : PWM2;
		PWM3 = PWM3 < 1000 ? 1000 : PWM3 > 2000 ? 2000 : PWM3;
		PWM4 = PWM4 < 1000 ? 1000 : PWM4 > 2000 ? 2000 : PWM4;

		if (CHANNEL[THRUST] >= 1010)
		{
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PWM1);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PWM2);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, PWM3);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, PWM4);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 1000);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 1000);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 1000);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 1000);
		}
	}
}
void MS5611_s(void)
{
	for (int i = 0; i < 1000; i++)
	{
		MS5611_BAR();
		sample[i] = MS5611.Height;
		mean += sample[i];
	}
	mean /= 1000;
	for (int i = 0; i < 1000; i++)
	{
		variance = (MS5611.Height - mean) * (MS5611.Height - mean);
		sigma = sqrt(variance);
	}
}
void ICM20948_s(void)
{
	for (int i = 0; i < 1000; i++)
	{
		ICM20948_IMU();
		sample[i] = imu.acc_filter_linear[2];
		mean += sample[i];
	}
	mean /= 1000;
	for (int i = 0; i < 1000; i++)
	{
		variance = (imu.acc_filter_linear[2] - mean) * (imu.acc_filter_linear[2] - mean);
		sigma = sqrt(variance);
	}
}
void MTF01P_s(void)
{
	for (int i = 0; i < 1000; i++)
	{
		MS5611_BAR();
		sample[i] = payload.flow_vel_x;
		mean += sample[i];
	}
	mean /= 1000;
	for (int i = 0; i < 1000; i++)
	{
		variance = (payload.flow_vel_x - mean) * (payload.flow_vel_x - mean);
	}
	sigma = variance / 1000;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
