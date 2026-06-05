/*
 * MS5611_SPI.c
 *
 *  Created on: Feb 26, 2026
 *      Author: tanlo
 */

#include "MS5611_SPI.h"

static void MS5611_SPI_WRITE(SPI_HandleTypeDef *hspi, uint8_t data);
static void MS5611_SPI_READ(SPI_HandleTypeDef *hspi, uint8_t *cmd, uint8_t *data, uint8_t size);

uint8_t MS5611_SPI_Init(SPI_HandleTypeDef *hspi, MS5611_SPI* datastruct)
{
	//reset
	MS5611_SPI_WRITE(hspi, CMD_RESET);
	//READ DATA FROM MS5611 MEMORY
	READ_PROM(hspi, datastruct);
	return OK;
}
uint8_t READ_PROM(SPI_HandleTypeDef *hspi, MS5611_SPI* datastruct)
{
	uint8_t PROM_CMD[8]=
	{
			CMD_PROM_READ_0,
			CMD_PROM_READ_1,
			CMD_PROM_READ_2,
			CMD_PROM_READ_3,
			CMD_PROM_READ_4,
			CMD_PROM_READ_5,
			CMD_PROM_READ_6,
			CMD_PROM_READ_7,
	};
	uint8_t data[2];
	MS5611_SPI_READ(hspi, &PROM_CMD[0], data, 2);
	datastruct->reserve = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
	//Read C1-C6
	for (uint8_t i=1; i<=6; i++)
	{
	MS5611_SPI_READ(hspi, &PROM_CMD[i], data, 2);
	datastruct->C[i-1] = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
	}
	//CRC
	MS5611_SPI_READ(hspi, &PROM_CMD[7], data, 2);
	datastruct->crc = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
	return OK;
}
uint8_t MS5611_SPI_TRIGGER_TEMP(SPI_HandleTypeDef *hspi, uint8_t resolution)
{
	uint8_t T;
	switch(resolution)
	{
		case TEMP_OSR_256: T = 1;
		break;
		case TEMP_OSR_512: T = 2;
		break;
		case TEMP_OSR_1024: T= 3;
		break;
		case TEMP_OSR_2048: T = 5;
		break;
		case TEMP_OSR_4096: T = 9;
		break;
	}
	MS5611_SPI_WRITE(hspi, resolution);
	return T;
}
void MS5611_SPI_READ_TEMP(SPI_HandleTypeDef *hspi, MS5611_SPI* datastruct)
{
	uint8_t data[3];
	uint8_t ADC_CMD = CMD_ADC_READ;
	MS5611_SPI_READ(hspi, &ADC_CMD, data, 3);
	datastruct->D[1] = (uint32_t)(((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2]);
}
uint8_t MS5611_SPI_TRIGGER_PRES(SPI_HandleTypeDef *hspi, uint8_t resolution)
{
	uint8_t T;
	switch(resolution)
		{
		case PRES_OSR_256: T = 1;
		break;
		case PRES_OSR_512: T = 2;
		break;
		case PRES_OSR_1024: T= 3;
		break;
		case PRES_OSR_2048: T = 5;
		break;
		case PRES_OSR_4096: T = 9;
		break;
		}
	MS5611_SPI_WRITE(hspi, resolution);
	return T;
}
void MS5611_SPI_READ_PRES(SPI_HandleTypeDef *hspi, MS5611_SPI* datastruct)
{
	uint8_t data[3];
	uint8_t ADC_CMD = CMD_ADC_READ;
	MS5611_SPI_READ(hspi, &ADC_CMD, data, 3);
	datastruct->D[0] = (uint32_t)(((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2]);
}

static void MS5611_SPI_WRITE(SPI_HandleTypeDef *hspi, uint8_t data)
{
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &data, 1, 10);
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS, GPIO_PIN_SET);
}

static void MS5611_SPI_READ(SPI_HandleTypeDef *hspi, uint8_t *cmd, uint8_t *data, uint8_t size)
{

	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd, 1, 10);
	HAL_SPI_Receive(hspi, data, size, 10);
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS, GPIO_PIN_SET);
}
void MS5611_SPI_CALCULATE(MS5611_SPI *datastruct)
{
	int64_t dT = 0, T2 = 0, TEMP = 0, OFF = 0, OFF2 = 0, SENS = 0, SENS2 = 0, P = 0;
	dT = datastruct->D[1]-((int64_t)(datastruct->C[4]<<8));
	TEMP = 2000 + ((int64_t) (dT*(datastruct->C[5]))>>23);
	OFF = (((int64_t)(datastruct->C[1])) << 16) + (((datastruct->C[3]) * dT) >> 7);
	SENS = (((int64_t)(datastruct->C[0])) << 15) + (((datastruct->C[2]) * dT) >> 8);

	if(TEMP < 2000) { //temperature < 20°C
		T2 = ( dT*dT )>>31;
		OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 2;
		SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 4;

		if (TEMP < -1500) { //temperature < -15°C
			OFF2 = OFF2 + (7 * (TEMP + 1500) * (TEMP + 1500));
			SENS2 = SENS2 + (11 * (TEMP + 1500) * (TEMP + 1500) / 2);
		}
	}
	else { //temperature > 20°C
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}

	datastruct->dT = dT;
	datastruct->OFF = OFF - OFF2;
	datastruct->TEMP = TEMP - T2;
	datastruct->SENS = SENS - SENS2;

	P = ((((int64_t)(datastruct->D[0]) * (datastruct->SENS))>>21) - (datastruct->OFF))>>15;
	datastruct->P = P;
}

