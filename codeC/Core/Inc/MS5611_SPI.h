/*
 * MS5611_SPI.h
 *
 *  Created on: Feb 26, 2026
 *      Author: tanlo
 */

#ifndef INC_MS5611_SPI_H_
#define INC_MS5611_SPI_H_

#include "main.h"
#include "stdio.h"
#include "stdlib.h"

#define SPI_CS_PORT GPIOA
#define SPI_CS GPIO_PIN_4
#define OK 0x01

#define CMD_ADC_READ 0x00
#define CMD_RESET 0x1E
#define CMD_PROM_READ_0 0xA0
#define CMD_PROM_READ_1 0xA2
#define CMD_PROM_READ_2 0xA4
#define CMD_PROM_READ_3 0xA6
#define CMD_PROM_READ_4 0xA8
#define CMD_PROM_READ_5 0xAA
#define CMD_PROM_READ_6 0xAC
#define CMD_PROM_READ_7 0xAE

#define PRES_OSR_256  0x40
#define PRES_OSR_512  0x42
#define PRES_OSR_1024 0x44
#define PRES_OSR_2048 0x46
#define PRES_OSR_4096 0x48

#define TEMP_OSR_256      0x50
#define TEMP_OSR_512  	  0x52
#define TEMP_OSR_1024 	  0x54
#define TEMP_OSR_2048     0x56
#define TEMP_OSR_4096     0x58

typedef struct
{
	uint8_t adress;
	uint16_t C[6];	//PROM
	uint16_t reserve;
	uint16_t crc;
	uint32_t D[2];	//D1 pressure data & D2 temperature data
	int64_t dT;	//Difference between actual and reference temperature
	int64_t OFF;	//Offset at actual temperature
	int64_t SENS;	//Sensitivity at actual temperature
	int32_t TEMP;	//Actual temperature
	int32_t P;	//Actual pressure
	float Ground;
	float Height;
	float Distance;
}MS5611_SPI;
typedef enum {
    MS5611_IDLE,
	MS5611_TRIGGER_TEMP,
    MS5611_READ_TEMP,
	MS5611_TRIGGER_PRES,
    MS5611_READ_PRES,
    MS5611_READ_PRES_CALCULATE
} MS5611_State_t;

void MS5611_SPI_READ_TEMP(SPI_HandleTypeDef *hspi, MS5611_SPI* datastruct);
void MS5611_SPI_READ_PRES(SPI_HandleTypeDef *hspi, MS5611_SPI* datastruct);
void MS5611_SPI_CALCULATE(MS5611_SPI *datastruct);
uint8_t MS5611_SPI_TRIGGER_PRES(SPI_HandleTypeDef *hspi, uint8_t resolution);
uint8_t MS5611_SPI_TRIGGER_TEMP(SPI_HandleTypeDef *hspi, uint8_t resolution);
uint8_t MS5611_SPI_Init(SPI_HandleTypeDef *hspi, MS5611_SPI* datastruct);
uint8_t READ_PROM(SPI_HandleTypeDef *hspi, MS5611_SPI* datastruct);
#endif /* INC_MS5611_SPI_H_ */
