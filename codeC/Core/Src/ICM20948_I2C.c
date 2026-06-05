/*
 * ICM20948_I2C.c
 *
 *  Created on: Feb 26, 2026
 *      Author: tanlo
 */
#include "ICM20948_I2C.h"
#include "math.h"
// Constants for conversion


float gyro_scale = 131.0f; //Default is 250dps
float accel_scale = 16384.0f; //Default is 2g
static uint8_t bytes_DMA[20];
volatile uint8_t dma_done = 1;
volatile uint8_t data_ready = 1;
volatile uint8_t mag_updated = 0;
//static float prev_mag_raw = 0;
ICM20948_GYRO_DATA *gyro_DMA;
ICM20948_ACCEL_DATA *acc_DMA;
AK09916_DATA *mag_DMA;
// Private function to set a register bank

static HAL_StatusTypeDef ICM20948_WRITE(I2C_HandleTypeDef *hi2c, uint8_t ChipAddress, uint8_t MemAddress, uint8_t* data, uint8_t Size);
static HAL_StatusTypeDef ICM20948_READ(I2C_HandleTypeDef *hi2c, uint8_t ChipAddress, uint8_t MemAddress, uint8_t* data, uint8_t Size);
static HAL_StatusTypeDef ICM20948_SET_BANK(I2C_HandleTypeDef *hi2c, uint8_t bank);
static HAL_StatusTypeDef ICM20948_WRITE_AK09916(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value, uint8_t length);
static HAL_StatusTypeDef ICM20948_READ_AK09916(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t length);
//static HAL_StatusTypeDef AK09916_init(I2C_HandleTypeDef *hi2c);

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) return;


    acc_DMA->ACC_DATA[0] = ((float)(int16_t)((bytes_DMA[0]  << 8) | bytes_DMA[1])  / accel_scale)*g - acc_DMA->x_offset;
    acc_DMA->ACC_DATA[1] = ((float)(int16_t)((bytes_DMA[2]  << 8) | bytes_DMA[3])  / accel_scale)*g - acc_DMA->y_offset;
    acc_DMA->ACC_DATA[2] = ((float)(int16_t)((bytes_DMA[4]  << 8) | bytes_DMA[5])  / accel_scale)*g - acc_DMA->z_offset;

    gyro_DMA->GYRO_DATA[0] = ((float)(int16_t)((bytes_DMA[6]  << 8) | bytes_DMA[7])  / gyro_scale) - gyro_DMA->x_offset;
    gyro_DMA->GYRO_DATA[1] = ((float)(int16_t)((bytes_DMA[8]  << 8) | bytes_DMA[9])  / gyro_scale) - gyro_DMA->y_offset;
    gyro_DMA->GYRO_DATA[2] = ((float)(int16_t)((bytes_DMA[10] << 8) | bytes_DMA[11]) / gyro_scale) - gyro_DMA->z_offset;

	mag_DMA->MAG_DATA[0] = ((float)(int16_t)((bytes_DMA[15] << 8) | bytes_DMA[14]) - mag_DMA->xOffset) * mag_DMA->xScale;
	mag_DMA->MAG_DATA[1] = ((float)(int16_t)((bytes_DMA[17] << 8) | bytes_DMA[16]) - mag_DMA->yOffset) * mag_DMA->yScale;
	mag_DMA->MAG_DATA[2] = ((float)(int16_t)((bytes_DMA[19] << 8) | bytes_DMA[18]) - mag_DMA->zOffset) * mag_DMA->zScale;

//    if (bytes_DMA[15] != prev_mag_raw)  // mag changed
//    {
//        mag_updated = 1;
//        prev_mag_raw = bytes_DMA[15];
//    }
    dma_done = 1;
    data_ready = 1;
}

static HAL_StatusTypeDef ICM20948_WRITE(I2C_HandleTypeDef *hi2c, uint8_t ChipAddress, uint8_t MemAddress, uint8_t* data, uint8_t Size)
{
	uint8_t tx_data = *data;
	uint8_t rx_data = 0;
	while (rx_data != tx_data)
	{
		if (HAL_I2C_Mem_Write(hi2c, ChipAddress, MemAddress, 1, data, Size, 100) != HAL_OK)
			return HAL_ERROR;
		if (HAL_I2C_Mem_Read(hi2c, ChipAddress, MemAddress, 1, data, Size, 100) != HAL_OK)
			return HAL_ERROR;
		rx_data = *data;
	}
	*data = 0x00;
	return HAL_OK;
}
static HAL_StatusTypeDef ICM20948_READ(I2C_HandleTypeDef *hi2c, uint8_t ChipAddress, uint8_t MemAddress, uint8_t* data, uint8_t Size)
{
	if (HAL_I2C_Mem_Read(hi2c, ChipAddress, MemAddress, 1, data, Size, 100) != HAL_OK)
		return HAL_ERROR;
	return HAL_OK;
}
static HAL_StatusTypeDef ICM20948_SET_BANK(I2C_HandleTypeDef *hi2c, uint8_t bank)
{
	uint8_t tx_data = (bank << 4);
	return HAL_I2C_Mem_Write(hi2c, ICM20948_ADDR_AD0_LOW, REG_BANK_SEL, 1,
			&tx_data, 1, 1);
}
HAL_StatusTypeDef ICM20948_I2C_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t rx_data = 0;
	uint8_t tx_data = 0;
	// BANK 0
	ICM20948_SET_BANK(hi2c, 0);
	// 1. Verify WHO_AM_I register .
	ICM20948_READ(hi2c, ICM20948_ADDR_AD0_LOW, WHO_AM_I, &rx_data, 1);
	// Correct WHO_AM_I value is 0xEA
	if (rx_data != 0xEA)
		return HAL_ERROR;
	// 2. Perform a soft reset on the device
	tx_data = DEVICE_RESET << 7;
	ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, PWR_MGMT_1, &tx_data, 1);
	HAL_Delay(100);
	// 3. Wake up the device and set the clock source to PLL
	tx_data = CLKSEL;
	ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, PWR_MGMT_1, &tx_data, 1);
	tx_data = 0x00;
	ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, LP_CONFIG, &tx_data, 1);
	//- Turn off bypass, ensure ICM20948 become master
	tx_data = 0x00;
	ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, INT_PIN_CFG, &tx_data, 1);
	//- ENABLE INTERUPT WHEN DATA READY
	tx_data = RAW_DATA_0_RDY_EN;
	ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, INT_ENABLE_1, &tx_data, 1);
	tx_data = I2C_MST_EN << 5;
	ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, USER_CTRL, &tx_data, 1);
	// BANK 3
	ICM20948_SET_BANK(hi2c, 3);
	//- Turn on I2C Master, clock 400Hz
	tx_data = I2C_MST_CLK;
	ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, I2C_MST_CTRL, &tx_data, 1);

	ICM20948_WRITE_AK09916(hi2c, AK09916_CONTROL_3, 0x01, 1);
	HAL_Delay(10);
	ICM20948_WRITE_AK09916(hi2c, AK09916_CONTROL_2, 0x08, 1);
	ICM20948_READ_AK09916(hi2c, AK09916_HXL, 8);
//	AK09916_init(hi2c);
	HAL_Delay(100);


	// BANK 2
	ICM20948_SET_BANK(hi2c, 2);
	ICM20948_SET_GYRO_SCALE(hi2c, GYRO_SCALE_2000_DPS);
	ICM20948_SET_ACCEL_SCALE(hi2c, ACCEL_SCALE_2g);
	ICM20948_SET_GYRO_DLPF(hi2c, GYRO_DLPFCFG_4);
	ICM20948_SET_ACCEL_DLPF(hi2c, ACCEL_DLPFCFG_4);
	// BANK 0
	ICM20948_SET_BANK(hi2c, 0);
	return HAL_OK;
}
static HAL_StatusTypeDef ICM20948_READ_AK09916(I2C_HandleTypeDef *hi2c,
        uint8_t reg, uint8_t length)
{
    uint8_t tx_data;
    ICM20948_SET_BANK(hi2c, 3);

    tx_data = I2C_SLV0_RNW << 7 | AK09916_ADDRESS;
    ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, I2C_SLV0_ADDR, &tx_data, 1);

    tx_data = reg;
    ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, I2C_SLV0_REG, &tx_data, 1);

    tx_data = I2C_SLV0_EN << 7 | length;                  // EN | 1 byte
    ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, I2C_SLV0_CTRL, &tx_data, 1);

    HAL_Delay(10);
    return HAL_OK;
}
static HAL_StatusTypeDef ICM20948_WRITE_AK09916(I2C_HandleTypeDef *hi2c,
        uint8_t reg, uint8_t value, uint8_t length)
{
    uint8_t tx_data;
    ICM20948_SET_BANK(hi2c, 3);

    tx_data = AK09916_ADDRESS;       // bit7=0 → WRITE
    ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, I2C_SLV0_ADDR, &tx_data, 1);

    tx_data = reg;
    ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, I2C_SLV0_REG, &tx_data, 1);

    tx_data = value;
    ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, I2C_SLV0_DO, &tx_data, 1);

    tx_data = I2C_SLV0_EN << 7 | length;                  // EN | 1 byte
    ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, I2C_SLV0_CTRL, &tx_data, 1);

    HAL_Delay(10);
    return HAL_OK;
}
HAL_StatusTypeDef ICM20948_READ_ACCEL_GYRO(I2C_HandleTypeDef *hi2c,
		ICM20948_ACCEL_DATA *acc, ICM20948_GYRO_DATA *gyro) {
	uint8_t bytes[12];

	// Set bank 0 to access sensor data registers
	ICM20948_READ(hi2c, ICM20948_ADDR_AD0_LOW, ACCEL_XOUT_H, bytes, 12);

	// Combine high and low bytes into signed 16-bit integers
	acc->ACC_DATA[0] = ((float)((int16_t) ((bytes[0] << 8) | bytes[1]))/ accel_scale)*g - acc->x_offset;
	acc->ACC_DATA[1] = ((float)((int16_t) ((bytes[2] << 8) | bytes[3]))/ accel_scale)*g - acc->y_offset;
	acc->ACC_DATA[2] = ((float)((int16_t) ((bytes[4] << 8) | bytes[5]))/ accel_scale)*g - acc->z_offset;

	gyro->GYRO_DATA[0] = ((float)((int16_t) ((bytes[6] << 8) | bytes[7]))/ gyro_scale) - gyro->x_offset;
	gyro->GYRO_DATA[1] = ((float)((int16_t) ((bytes[8] << 8) | bytes[9]))/ gyro_scale) - gyro->y_offset;
	gyro->GYRO_DATA[2] = ((float)((int16_t) ((bytes[10] << 8) | bytes[11]))/ gyro_scale) - gyro->z_offset;

	return HAL_OK;
}
HAL_StatusTypeDef ICM20948_READ_ACCEL_GYRO_MAG_DMA(I2C_HandleTypeDef *hi2c,
		ICM20948_ACCEL_DATA *acc, ICM20948_GYRO_DATA *gyro, AK09916_DATA *mag) {

	// Set bank 0 to access sensor data registers
	gyro_DMA = gyro;
	acc_DMA = acc;
	mag_DMA = mag;
	dma_done = 0;
	if(HAL_I2C_Mem_Read_DMA(hi2c, ICM20948_ADDR_AD0_LOW, ACCEL_XOUT_H, 1, bytes_DMA, 20) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}

HAL_StatusTypeDef ICM20948_calirate_accel_gyro(I2C_HandleTypeDef *hi2c,
		ICM20948_GYRO_DATA *gyro, ICM20948_ACCEL_DATA *acc)
{
//	if (ICM20948_SET_BANK(hi2c, 2) != HAL_OK) {
//		return HAL_ERROR;
//	}
	ICM20948_GYRO_DATA temp_gyro = { 0 };   // MUST INIT TO ZERO
	ICM20948_GYRO_DATA sample_gyro;
	ICM20948_ACCEL_DATA temp_acc = { 0 };   // MUST INIT TO ZERO
	ICM20948_ACCEL_DATA sample_acc;

	for (int i = 0; i < 2000; i++) {
		if (ICM20948_READ_ACCEL_GYRO(hi2c, &sample_acc, &sample_gyro) != HAL_OK)
			return HAL_ERROR;

		temp_gyro.GYRO_DATA[0] += sample_gyro.GYRO_DATA[0];
		temp_gyro.GYRO_DATA[1] += sample_gyro.GYRO_DATA[1];
		temp_gyro.GYRO_DATA[2] += sample_gyro.GYRO_DATA[2];

		temp_acc.ACC_DATA[0] += sample_acc.ACC_DATA[0];
		temp_acc.ACC_DATA[1] += sample_acc.ACC_DATA[1];
		temp_acc.ACC_DATA[2] += sample_acc.ACC_DATA[2];
		HAL_Delay(2);
	}

	gyro->x_offset = temp_gyro.GYRO_DATA[0] / 2000;
	gyro->y_offset = temp_gyro.GYRO_DATA[1] / 2000;
	gyro->z_offset = temp_gyro.GYRO_DATA[2] / 2000;

	acc->x_offset = temp_acc.ACC_DATA[0] / 2000;
	acc->y_offset = temp_acc.ACC_DATA[1] / 2000;
	acc->z_offset = temp_acc.ACC_DATA[2] / 2000 - g;
	return HAL_OK;
}
HAL_StatusTypeDef ICM20948_SET_GYRO_SCALE(I2C_HandleTypeDef *hi2c,
		Gyro_scale_t scale) {
	uint8_t data = 0;
	ICM20948_READ(hi2c, ICM20948_ADDR_AD0_LOW, GYRO_CONFIG_1, &data, 1);
	data &= ~(0x06); // Reset bit 2:1
	data |= (scale << 1); //Shift to bit 1 postion
	ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, GYRO_CONFIG_1, &data, 1);
	switch (scale) {
	case GYRO_SCALE_250_DPS:
		gyro_scale = 131.0;
		break;
	case GYRO_SCALE_500_DPS:
		gyro_scale = 65.5;
		break;
	case GYRO_SCALE_1000_DPS:
		gyro_scale = 32.8;
		break;
	case GYRO_SCALE_2000_DPS:
		gyro_scale = 16.4;
		break;
	}

	return HAL_OK;
}

HAL_StatusTypeDef ICM20948_SET_ACCEL_SCALE(I2C_HandleTypeDef *hi2c,
		Accel_scale_t scale) {
	uint8_t data = 0;
	ICM20948_READ(hi2c, ICM20948_ADDR_AD0_LOW, ACCEL_CONFIG, &data, 1);
	data &= ~(0x06); // Reset bit 2:1
	data |= (scale << 1); //Shift to bit 1 postion
	ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, ACCEL_CONFIG, &data, 1);
	switch (scale) {
	case ACCEL_SCALE_2g:
		accel_scale = 16384.0;
		break;
	case ACCEL_SCALE_4g:
		accel_scale = 8192.0;
		break;
	case ACCEL_SCALE_8g:
		accel_scale = 4096.0;
		break;
	case ACCEL_SCALE_16g:
		accel_scale = 2048.0;
		break;
	}

	return HAL_OK;
}

HAL_StatusTypeDef ICM20948_SET_GYRO_DLPF(I2C_HandleTypeDef *hi2c,
		Gyro_DLPFCFG_t config) {

	uint8_t data = 0;
	ICM20948_READ(hi2c, ICM20948_ADDR_AD0_LOW, GYRO_CONFIG_1, &data, 1);
	data &= ~(0x38); // Reset bit 5:3
	data |= (config << 3); //Shift to bit 3c postion
	ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, GYRO_CONFIG_1, &data, 1);
	return HAL_OK;
}

HAL_StatusTypeDef ICM20948_SET_ACCEL_DLPF(I2C_HandleTypeDef *hi2c,
		Accel_DLPFCFG_t config) {

	uint8_t data = 0;
	ICM20948_READ(hi2c, ICM20948_ADDR_AD0_LOW, ACCEL_CONFIG, &data, 1);
	data &= ~(0x38); // Reset bit 5:3
	data |= (config << 3); //Shift to bit 3 postion
	ICM20948_WRITE(hi2c, ICM20948_ADDR_AD0_LOW, ACCEL_CONFIG, &data, 1);
	return HAL_OK;
}

//static HAL_StatusTypeDef AK09916_init(I2C_HandleTypeDef *hi2c)
//{
//	uint8_t data = 0;
//	// 1. Verify WHO_AM_I register
//	if (HAL_I2C_Mem_Read(hi2c, AK09916_ADDRESS, AK09916_WHO_AM_I, 1, &data, 1,
//			100) != HAL_OK)
//		return HAL_ERROR;
//	// Correct WHO_AM_I value is 0x09
//	if (data != 0x09)
//		return HAL_ERROR;
//
//	// 2. Soft reset for AK09916
//	data = 0x01;
//	if (HAL_I2C_Mem_Write(hi2c, AK09916_ADDRESS, AK09916_CONTROL_3, 1, &data, 1,
//			100) != HAL_OK)
//		return HAL_ERROR;
//	HAL_Delay(10);
//
//	// 3. Continuous mode 4: 100 Hz
//	data = 0x08;
//	if (HAL_I2C_Mem_Write(hi2c, AK09916_ADDRESS, AK09916_CONTROL_2, 1, &data, 1,
//			100) != HAL_OK)
//		return HAL_ERROR;
//	return HAL_OK;
//}

HAL_StatusTypeDef ICM20948_READ_MAG(I2C_HandleTypeDef *hi2c,
		AK09916_DATA *mag) {
	uint8_t mag_bytes[8] = { 0 };
	ICM20948_READ(hi2c, ICM20948_ADDR_AD0_LOW, EXT_SLV_SENS_DATA_00, mag_bytes, 8);
	if (mag_bytes[7] & 0x08)
		return HAL_ERROR;
	mag->raw_x = ((mag_bytes[1] << 8) | mag_bytes[0]);
	mag->raw_y = ((mag_bytes[3] << 8) | mag_bytes[2]);
	mag->raw_z = ((mag_bytes[5] << 8) | mag_bytes[4]);

	mag->MAG_DATA[0] = (mag->raw_x - mag->xOffset) * mag->xScale;
	mag->MAG_DATA[1] = (mag->raw_y - mag->yOffset) * mag->yScale;
	mag->MAG_DATA[2] = (mag->raw_z - mag->zOffset) * mag->zScale;
	return HAL_OK;
}

HAL_StatusTypeDef ak09916_calibrate(I2C_HandleTypeDef *hi2c,
		AK09916_DATA *mag) {
//	mag->xMax = -10000;
//	mag->xMin = 10000;
//	mag->yMax = -10000;
//	mag->yMin = 10000;
//	mag->zMax = -10000;
//	mag->zMin = 10000;
	for (int i = 0; i < 40; i++) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		if (ICM20948_READ_MAG(hi2c, mag) != HAL_OK)
			return HAL_ERROR;
		mag->xMax = (mag->raw_x > mag->xMax) ? mag->raw_x : mag->xMax;
		mag->yMax = (mag->raw_y > mag->yMax) ? mag->raw_y : mag->yMax;
		mag->zMax = (mag->raw_z > mag->zMax) ? mag->raw_z : mag->zMax;

		mag->xMin = (mag->raw_x < mag->xMin) ? mag->raw_x : mag->xMin;
		mag->yMin = (mag->raw_y < mag->yMin) ? mag->raw_y : mag->yMin;
		mag->zMin = (mag->raw_z < mag->zMin) ? mag->raw_z : mag->zMin;

		mag->xOffset = (mag->xMax + mag->xMin) / 2.0;
		mag->yOffset = (mag->yMax + mag->yMin) / 2.0;
		mag->zOffset = (mag->zMax + mag->zMin) / 2.0;

		mag->xScale = 2.0 / ((mag->xMax - mag->xMin));
		mag->yScale = 2.0 / ((mag->yMax - mag->yMin));
		mag->zScale = 2.0 / ((mag->zMax - mag->zMin));

		HAL_Delay(100);

	}

	return HAL_OK;
}
