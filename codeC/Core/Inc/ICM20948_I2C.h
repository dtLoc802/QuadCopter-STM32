#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_

#include "stm32f4xx_hal.h"


#define g 9.787
// Define the two possible I2C addresses for the ICM-20948
#define ICM20948_ADDR_AD0_LOW  (0x68 << 1) // 0xD0
#define ICM20948_ADDR_AD0_HIGH (0x69 << 1) // 0xD2

// ICM-20948 registers used for this example
// -BANK 0- //
#define WHO_AM_I          		0x00
#define USER_CTRL   			0x03
#define LP_CONFIG         		0x05
#define PWR_MGMT_1    	  		0x06
#define INT_ENABLE_1      		0X11
#define ACCEL_XOUT_H  	  		0x2D
#define GYRO_XOUT_H   	  		0x33
#define REG_BANK_SEL  	 		0x7F
#define INT_PIN_CFG   	  		0x0F
#define EXT_SLV_SENS_DATA_00	0x3B

// -BANK 2- //
#define GYRO_CONFIG_1 	  		0x01
#define ACCEL_CONFIG  	  		0x14
#define REG_XG_OFFS_USRH  		0x03
#define REG_XG_OFFS_USRL  		0x04
#define REG_YG_OFFS_USRH  		0x05
#define REG_YG_OFFS_USRL  		0x06
#define REG_ZG_OFFS_USRH  		0x07
#define REG_ZG_OFFS_USRL  		0x08
// -BANK 3- //
#define I2C_MST_CTRL 	  		0x01
#define I2C_SLV0_ADDR	  		0x03
#define I2C_SLV0_REG	  		0x04
#define I2C_SLV0_CTRL	  		0x05
#define I2C_SLV0_DO         	0x06
// AK09916
#define AK09916_ADDRESS		   0x0C
#define AK09916_WHO_AM_I       0x01
#define AK09916_HXL            0x11
#define AK09916_HXH            0x12
#define AK09916_ST2            0x18
#define AK09916_CONTROL_2	   0x31
#define AK09916_CONTROL_3	   0x32

// FUNCTION
#define DEVICE_RESET 1
#define CLKSEL 1
#define RAW_DATA_0_RDY_EN 1
#define I2C_MST_CLK 7
#define I2C_SLV0_RNW 1
#define I2C_SLV0_EN 1
#define I2C_SLV0_LENG 8
#define I2C_MST_EN 1
typedef struct {
	int16_t raw_x;
	int16_t raw_y;
	int16_t raw_z;

	float ACC_DATA[3];

	float x_offset;
	float y_offset;
	float z_offset;
} ICM20948_ACCEL_DATA;

// Structure to hold 3-axis sensor data in deg/s
typedef struct {

	int16_t raw_x;
	int16_t raw_y;
	int16_t raw_z;

	float GYRO_DATA[3];

	float x_offset;
	float y_offset;
	float z_offset;
} ICM20948_GYRO_DATA;

typedef struct {

	int16_t raw_x;
	int16_t raw_y;
	int16_t raw_z;

	float MAG_DATA[3];

	int16_t xMax, xMin;
	int16_t yMax, yMin;
	int16_t zMax, zMin;

	float xOffset, yOffset, zOffset;
	float xScale, yScale, zScale;

} AK09916_DATA;

typedef struct {
	float gyro_filter[3], gyro_filter_linear[3];
	float acc_filter[3], acc_filter_linear[3];
	float angle_deg[3], angle_rad[3];
	float yaw;
} ICM20948_FILTER;


typedef enum {
	GYRO_SCALE_250_DPS = 0,
	GYRO_SCALE_500_DPS,
	GYRO_SCALE_1000_DPS,
	GYRO_SCALE_2000_DPS
} Gyro_scale_t;

typedef enum {
	ACCEL_SCALE_2g = 0, ACCEL_SCALE_4g, ACCEL_SCALE_8g, ACCEL_SCALE_16g
} Accel_scale_t;

typedef enum {
	GYRO_DLPFCFG_0 = 0,
	GYRO_DLPFCFG_1,
	GYRO_DLPFCFG_2,
	GYRO_DLPFCFG_3,
	GYRO_DLPFCFG_4,
	GYRO_DLPFCFG_5,
	GYRO_DLPFCFG_6,
	GYRO_DLPFCFG_7
} Gyro_DLPFCFG_t;

typedef enum {
	ACCEL_DLPFCFG_0 = 0,
	ACCEL_DLPFCFG_1,
	ACCEL_DLPFCFG_2,
	ACCEL_DLPFCFG_3,
	ACCEL_DLPFCFG_4,
	ACCEL_DLPFCFG_5,
	ACCEL_DLPFCFG_6,
	ACCEL_DLPFCFG_7
} Accel_DLPFCFG_t;
extern volatile uint8_t dma_done;
extern volatile uint8_t data_ready;
HAL_StatusTypeDef ICM20948_I2C_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef ICM20948_READ_ACCEL_GYRO(I2C_HandleTypeDef *hi2c,
		ICM20948_ACCEL_DATA *acc, ICM20948_GYRO_DATA *gyro);

HAL_StatusTypeDef ICM20948_READ_ACCEL_GYRO_MAG_DMA(I2C_HandleTypeDef *hi2c,
		ICM20948_ACCEL_DATA *acc, ICM20948_GYRO_DATA *gyro, AK09916_DATA *mag);

HAL_StatusTypeDef ICM20948_READ_MAG(I2C_HandleTypeDef *hi2c,
		AK09916_DATA *mag);

HAL_StatusTypeDef ICM20948_calirate_accel_gyro(I2C_HandleTypeDef *hi2c,
		ICM20948_GYRO_DATA *gyro, ICM20948_ACCEL_DATA *acc);

HAL_StatusTypeDef ICM20948_SET_GYRO_SCALE(I2C_HandleTypeDef *hi2c,
		Gyro_scale_t scale);

HAL_StatusTypeDef ICM20948_SET_ACCEL_SCALE(I2C_HandleTypeDef *hi2c,
		Accel_scale_t scale);

HAL_StatusTypeDef ICM20948_SET_GYRO_DLPF(I2C_HandleTypeDef *hi2c,
		Gyro_DLPFCFG_t config);

HAL_StatusTypeDef ICM20948_SET_ACCEL_DLPF(I2C_HandleTypeDef *hi2c,
		Accel_DLPFCFG_t config);

HAL_StatusTypeDef ak09916_calibrate(I2C_HandleTypeDef *hi2c,
		AK09916_DATA *mag);
#endif /* INC_ICM20948_H_ */
