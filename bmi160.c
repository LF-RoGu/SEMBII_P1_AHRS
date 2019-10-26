/*
 * bmi160.c
 *
 *  Created on: Oct 25, 2019
 *      Author: LuisFernando
 */

#include "bmi160.h"

uint8_t g_MasterCompletionFlag = false;

/* Buffer to read acc & gyr*/
uint8_t g_read_gyr[6] = {0};
uint8_t g_read_acc[6] = {0};

static i2c_master_handle_t g_master_handle;
static i2c_master_transfer_t g_master_transfer;
static i2c_master_config_t g_master_config;

static void i2c_master_callback(
		I2C_Type *base,
		i2c_master_handle_t *handle,
        status_t status,
		void * userData)
{
	//PRINTF("SUCCESS\n");
	if (status == kStatus_Success)
	{
		g_MasterCompletionFlag = true;
	}
}

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
void gpio_i2c_config(void)
{
	I2C_MasterGetDefaultConfig(&g_master_config);

	g_master_config.baudRate_Bps = BMI160_BAUDRATE;

	I2C_MasterInit(I2C0, &g_master_config, CLOCK_GetFreq(kCLOCK_BusClk));

	I2C_MasterTransferCreateHandle(I2C0, &g_master_handle,i2c_master_callback, NULL);
}
/*!
 * @brief This API
 */
void bmi160_write(uint16_t reg,uint8_t data)
{
	g_master_transfer.slaveAddress = BMI160_I2C_ADDR;
	g_master_transfer.direction = kI2C_Write;
	g_master_transfer.subaddress = reg;
	g_master_transfer.subaddressSize = BMI160_SUBADDRESS_SIZE;
	g_master_transfer.data = &data;
	g_master_transfer.dataSize = BMI160_DATA_SIZE;
	g_master_transfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_master_handle,&g_master_transfer);

	while (!g_MasterCompletionFlag)
	{
	}
	g_MasterCompletionFlag = false;

	I2Cwritedelay();
}
/*!
 * @brief This API
 */
uint8_t bmi160_read(uint16_t reg)
{
	uint8_t data_temp;

	g_master_transfer.slaveAddress = BMI160_I2C_ADDR;
	g_master_transfer.direction = kI2C_Read;
	g_master_transfer.subaddress = reg;
	g_master_transfer.subaddressSize = BMI160_SUBADDRESS_SIZE;
	g_master_transfer.data = &data_temp;
	g_master_transfer.dataSize = BMI160_DATA_SIZE;
	g_master_transfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_master_handle,&g_master_transfer);

	while (!g_MasterCompletionFlag)
	{
	}
	g_MasterCompletionFlag = false;

	I2Cwritedelay();

	return data_temp;
}
/*!
 * @brief This API
 */
void bmi160_read_acc(void)
{
	g_read_acc[0] = bmi160_read(BMI160_READ_ACC_X_H);
	g_read_acc[1] = bmi160_read(BMI160_READ_ACC_X_L);
	g_read_acc[2] = bmi160_read(BMI160_READ_ACC_Y_H);
	g_read_acc[3] = bmi160_read(BMI160_READ_ACC_Y_L);
	g_read_acc[4] = bmi160_read(BMI160_READ_ACC_Z_H);
	g_read_acc[5] = bmi160_read(BMI160_READ_ACC_Z_L);
}
/*!
 * @brief This API
 */
void bmi160_read_gyr(void)
{
	g_read_gyr[0] = bmi160_read(BMI160_READ_GYR_X_H);
	g_read_gyr[1] = bmi160_read(BMI160_READ_GYR_X_L);
	g_read_gyr[2] = bmi160_read(BMI160_READ_GYR_Y_H);
	g_read_gyr[3] = bmi160_read(BMI160_READ_GYR_Y_L);
	g_read_gyr[4] = bmi160_read(BMI160_READ_GYR_Z_H);
	g_read_gyr[5] = bmi160_read(BMI160_READ_GYR_Z_L);
}
/*!
 * @brief This API
 */
void bmi160_offset_gyr(uint8_t accel,uint8_t off)
{
	switch(accel)
	{
	case X_ENUM:
		bmi160_write(BMI160_OFFSET_GYR_X,off);
		break;
	case Y_ENUM:
		bmi160_write(BMI160_OFFSET_GYR_Y,off);
		break;
	case Z_ENUM:
		bmi160_write(BMI160_OFFSET_GYR_Z,off);
		break;
	default:
		break;
	}
}
/*!
 * @brief This API
 */
void bmi160_offset_acc(uint8_t accel,uint8_t off)
{
	switch(accel)
	{
	case X_ENUM:
		bmi160_write(BMI160_OFFSET_ACC_X,off);
		break;
	case Y_ENUM:
		bmi160_write(BMI160_OFFSET_ACC_Y,off);
		break;
	case Z_ENUM:
		bmi160_write(BMI160_OFFSET_ACC_Z,off);
		break;
	default:
		break;
	}
}
void I2Cwritedelay(void)
{
	for(uint32_t i = 120000000;i==0;i--)
	{
	}

}
