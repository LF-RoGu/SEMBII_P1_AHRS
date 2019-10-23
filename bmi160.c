/*
 * bmi160.c
 *
 *  Created on: Oct 23, 2019
 *      Author: LuisFernando
 */
#include "bmi160.h"

static SemaphoreHandle_t g_device_mutex;
i2c_master_handle_t g_master_handle;
struct bmi160_device g_device;

static void i2c_master_callback(
		I2C_Type *base,
		i2c_master_handle_t *handle,
        status_t status,
		void * userData)
{
	//PRINTF("SUCCESS\n");
	if (status == kStatus_Success)
	{
		g_device.master_transfer_flag = true;

	}
}

void bmi160_write(uint8_t data, uint8_t address)
{
	i2c_master_transfer_t master_transfer;

	I2C_MasterTransferCreateHandle(I2C0, &g_master_handle,i2c_master_callback, NULL);

	g_device.data = data;
	g_device.address = address;

	xSemaphoreTake(g_device_mutex,portMAX_DELAY);


	master_transfer.slaveAddress = BMI160_I2C_ADDR;
	master_transfer.direction = kI2C_Write;
	master_transfer.subaddress = g_device.address;
	master_transfer.subaddressSize = BMI160_I2C_SUBADDR_SIZE;
	master_transfer.data = &g_device.data;
	master_transfer.dataSize = BMI160_I2C_DATA_SIZE;
	master_transfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_master_handle,&master_transfer);

	while (!g_device.master_transfer_flag)
	{
		/*
		 * Do Nothing
		 */
	}
	g_device.master_transfer_flag = false;

	xSemaphoreGive(g_device_mutex);
}

uint8_t bmi160_read(uint8_t address)
{
	i2c_master_transfer_t master_transfer;

	I2C_MasterTransferCreateHandle(I2C0, &g_master_handle,i2c_master_callback, NULL);

	g_device.address = address;

	xSemaphoreTake(g_device_mutex,portMAX_DELAY);


	master_transfer.slaveAddress = BMI160_I2C_ADDR;
	master_transfer.direction = kI2C_Read;
	master_transfer.subaddress = g_device.address;
	master_transfer.subaddressSize = BMI160_I2C_SUBADDR_SIZE;
	master_transfer.data = &g_device.data;
	master_transfer.dataSize = BMI160_I2C_DATA_SIZE;
	master_transfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_master_handle,&master_transfer);

	while (!g_device.master_transfer_flag)
	{
		/*
		 * Do Nothing
		 */
	}
	g_device.master_transfer_flag = false;

	xSemaphoreGive(g_device_mutex);

	return g_device.data;
}
