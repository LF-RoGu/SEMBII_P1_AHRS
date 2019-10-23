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

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
void i2c_init(void)
{
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_I2c0);

	port_pin_config_t config_i2c_t =
	{
			kPORT_PullUp,
			kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable,
		    kPORT_OpenDrainEnable,
			kPORT_LowDriveStrength,
			kPORT_MuxAlt2,
		    kPORT_UnlockRegister,
	};

	PORT_SetPinConfig(PORTB, 2, &config_i2c_t);
	PORT_SetPinConfig(PORTB, 3, &config_i2c_t);

	i2c_master_config_t master_config_t;

	I2C_MasterGetDefaultConfig(&master_config_t);
	master_config_t.baudRate_Bps = BMI160_I2C_BAUDRATE;
	I2C_MasterInit(I2C0, &master_config_t, CLOCK_GetFreq(kCLOCK_BusClk));
}

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
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

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
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
