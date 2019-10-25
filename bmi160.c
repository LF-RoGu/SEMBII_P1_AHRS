/*
 * bmi160.c
 *
 *  Created on: Oct 23, 2019
 *      Author: LuisFernando
 */
#include "bmi160.h"

extern SemaphoreHandle_t g_device_mutex;
i2c_master_handle_t g_master_handle;
i2c_rtos_handle_t g_rtos_handle;
i2c_master_config_t g_master_config_t;
struct bmi160_device g_device;

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
void gpio_i2c_config(void)
{
	/*** GPIO CONFIGURATION **/
	gpio_pin_control_register_t i2c_config = GPIO_MUX2 | GPIO_PS;

	/** Activate PORT B clock gating **/
	GPIO_clock_gating(GPIO_B);

	/*Set the configuration for i2c_config_Tx*/
	GPIO_pin_control_register(GPIO_B, bit_2, &i2c_config);
	/*Set the configuration for i2c_config_Rx*/
	GPIO_pin_control_register(GPIO_B, bit_3, &i2c_config);
	/*Set the config fot the pin that it will send the i2c protocol*/
	GPIO_data_direction_pin(GPIO_B,GPIO_OUTPUT, bit_2);
	/*Set the config fot the pin that it will receive the i2c protocol*/
	GPIO_data_direction_pin(GPIO_B,GPIO_INPUT, bit_3);
}

void bmi160_rtos_init(void)
{
	uint32_t src_clk = CLOCK_GetFreq(kCLOCK_BusClk);

	g_master_config_t.baudRate_Bps = BMI160_I2C_BAUDRATE;

	g_master_config_t.enableMaster = true;

	I2C_RTOS_Init(&g_rtos_handle,I2C0, &g_master_config_t, src_clk);
}

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
void bmi160_write(uint8_t data, uint8_t address)
{
	i2c_master_transfer_t master_transfer;

	g_device.data = data;
	g_device.address = address;

	master_transfer.slaveAddress = BMI160_I2C_ADDR;
	master_transfer.direction = kI2C_Write;
	master_transfer.subaddress = g_device.address;
	master_transfer.subaddressSize = BMI160_I2C_SUBADDR_SIZE;
	master_transfer.data = &g_device.data;
	master_transfer.dataSize = BMI160_I2C_DATA_SIZE;
	master_transfer.flags = kI2C_TransferDefaultFlag;

	I2C_RTOS_Transfer(&g_rtos_handle, &master_transfer);

}

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
uint8_t bmi160_read(uint8_t address);
