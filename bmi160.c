/*
 * Created by: César Villarreal & Luís Fernando
 */

#include "bmi160.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

static i2c_master_handle_t g_master_handle;
static i2c_master_transfer_t g_master_transfer;
static i2c_master_config_t g_master_config;

#define DEBUG 1

static SemaphoreHandle_t g_i2c0;

static unsigned int count;

boolean_t soft_reset = 0;

static void i2c_master_callback(
		I2C_Type *base,
		i2c_master_handle_t *handle,
        status_t status,
		void * userData)
{

	printf("give\n\r");

//	do
//	{
//		I2C_MasterTransferGetCount(I2C0, &g_master_handle, &count);
//	}while(count != 0);

	xSemaphoreGive(g_i2c0);
	//xSemaphoreGive(g_mutex2);
}

void accelerometer_task()
{

	for(;;)
	{
		//bmi160_get_data();
//		if( != kStatus_I2C_Busy)
//		{
//			xSemaphoreGive(g_mutex1);
//		}
	}
}

void bmi_160_init(struct bmi160_t* device)
{
	uint8_t read_id;

	if(TRUE != soft_reset)
	{

		/* Set I2C's master with the default configuration */
		I2C_MasterGetDefaultConfig(&g_master_config);
		/* Change i2c baud rate */
		g_master_config.baudRate_Bps = BMI160_BAUDRATE;
		/* Initialize master node */
		I2C_MasterInit(I2C0, &g_master_config, CLOCK_GetFreq(kCLOCK_BusClk));
		/* Create handler */
		I2C_MasterTransferCreateHandle(I2C0, &g_master_handle, i2c_master_callback, NULL);

	#ifdef DEBUG
		printf("reading chip's id:\n\r");
	#endif

		while(read_id != BMI160_CHIP_ID)
		{
			read_id = bmi160_read(BMI160_READ_CHIP_ID);

	#ifdef DEBUG
			printf("id = %x \n\r", read_id);
			if(read_id == BMI160_CHIP_ID)
			{
				printf("found chip's ID value\n\r");
				bmi160_get_device_config(device);
			}
			else
			{
				printf("reading chip id register..\n\r");
			}
	#endif
		}

		device->chip_id = read_id;

		bmi160_normal_mode_config(device);

		bmi160_soft_reset(device);
	}
	else
	{

	}

//	bmi160_get_device_config(device);
}

/*!
 * @brief This API
 */
void bmi160_normal_mode_config(struct bmi160_t* device)
{
	uint8_t acc_conf;

	/* Deactivate under-sampling */
	device->acc_config.acc_us = 0x00;
	/* Select the Output data rate, range of accelerometer sensor */
	device->acc_config.acc_odr = BMI160_ACCEL_ODR_200HZ;
	/* Set accelerometers g range */
	device->acc_config.acc_range = BMI160_ACCEL_RANGE_2G;
	/* Set bandwidth */
	device->acc_config.acc_bw = BMI160_ACCEL_BW_NORMAL_AVG4;
	/* Select the power mode of accelerometer sensor */
	device->acc_config.acc_pwr = BMI160_ACCEL_NORMAL_MODE;

	acc_conf = (device->acc_config.acc_us | device->acc_config.acc_bw | device->acc_config.acc_odr);

	bmi160_write(BMI160_ACC_CONF, acc_conf);

	bmi160_write(BMI160_ACC_RANGE, device->acc_config.acc_range);

	bmi160_write(BMI160_COMMAND_REG_ADDR, device->acc_config.acc_pwr);
}
/************************************************************************************/
/*!
 * @brief This API
 */
void bmi160_write(uint8_t reg, uint8_t data)
{

	g_master_transfer.slaveAddress = BMI160_I2C_ADDR;
	g_master_transfer.direction = kI2C_Write;
	g_master_transfer.subaddress = reg;
	g_master_transfer.subaddressSize = BMI160_SUBADDRESS_SIZE;
	g_master_transfer.data = &data;
	g_master_transfer.dataSize = BMI160_DATA_SIZE;
	g_master_transfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_master_handle, &g_master_transfer);
	//xSemaphoreTake(g_i2c0,portMAX_DELAY);

}

/*!
 * @brief This API
 */
uint8_t bmi160_read(uint8_t reg)
{
	uint8_t data_temp;

	g_master_transfer.slaveAddress = BMI160_I2C_ADDR;
	g_master_transfer.direction = kI2C_Read;
	g_master_transfer.subaddress = reg;
	g_master_transfer.subaddressSize = BMI160_SUBADDRESS_SIZE;
	g_master_transfer.data = &data_temp;
	g_master_transfer.dataSize = BMI160_DATA_SIZE;
	g_master_transfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferNonBlocking(I2C0, &g_master_handle, &g_master_transfer);
	xSemaphoreTake(g_i2c0,portMAX_DELAY);

	return data_temp;
}

/*!
 * @brief This API
 */
void bmi160_get_device_config(struct bmi160_t* device)
{
	uint8_t read_val;

	read_val = bmi160_read(BMI160_ACC_CONF);

#ifdef DEBUG
	printf("undersampling (7) = %x\n\r", (read_val >> 7));
	printf("read_val  = %x\n\r", (read_val));
#endif
}

void bmi160_soft_reset(struct bmi160_t* device)
{
	bmi160_write(BMI160_COMMAND_REG_ADDR, BMI160_SOFT_RESET_CMD);

	soft_reset = TRUE;
}
