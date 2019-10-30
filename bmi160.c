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

uint16_t g_data_axis_acc[3] = {0};
uint16_t g_data_axis_gyr[3] = {0};

static i2c_master_handle_t g_master_handle;
static i2c_master_transfer_t g_master_transfer;
static i2c_master_config_t g_master_config;

struct bmi160_device sensor;
struct comm_msg_acc_t acc_device;
struct comm_msg_acc_t gyr_device;

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
void bmi160_normal_mode_config(void)
{
	/*
	 * ACC
	 */
	/* Select the Output data rate, range of accelerometer sensor */
	sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
	sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4; //en teoria es un 0x20
	/* Select the power mode of accelerometer sensor */
	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
	/*
	 * GYRO
	 */
	/* Select the Output data rate, range of Gyroscope sensor */
	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	/* Select the power mode of Gyroscope sensor */
	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	/* Send the Data to the sensor*/

	/* SET POWER MODE*/
	/* ACC*/
	bmi160_write(BMI160_COMMAND_REG_ADDR,sensor.accel_cfg.power);
	/* GYR*/
	bmi160_write(BMI160_COMMAND_REG_ADDR,sensor.gyro_cfg.power);

	/* ACC*/
	bmi160_write(BMI160_ACC_CONF,(sensor.accel_cfg.bw | sensor.accel_cfg.odr));
	bmi160_write(BMI160_ACC_RANGE,sensor.accel_cfg.range);

	/* GYR*/
	bmi160_write(BMI160_GYR_CONF,(sensor.gyro_cfg.bw | sensor.gyro_cfg.odr));
	bmi160_write(BMI160_GYR_RANGE,sensor.gyro_cfg.range);
}
/************************************************************************************/
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
		/*
		 * Do nothing
		 */
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
	uint8_t index = 0;
	uint8_t data_temp;

	uint16_t reg_address = BMI160_READ_ACC_X_L;

	for(index = X_LOW; index <= Z_HIGH; index++)
	{
		data_temp = bmi160_read(reg_address);
		g_read_acc[index] = data_temp;

		reg_address++;
	}

	/**/
	data_axis_acc();
}
/*!
 * @brief This API
 */
void bmi160_read_gyr(void)
{
	uint8_t index = 0;
	uint8_t data_temp;

	uint16_t reg_address = BMI160_READ_GYR_X_L;

	for(index = X_LOW; index <= Z_HIGH; index++)
	{
		data_temp = bmi160_read(reg_address);
		g_read_gyr[index] = data_temp;

		reg_address++;
	}

	/**/
	data_axis_gyr();
}
/*!
 * @brief This API
 */
void data_axis_acc(void)
{
	uint16_t data_temp;

	/* Data axis X*/
	data_temp = ((uint16_t) g_read_acc[X_LOW] << 8) | g_read_acc[X_HIGH];

	g_data_axis_acc[X_REG] = data_temp;
	/* Data axis Y*/
	data_temp = ((uint16_t)g_read_acc[Y_LOW] << 8) | g_read_acc[Y_HIGH];

	g_data_axis_acc[Y_REG] = data_temp;
	/* Data axis Z*/
	data_temp = ((uint16_t)g_read_acc[Z_LOW] << 8) | g_read_acc[Z_HIGH];

	g_data_axis_acc[Z_REG] = data_temp;


	/* Print data axis*/
	bmi160_print_acc_dec();
	/* Convert data axis*/
	convert_value_acc(g_data_axis_acc);
}
/*!
 * @brief This API
 */
void data_axis_gyr(void)
{
	uint16_t data_temp;

	/* Data axis X*/
	data_temp = ((uint16_t) g_read_gyr[X_LOW] << 8) | g_read_gyr[X_HIGH];

	g_data_axis_gyr[X_REG] = data_temp;
	/* Data axis Y*/
	data_temp = ((uint16_t)g_read_gyr[Y_LOW] << 8) | g_read_gyr[Y_HIGH];

	g_data_axis_gyr[Y_REG] = data_temp;
	/* Data axis Z*/
	data_temp = ((uint16_t)g_read_gyr[Z_LOW] << 8) | g_read_gyr[Z_HIGH];

	g_data_axis_gyr[Z_REG] = data_temp;


	/* Print data axis in uint32_t*/
	bmi160_print_gyr_dec();
	/* Convert data axis*/
	convert_value_gyr(g_data_axis_gyr);
}
/*!
 * @brief This API
 */
void bmi160_print_acc_dec(void)
{
	printf("acc axis: X: %d Y: %d Z: %d\n", g_data_axis_acc[X_REG],g_data_axis_acc[Y_REG],g_data_axis_acc[Z_REG]);
}

/*!
 * @brief This API
 */
void bmi160_print_acc_float(void)
{
	printf("FLOAT acc axis: X: %.2f Y: %.2f Z: %.2f\n", acc_device.x,acc_device.y,acc_device.z);
}

/*!
 * @brief This API
 */
void bmi160_print_gyr_dec(void)
{
	printf("gyr axis: X: %d Y: %d Z: %d\n", g_data_axis_gyr[X_REG],g_data_axis_gyr[Y_REG],g_data_axis_gyr[Z_REG]);
}

/*!
 * @brief This API
 */
void bmi160_print_gyr_float(void)
{
	printf("FLOAT gyr axis: X: %.2f Y: %.2f Z: %.2f\n", gyr_device.x,gyr_device.y,gyr_device.z);
}

/*!
 * @brief This API
 */
void bmi160_get_data(void)
{
	/**/
	bmi160_read_acc();
	bmi160_read_gyr();
}

/*!
 * @brief This API
 */
void convert_value_acc(uint16_t *acc_axis_data)
{
	uint8_t index;

	for(index = X_REG;index <= Z_REG;index++)
	{
		switch (index)
		{
		case X_REG:
			if(acc_axis_data[X_REG] >= (BMI160_MAX_VALUE/2))
			{
				acc_device.x = (BMI160_ACC_UINTS_RANGE_POSITIVE*(float)acc_axis_data[X_REG])/BMI160_MAX_VALUE;
			}
			else if (acc_axis_data[X_REG] < (BMI160_MAX_VALUE/2))
			{
				acc_device.x = (BMI160_ACC_UINTS_RANGE_NEGATIVE*(float)acc_axis_data[X_REG])/BMI160_MAX_VALUE;
			}
			else
			{
				/*
				 * Do Nothing
				 */
			}
			break;
		case Y_REG:
			if(acc_axis_data[Y_REG] >= (BMI160_MAX_VALUE/2))
			{
				acc_device.y = (BMI160_ACC_UINTS_RANGE_POSITIVE*(float)acc_axis_data[Y_REG])/BMI160_MAX_VALUE;
			}
			else if (acc_axis_data[Y_REG] < (BMI160_MAX_VALUE/2))
			{
				acc_device.y = (BMI160_ACC_UINTS_RANGE_NEGATIVE*(float)acc_axis_data[Y_REG])/BMI160_MAX_VALUE;
			}
			else
			{
				/*
				 * Do Nothing
				 */
			}
			break;
		case Z_REG:
			if(acc_axis_data[Z_REG] >= (BMI160_MAX_VALUE/2))
			{
				acc_device.z = (BMI160_ACC_UINTS_RANGE_POSITIVE*(float)acc_axis_data[Z_REG])/BMI160_MAX_VALUE;
			}
			else if (acc_axis_data[Z_REG] < (BMI160_MAX_VALUE/2))
			{
				acc_device.z = (BMI160_ACC_UINTS_RANGE_NEGATIVE*(float)acc_axis_data[Z_REG])/BMI160_MAX_VALUE;
			}
			else
			{
				/*
				 * Do Nothing
				 */
			}
			break;
		default:
			/*
			 * Do Nothing
			 */
			break;
		}
	}
	/* Print data axis in float*/
	bmi160_print_acc_float();
}

/*!
 * @brief This API
 */
void convert_value_gyr(uint16_t *gyr_axis_data)
{
	uint8_t index;

	for(index = X_REG;index <= Z_REG;index++)
	{
		switch (index)
		{
		case X_REG:
			if(gyr_axis_data[X_REG] >= (BMI160_MAX_VALUE/2))
			{
				gyr_device.x = (BMI160_GYR_UNITS_RANGE_POSITIVE*(float)gyr_axis_data[X_REG])/BMI160_MAX_VALUE;
			}
			else if (gyr_axis_data[X_REG] < (BMI160_MAX_VALUE/2))
			{
				gyr_device.x = (BMI160_GYR_UNITS_RANGE_NEGATIVE*(float)gyr_axis_data[X_REG])/BMI160_MAX_VALUE;
			}
			else
			{
				/*
				 * Do Nothing
				 */
			}
			break;
		case Y_REG:
			if(gyr_axis_data[Y_REG] >= (BMI160_MAX_VALUE/2))
			{
				gyr_device.y = (BMI160_GYR_UNITS_RANGE_POSITIVE*(float)gyr_axis_data[Y_REG])/BMI160_MAX_VALUE;
			}
			else if (gyr_axis_data[Y_REG] < (BMI160_MAX_VALUE/2))
			{
				gyr_device.y = (BMI160_GYR_UNITS_RANGE_NEGATIVE*(float)gyr_axis_data[Y_REG])/BMI160_MAX_VALUE;
			}
			else
			{
				/*
				 * Do Nothing
				 */
			}
			break;
		case Z_REG:
			if(gyr_axis_data[Z_REG] >= (BMI160_MAX_VALUE/2))
			{
				gyr_device.z = (BMI160_GYR_UNITS_RANGE_POSITIVE*(float)gyr_axis_data[Z_REG])/BMI160_MAX_VALUE;
			}
			else if (gyr_axis_data[Z_REG] < (BMI160_MAX_VALUE/2))
			{
				gyr_device.z = (BMI160_GYR_UNITS_RANGE_NEGATIVE*(float)gyr_axis_data[Z_REG])/BMI160_MAX_VALUE;
			}
			else
			{
				/*
				 * Do Nothing
				 */
			}
			break;
		default:
			/*
			 * Do Nothing
			 */
			break;
		}
	}
	/* Print data axis in float*/
	bmi160_print_gyr_float();
}
/*!
 * @brief This API
 */
void I2Cwritedelay(void)
{
	for(uint32_t i = 120000000;i==0;i--)
	{
	}

}
