/*
 * bmi160.c
 *
 *  Created on: Oct 29, 2019
 *      Author: LuisFernando
 */

#include "bmi160.h"


/* Buffer to read acc & gyr*/
uint8_t g_read_gyr[6] = {0};
uint8_t g_read_acc[6] = {0};

uint16_t g_data_axis_acc[3] = {0};
uint16_t g_data_axis_gyr[3] = {0};

uint8_t header_counter = 0;

float acc_var_x [BMI160_VAR_BUFFER_SIZE] = {0};
uint8_t acc_index_x = 0;
float acc_var_y [BMI160_VAR_BUFFER_SIZE] = {0};
uint8_t acc_index_y = 0;
float acc_var_z [BMI160_VAR_BUFFER_SIZE] = {0};
uint8_t acc_index_z = 0;

float gyr_var_x [BMI160_VAR_BUFFER_SIZE] = {0};
uint8_t gyr_index_x = 0;
float gyr_var_y [BMI160_VAR_BUFFER_SIZE] = {0};
uint8_t gyr_index_y = 0;
float gyr_var_z [BMI160_VAR_BUFFER_SIZE] = {0};
uint8_t gyr_index_z = 0;

float acc_var[3] = {0};
float gyr_var[3] = {0};

struct bmi160_device sensor;
struct comm_msg_acc_t acc_device;
struct comm_msg_acc_t gyr_device;

struct rtos_i2c_bmi160_package sensor_package;

/*!
 * @brief This API
 */
void bmi160_normal_mode_config(void)
{

	while(TRUE)
	{
		/* SOFT RESET*/
		sensor_package.i2c_number = rtos_i2c_0;
		sensor_package.buffer = BMI160_SOFT_RESET_CMD;
		sensor_package.length = BMI160_DATA_SIZE;
		sensor_package.slave_addr = BMI160_I2C_ADDR;
		sensor_package.subaddr = BMI160_COMMAND_REG_ADDR;
		sensor_package.subsize = BMI160_SUBADDRESS_SIZE;

		rtos_i2c_transfer(
				sensor_package.i2c_number,
				sensor_package.buffer,
				sensor_package.length,
				sensor_package.slave_addr,
				sensor_package.subaddr,
				sensor_package.subsize);
		/*
		 * ACC
		 */
		/* Select the Output data rate, range of accelerometer sensor */
		sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
		sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
		/* Select the power mode of accelerometer sensor */
		sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
		/*
		 * GYRO
		 */
		/* Select the Output data rate, range of Gyroscope sensor */
		sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
		sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
		/* Select the power mode of Gyroscope sensor */
		sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;


		/* Sensor package*/
		/* ACC*/
		sensor_package.i2c_number = rtos_i2c_0;
		sensor_package.buffer = &sensor.accel_cfg.power;
		sensor_package.length = BMI160_DATA_SIZE;
		sensor_package.slave_addr = BMI160_I2C_ADDR;
		sensor_package.subaddr = BMI160_COMMAND_REG_ADDR;
		sensor_package.subsize = BMI160_SUBADDRESS_SIZE;

		/* SET POWER MODE*/
		rtos_i2c_transfer(
				sensor_package.i2c_number,
				sensor_package.buffer,
				sensor_package.length,
				sensor_package.slave_addr,
				sensor_package.subaddr,
				sensor_package.subsize);

		/* GYR*/
		sensor_package.i2c_number = rtos_i2c_0;
		sensor_package.buffer = &sensor.gyro_cfg.power;
		sensor_package.length = BMI160_DATA_SIZE;
		sensor_package.slave_addr = BMI160_I2C_ADDR;
		sensor_package.subaddr = BMI160_COMMAND_REG_ADDR;
		sensor_package.subsize = BMI160_SUBADDRESS_SIZE;

		/* SET POWER MODE*/
		rtos_i2c_transfer(
				sensor_package.i2c_number,
				sensor_package.buffer,
				sensor_package.length,
				sensor_package.slave_addr,
				sensor_package.subaddr,
				sensor_package.subsize);

		/*Kill task*/
		vTaskSuspend(NULL);

	}
}

/*!
 * @brief This API
 */
void bmi160_read_acc(void)
{
	uint8_t index = 0;

	uint8_t data_temp;

	/* Sensor package*/
	/* ACC*/
	sensor_package.i2c_number = rtos_i2c_0;
	sensor_package.buffer = &data_temp;
	sensor_package.length = BMI160_DATA_SIZE;
	sensor_package.slave_addr = BMI160_I2C_ADDR;
	sensor_package.subaddr = BMI160_READ_ACC_X_L;
	sensor_package.subsize = BMI160_SUBADDRESS_SIZE;

	for(index = X_LOW; index < Z_HIGH; index++)
	{
		rtos_i2c_receive(
						sensor_package.i2c_number,
						sensor_package.buffer,
						sensor_package.length,
						sensor_package.slave_addr,
						sensor_package.subaddr,
						sensor_package.subsize);

		g_read_acc[index] = *sensor_package.buffer;

		sensor_package.subaddr++;
	}
	/*
	 *
	 */
	data_axis_acc();
}
/*!
 * @brief This API
 */
void bmi160_read_gyr(void)
{
	uint8_t index = 0;

	uint8_t data_temp;

	/* Sensor package*/
	/* ACC*/
	sensor_package.i2c_number = rtos_i2c_0;
	sensor_package.buffer = &data_temp;
	sensor_package.length = BMI160_DATA_SIZE;
	sensor_package.slave_addr = BMI160_I2C_ADDR;
	sensor_package.subaddr = BMI160_READ_GYR_X_L;
	sensor_package.subsize = BMI160_SUBADDRESS_SIZE;

	for(index = X_LOW; index < Z_HIGH; index++)
	{
		rtos_i2c_receive(
						sensor_package.i2c_number,
						sensor_package.buffer,
						sensor_package.length,
						sensor_package.slave_addr,
						sensor_package.subaddr,
						sensor_package.subsize);

		g_read_gyr[index] = *sensor_package.buffer;

		sensor_package.subaddr++;
	}

	/*
	 *
	 */
	data_axis_gyr();
}

/*!
 * @brief This API
 */
void bmi160_read_acc_gyr(void)
{
	while(TRUE)
	{
		/* Read values and stores it in the acc and gyt structure*/
		bmi160_read_acc();
		bmi160_read_gyr();

		/* Get the var of both data*/
		//bmi160_varianza();

		/* Send data to the mahony*/
		bmi160_send_mahony();
	}
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

	/*
	 *
	 */
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

	/*
	 *
	 */
	convert_value_gyr(g_data_axis_gyr);
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
}

/*!
 * @brief This API
 */
void bmi160_varianza(void)
{
	float media_gyr_x = 0;
	float media_gyr_y = 0;
	float media_gyr_z = 0;

	float media_acc_x = 0;
	float media_acc_y = 0;
	float media_acc_z = 0;
	/*
	 *
	 */
	acc_device.header = header_counter;

	acc_var_x[acc_index_x] = acc_device.x;
	acc_index_x++;
	acc_var_y[acc_index_y] = acc_device.y;
	acc_index_y++;
	acc_var_z[acc_index_z] = acc_device.z;
	acc_index_z++;

	/*
	 *
	 */
	gyr_device.header = header_counter;

	gyr_var_x[gyr_index_x] = gyr_device.x;
	gyr_index_x++;
	gyr_var_y[gyr_index_y] = gyr_device.y;
	gyr_index_y++;
	gyr_var_z[gyr_index_z] = gyr_device.z;
	gyr_index_z++;

	header_counter++;

	if(BMI160_VAR_BUFFER_SIZE == header_counter)
	{
		/* do var calculus*/
		uint8_t index = 0;
		for(index = 0; index <= BMI160_VAR_BUFFER_SIZE; index++)
		{
			acc_var[X_REG] += acc_var_x[index];
			gyr_var[X_REG] += gyr_var_x[index];

			acc_var[Y_REG] += acc_var_y[index];
			gyr_var[Y_REG] += gyr_var_y[index];

			acc_var[Z_REG] += acc_var_z[index];
			gyr_var[Z_REG] += gyr_var_z[index];
			if(BMI160_VAR_BUFFER_SIZE == index)
			{
				/* Calcula la media de la muestra*/
				media_acc_x = acc_var[X_REG] / BMI160_VAR_BUFFER_SIZE;
				media_gyr_x = gyr_var[X_REG] / BMI160_VAR_BUFFER_SIZE;

				media_acc_x = acc_var[Y_REG] / BMI160_VAR_BUFFER_SIZE;
				media_gyr_x = gyr_var[Y_REG] / BMI160_VAR_BUFFER_SIZE;

				media_acc_x = acc_var[Z_REG] / BMI160_VAR_BUFFER_SIZE;
				media_gyr_x = gyr_var[Z_REG] / BMI160_VAR_BUFFER_SIZE;

				/* Calcula la varianza de la muestra*/

				uint8_t index_var = 0;
				for(index_var = 0; index_var < BMI160_VAR_BUFFER_SIZE; index_var++)
				{
					/* Varianza de las muestras*/
					acc_var[X_REG] += ((acc_var_x[index_var] - media_acc_x) * (acc_var_x[index_var] - media_acc_x)) / (BMI160_VAR_BUFFER_SIZE - 1);
					gyr_var[X_REG] += ((gyr_var_x[index_var] - media_gyr_x) * (gyr_var_x[index_var] - media_gyr_x)) / (BMI160_VAR_BUFFER_SIZE - 1);

					acc_var[Y_REG] += ((acc_var_y[index_var] - media_acc_y) * (acc_var_y[index_var] - media_acc_y)) / (BMI160_VAR_BUFFER_SIZE - 1);
					gyr_var[Y_REG] += ((gyr_var_y[index_var] - media_gyr_y) * (gyr_var_y[index_var] - media_gyr_y)) / (BMI160_VAR_BUFFER_SIZE - 1);

					acc_var[Z_REG] += ((acc_var_z[index_var] - media_acc_z) * (acc_var_z[index_var] - media_acc_z)) / (BMI160_VAR_BUFFER_SIZE - 1);
					gyr_var[Z_REG] += ((gyr_var_z[index_var] - media_gyr_z) * (gyr_var_z[index_var] - media_gyr_z)) / (BMI160_VAR_BUFFER_SIZE - 1);
				}
				/* Desviacion estardar*/
				acc_var[X_REG] = sqrt(acc_var[X_REG]);
				gyr_var[X_REG] = sqrt(gyr_var[X_REG]);

				acc_var[Y_REG] = sqrt(acc_var[Y_REG]);
				gyr_var[Y_REG] = sqrt(gyr_var[Y_REG]);

				acc_var[Z_REG] = sqrt(acc_var[Z_REG]);
				gyr_var[Z_REG] = sqrt(gyr_var[Z_REG]);
			}
		}
		/* rst buffer counter*/
		if((BMI160_VAR_BUFFER_SIZE == gyr_device.header) && (BMI160_VAR_BUFFER_SIZE == acc_device.header))
		{
			acc_index_x = 0;
			acc_index_y = 0;
			acc_index_z = 0;

			gyr_index_x = 0;
			gyr_index_y = 0;
			gyr_index_z = 0;

			header_counter = 0;
		}
	}
}

/*!
 * @brief This API
 */
void bmi160_send_mahony(void)
{
	while(TRUE)
	{
		MahonyAHRSupdateIMU(gyr_device.x,gyr_device.y,gyr_device.z,acc_device.x,acc_device.y,acc_device.z);
	}
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
	printf("acc axis: X: %f Y: %f Z: %f\n", acc_device.x,acc_device.y,acc_device.z);
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
	printf("gyr axis: X: %f Y: %f Z: %f\n", acc_device.x,acc_device.y,acc_device.z);
}
