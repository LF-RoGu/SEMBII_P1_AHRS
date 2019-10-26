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
	sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
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
	/* GYR*/
	bmi160_write(BMI160_GYR_RANGE,sensor.gyro_cfg.range);
	bmi160_write(BMI160_GYR_CONF,(sensor.gyro_cfg.bw | sensor.gyro_cfg.odr));
	/* SET POWER MODE*/
	bmi160_write(BMI160_COMMAND_REG_ADDR,sensor.gyro_cfg.power);
	/* ACC*/
	bmi160_write(BMI160_ACC_RANGE,sensor.accel_cfg.range);
	bmi160_write(BMI160_ACC_CONF,(sensor.accel_cfg.bw | sensor.accel_cfg.odr));
	/* SET POWER MODE*/
	bmi160_write(BMI160_COMMAND_REG_ADDR,sensor.accel_cfg.power);
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
	}
	g_MasterCompletionFlag = false;

	I2Cwritedelay();
}
/*!
 * @brief This API
 */
uint8_t bmi160_read_pmu_status(void)
{
	uint8_t pmu_status_t;
	/* PMU STATUS*/
	pmu_status_t = bmi160_read(BMI160_READ_PMU_STATUS);

	switch(pmu_status_t)
	{
	case BMI160_ACC_PMU_STATUS:
		pmu_status_t = PMU_ACC_ERROR;
		break;
	case BMI160_GYR_PMU_STATUS:
		pmu_status_t = PMU_GYR_ERROR;
		break;
	case BMI160_MAG_PMU_STATUS:
		pmu_status_t = PMU_MAG_ERROR;
		break;
	default:
		break;
	}
	return pmu_status_t;
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

	/**/
	data_axis_acc();
}
void data_axis_acc(void)
{
	uint16_t data_temp;
	/* Data axis X*/
	data_temp = ((uint16_t) g_read_acc[0]) << 8;
	data_temp += g_read_acc[0];

	g_data_axis_acc[0] = data_temp;
	/* Data axis Y*/
	data_temp = ((uint16_t) g_read_acc[3]) << 8;
	data_temp += g_read_acc[2];

	g_data_axis_acc[1] = data_temp;
	/* Data axis Z*/
	data_temp = ((uint16_t) g_read_acc[5]) << 8;
	data_temp += g_read_acc[4];

	g_data_axis_acc[3] = data_temp;


	/* Print axis*/
	bmi160_print_acc();
}
/*!
 * @brief This API
 */
void bmi160_print_acc(void)
{
	printf("acc axis: X: %d Y: %d Z: %d\n", g_data_axis_acc[0],g_data_axis_acc[1],g_data_axis_acc[2]);
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

	/**/
	data_axis_gyr();
}
void data_axis_gyr(void)
{
	uint16_t data_temp;
	/* Data axis X*/
	data_temp = ((uint16_t) g_read_gyr[0]) << 8;
	data_temp += g_read_gyr[0];

	g_data_axis_gyr[0] = data_temp;
	/* Data axis Y*/
	data_temp = ((uint16_t) g_read_gyr[3]) << 8;
	data_temp += g_read_gyr[2];

	g_data_axis_gyr[1] = data_temp;
	/* Data axis Z*/
	data_temp = ((uint16_t) g_read_gyr[5]) << 8;
	data_temp += g_read_gyr[4];

	g_data_axis_gyr[3] = data_temp;


	/* Print axis*/
	bmi160_print_gyr();
}
/*!
 * @brief This API
 */
void bmi160_print_gyr(void)
{
	printf("gyr axis: X: %d Y: %d Z: %d\n", g_data_axis_gyr[0],g_data_axis_gyr[1],g_data_axis_gyr[2]);
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
