/*
 * bmi160.h
 *
 *  Created on: Oct 25, 2019
 *      Author: LuisFernando
 */

#ifndef BMI160_H_
#define BMI160_H_

#include "fsl_i2c.h"
#include "bits.h"
#include "bmi160_defs.h"

#define BMI160_DATA_SIZE 					(1U)
#define BMI160_SUBADDRESS_SIZE 				(1U)
#define BMI160_BAUDRATE 					(100000U)
/* Offset*/
/* Offset reg gyr*/
#define BMI160_OFFSET_GYR_Z 				(0x76)
#define BMI160_OFFSET_GYR_Y 				(0x75)
#define BMI160_OFFSET_GYR_X 				(0x74)
/* Offset reg acc*/
#define BMI160_OFFSET_ACC_Z 				(0x73)
#define BMI160_OFFSET_ACC_Y 				(0x72)
#define BMI160_OFFSET_ACC_X 				(0x71)

/* Read Reg*/
/* Read reg acc*/
#define BMI160_READ_ACC_Z_H 				(0x17)
#define BMI160_READ_ACC_Z_L 				(0x16)
#define BMI160_READ_ACC_Y_H 				(0x15)
#define BMI160_READ_ACC_Y_L 				(0x14)
#define BMI160_READ_ACC_X_H 				(0x13)
#define BMI160_READ_ACC_X_L 				(0x12)
/* Read reg gyr*/
#define BMI160_READ_GYR_Z_H 				(0x11)
#define BMI160_READ_GYR_Z_L 				(0x10)
#define BMI160_READ_GYR_Y_H 				(0x0F)
#define BMI160_READ_GYR_Y_L 				(0x0E)
#define BMI160_READ_GYR_X_H 				(0x0D)
#define BMI160_READ_GYR_X_L 				(0x0C)

/* Chip ID*/
#define BMI160_READ_CHIP_ID					(0x00)

/* STATUS*/
#define BMI160_READ_PMU_STATUS				(0x03)
/* ACC_PMU_STATUS*/
#define BMI160_ACC_PMU_STATUS				(0x48)
/* GYR_PMU_STATUS*/
#define BMI160_GYR_PMU_STATUS				(0x12)
/* MAG_PMU_STATUS*/
#define BMI160_MAG_PMU_STATUS				(0x03)

/* PMU CONF*/
#define BMI160_GYR_RANGE	                (0x43)
#define BMI160_GYR_CONF		                (0x42)

#define BMI160_ACC_RANGE	                (0x41)
#define BMI160_ACC_CONF		                (0x40)

typedef enum
{
	X_ENUM,
	Y_ENUM,
	Z_ENUM
}ACCEL_ENUM;

typedef enum
{
	PMU_ACC_ERROR,
	PMU_GYR_ERROR,
	PMU_MAG_ERROR
}PMU_STATUS_ENUM;

struct acc_config_t
{
    /*  accelerometer under-sampling */
    uint8_t acc_us;
    /*  accelerometer bandwidth */
    uint8_t acc_bw;
    /*  accelerometer output data rate*/
    uint8_t acc_odr;
	/*  accelerometer range */
	uint8_t acc_range;

	uint8_t acc_pwr;
};

struct accelerometer_t
{
	/* Accelerometer configuration */
	struct acc_config_t acc_config;
    /* Accelerometer chip ID */
	uint8_t chip_id;
};

struct bmi160_t
{
	/* Accelerometer chip ID */
	uint8_t chip_id;

	/* Accelerometer configuration */
	struct acc_config_t acc_config;

	uint8_t soft_rst;

};

struct bmi160_device
{
    /*! Structure to configure Accel sensor */
    struct bmi160_cfg accel_cfg;

    /*! Structure to configure Gyro sensor */
    struct bmi160_cfg gyro_cfg;
};

struct comm_msg_gyr_t
{
	uint32_t header;
	float x;
	float y;
	float z;
};

struct comm_msg_acc_t
{
	uint32_t header;
	float x;
	float y;
	float z;
};

/*!
 * @brief This API
 */
void gpio_i2c_config(void);
/*!
 * @brief This API
 */
void bmi160_normal_mode_config(struct bmi160_t* device);

/*!
 * @brief This API
 */
void bmi160_write(uint8_t reg, uint8_t data);

/*!
 * @brief This API
 */
uint8_t bmi160_read_pmu_status(void);

/*!
 * @brief This API
 */
uint8_t bmi160_read(uint8_t reg);

/* READ GYR AND ACC*/
/*!
 * @brief This API
 */
void bmi160_read_acc(void);

/*!
 * @brief This API
 */
void data_axis_acc(void);

/*!
 * @brief This API
 */
void bmi160_print_acc(void);

/*!
 * @brief This API
 */
void bmi160_read_gyr(void);

/*!
 * @brief This API
 */
void data_axis_gyr(void);

/*!
 * @brief This API
 */
void bmi160_print_gyr(void);

/*!
 * @brief This API
 */
void bmi160_get_data(void);

/* OFFSET SETUP*/
/*!
 * @brief This API
 */
void bmi160_offset_gyr(uint8_t accel,uint8_t off);

/*!
 * @brief This API
 */
void bmi160_offset_acc(uint8_t accel,uint8_t off);

/*!
 * @brief This API
 */
void I2Cwritedelay(void);

void accelerometer_task();

void bmi_160_init(struct bmi160_t* device);

void bmi160_soft_reset(struct bmi160_t* device);

void bmi160_get_device_config(struct bmi160_t* device);
#endif /* BMI160_H_ */
