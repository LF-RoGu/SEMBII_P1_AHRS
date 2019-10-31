/*
 * bmi160.h
 *
 *  Created on: Oct 29, 2019
 *      Author: LuisFernando
 */

#ifndef BMI160_H_
#define BMI160_H_

#include "rtos_i2c.h"
#include "bmi160_defs.h"
#include "mahony.h"
#include "rtos_uart.h"
#include <math.h>
#include <stdio.h>

#define BMI160_DATA_SIZE 					(1U)
#define BMI160_SUBADDRESS_SIZE 				(1U)
#define BMI160_BAUDRATE 					(100000U)
#define BMI160_VAR_BUFFER_SIZE				(10U)
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

/**/
#define BMI160_MAX_VALUE					(65535U)
#define BMI160_ACC_UINTS_RANGE_POSITIVE		(2U)
#define BMI160_ACC_UINTS_RANGE_NEGATIVE		(-2)
#define BMI160_GYR_UNITS_RANGE_POSITIVE		(2000U)
#define BMI160_GYR_UNITS_RANGE_NEGATIVE		(-2000)
#define PI_RAD								(3.14159)
#define BMI160_GYR_RAD_RANGE_POSITIVE		(2*PI_RAD)
#define BMI160_GYR_RAD_RANGE_NEGATIVE		(PI_RAD)

typedef enum
{
	X_REG,
	Y_REG,
	Z_REG
}ACCEL_ENUM;
typedef enum
{
	X_LOW,
	X_HIGH,
	Y_LOW,
	Y_HIGH,
	Z_LOW,
	Z_HIGH
}REG_READ_ENUM;

struct rtos_i2c_bmi160_package
{
	rtos_i2c_number_t i2c_number;
	uint8_t * buffer;
	uint16_t length;
	uint16_t slave_addr;
	uint16_t subaddr;
	uint8_t subsize;
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
 * @brief This API is for the startup of the device, so it can run on NORMAL MODE as standard.
 * Use RTOS system.
 */
void bmi160_normal_mode_config(void);

/*!
 * @brief This API if for the read the registers corresponding to the accelerometer.
 */
void bmi160_read_acc(void);

/*!
 * @brief This API is for the read of the registers corresponding to the gyroscope.
 */
void bmi160_read_gyr(void);

/*!
 * @brief This API is for the RTOS system so it can call the read of the registers for the accelerometer & the gyroscope.
 */
void bmi160_read_acc_gyr(void);

/*!
 * @brief This API is for the conversion of the registers of 8 bits each, to 16 bits each for the accelerometer.
 * Only for reading and processing purpose.
 */
void data_axis_acc(void);

/*!
 * @brief This API is for the conversion of the register of 8 bits each, to 16 bits each for the accelerometer.
 * Only for reading and processing purpose.
 */
void data_axis_gyr(void);

/*!
 * @brief This API is for the conversion of the value of the register corresponding to the accelerometer to floating point.
 */
void convert_value_acc(uint16_t *axis_data);

/*!
 * @brief This API is for the conversion of the value of the register corresponding to the accelerometer to degrees.
 * Processing data only
 */
void convert_angle_acc(uint16_t *acc_axis_data);

/*!
 * @brief This API is for the conversion of the value of the register corresponding to the gyroscope to floating point.
 */
void convert_value_gyr(uint16_t *axis_data);

/*!
 * @brief This API is for the conversion of the value of the register corresponding to the gyroscope to floating point.
 * Processing data only
 */
void convert_value_gyr(uint16_t *gyr_axis_data);

/*!
 * @brief This API is only for debug purpose, used to print the value captured of the registers in 16 bits format.
 */
void bmi160_print_acc_dec(void);

/*!
 * @brief This API is only for debu puspose, used to print the value captured of the registers in floatin point format.
 */
void bmi160_print_acc_float(void);

/*!
 * @brief This API is only for debug purpose, used to print the value captured of the registers in 16 bits format.
 */
void bmi160_print_gyr_dec(void);

/*!
 * @brief This API is only for debug purpose, used to print the value captured of the registers in floating point format.
 */
void bmi160_print_gyr_float(void);

/*!
 * @brief This API is used and apply of the mahony function provided by the teacher so it can read the AHRS system.
 */
void bmi160_send_mahony(void);

/*!
 * @brief This API is used to calculate the deviation of the values so it can prevent the wrong lectures of the values.
 */
void bmi160_varianza(void);


#endif /* BMI160_H_ */
