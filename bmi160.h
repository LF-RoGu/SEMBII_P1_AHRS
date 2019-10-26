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
#include "gpio.h"

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

typedef enum
{
	X_ENUM,
	Y_ENUM,
	Z_ENUM
}ACCEL_ENUM;

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

/*
 *
 */
void gpio_i2c_config(void);
/*
 *
 */
void bmi160_write(uint16_t reg,uint8_t data);
/*
 *
 */
uint8_t bmi160_read(uint16_t reg);
/* READ GYR AND ACC*/
/*
 *
 */
void bmi160_read_acc(void);
/*
 *
 */
void bmi160_read_gyr(void);
/* OFFSET SETUP*/
/*
 *
 */
void bmi160_offset_gyr(uint8_t accel,uint8_t off);
/*
 *
 */
void bmi160_offset_acc(uint8_t accel,uint8_t off);
/*
 *
 */
void I2Cwritedelay(void);

#endif /* BMI160_H_ */
