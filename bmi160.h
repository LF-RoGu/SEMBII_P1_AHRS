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
#include "FreeRTOS.h"

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "timers.h"


#include "event_groups.h"
#include "semphr.h"
#include "task.h"


#define BMI160_DATA_SIZE 					(1U)
#define BMI160_SUBADDRESS_SIZE 				(1U)
#define BMI160_BAUDRATE 					(115200U)
#define BMI160_VAR_BUFFER_SIZE				(10U)
#define BMI160_HEADER						(0xAAAAAAAA)
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
#define BMI160_MAX_VALUE					(32758U)
#define BMI160_ACC_UNITS_RANGE				(360U)
#define BMI160_GYR_UNITS_RANGE				(360U)


#define MAHONY_HEADER						(0U)
#define MAHONY_ROLL							(1U)
#define MAHONY_PITCH						(2U)
#define MAHONY_YAW							(3U)

#define mEventAccelRead						(1<<0)
#define mEventGyroRead						(2<<0)
#define mEventAccelRead						(3<<0)
#define mEventAccelRead						(4<<0)


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
	int8_t * buffer;
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

struct comm_msg_mahony_t
{
	uint32_t header;
	float roll_mahony;
	float pitch_mahony;
	float yaw_mahony;

};

void BMI160_CONTROL_TASK();
void BMI160_READ_ACC_TASK(void);
void BMI160_READ_GYRO_TASK(void);
void BMI160_SERIAL_COMM_TASK(void);
void BMI160_RECEIVE_PKG();
void bmi160_normal_mode_config(void);
void data_axis_acc(void);
void data_axis_gyr(void);
void convert_value_acc(int16_t *acc_axis_data);
void convert_value_gyr(int16_t *gyr_axis_data);
void bmi160_varianza(void);

#endif /* BMI160_H_ */
