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

#define BMI160_DATA_SIZE 			(1U)
#define BMI160_SUBADDRESS_SIZE 		(1U)
#define BMI160_BAUDRATE 					(100000U)

void gpio_i2c_config(void);

void bmi160_write(uint16_t reg,uint8_t data);

uint8_t bmi160_read(uint16_t reg);

void I2Cwritedelay(void);

#endif /* BMI160_H_ */
