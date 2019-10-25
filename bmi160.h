/*
 * bmi160.h
 *
 *  Created on: Oct 23, 2019
 *      Author: LuisFernando
 */

#ifndef BMI160_H_
#define BMI160_H_

#include "fsl_port.h"
#include "fsl_i2c.h"
#include "FREErtos.h"
#include "event_groups.h"
#include "semphr.h"
#include "fsl_i2c_freertos.h"
#include "gpio.h"
#include "i2c.h"

struct bmi160_device
{
    /*! Chip Id */
    uint8_t chip_id;

    /*! Device Id */
    uint8_t id;

    /*! Chip Id */
    uint8_t data;

    /*! Data Address */
    uint16_t address;

    /*! Transfer Complete Flag */
    uint8_t master_transfer_flag;
};

/* BMI160 I2C address */
#define BMI160_I2C_ADDR                      (0x68)
#define BMI160_I2C_SUBADDR_SIZE              (2U)
#define BMI160_I2C_DATA_SIZE              	 (2U)
#define BMI160_I2C_BAUDRATE					 (100000U)

/*
 * @brief This API
 */
void gpio_i2c_config(void);
/*
 * @brief This API
 */
void bmi160_rtos_init(void);
/*
 * @brief This API
 */
void bmi160_write(uint8_t data, uint8_t address);

/*
 * @brief This API
 */
uint8_t bmi160_read(uint8_t address);

#endif /* BMI160_H_ */
