/*
 * ahrs.h
 *
 *  Created on: Nov 1, 2019
 *      Author: LuisFernando
 */

#ifndef AHRS_H_
#define AHRS_H_

#include "bmi160.h"
#include "mahony.h"


/*!
 * @brief This API is used and apply of the mahony function provided by the teacher so it can read the AHRS system.
 */
void bmi160_send_mahony(void);

/*!
 * @brief This API is used to calculate the deviation of the values so it can prevent the wrong lectures of the values.
 */
void bmi160_varianza(void);

#endif /* AHRS_H_ */
