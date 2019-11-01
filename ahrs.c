/*
 * ahrs.c
 *
 *  Created on: Nov 1, 2019
 *      Author: LuisFernando
 */


#include "ahrs.h"


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

/*
 * Declaration of the structs to communicate between ahrs and mahony.
 */
struct bmi160_device sensor_ahrs;
struct comm_msg_acc_t acc_device_ahrs;
struct comm_msg_acc_t gyr_device_ahrs;

struct rtos_i2c_bmi160_package sensor_package_ahrs;

MahonyAHRSEuler_t mahony_device;

/*!
 * @brief This API is used to calculate the deviation of the values so it can prevent the wrong lectures of the values.
 * Processing data only.
 */
void bmi160_varianza(void)
{
	volatile uint8_t header_counter = 0;

	float media_gyr_x = 0;
	float media_gyr_y = 0;
	float media_gyr_z = 0;

	float media_acc_x = 0;
	float media_acc_y = 0;
	float media_acc_z = 0;


	/*
	 *
	 */

	acc_var_x[acc_index_x] = acc_device_ahrs.x;
	acc_index_x++;
	acc_var_y[acc_index_y] = acc_device_ahrs.y;
	acc_index_y++;
	acc_var_z[acc_index_z] = acc_device_ahrs.z;
	acc_index_z++;

	/*
	 *
	 */

	gyr_var_x[gyr_index_x] = gyr_device_ahrs.x;
	gyr_index_x++;
	gyr_var_y[gyr_index_y] = gyr_device_ahrs.y;
	gyr_index_y++;
	gyr_var_z[gyr_index_z] = gyr_device_ahrs.z;
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
		if((BMI160_VAR_BUFFER_SIZE == gyr_index_z) && (BMI160_VAR_BUFFER_SIZE == acc_index_z))
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
 * @brief This API is used and apply of the mahony function provided by the teacher so it can read the AHRS system.
 * Use RTOS system.
 */
void bmi160_send_mahony(void)
{
	while(TRUE)
	{
		/* ptr to mahony struct*/
	//	uint8_t* ptr = 0;

//		*ptr = &mahony_device;

		MahonyAHRSupdateIMU(gyr_device_ahrs.x,gyr_device_ahrs.y,gyr_device_ahrs.z,acc_device_ahrs.x,acc_device_ahrs.y,acc_device_ahrs.z);
	}
}
