/*
 * main.c
 *
 *  Created on: Nov 7, 2019
 *      Author: embedded
 */

#include "bmi160.h"

#include "rtos_i2c.h"
#include "rtos_uart.h"
#include <stdio.h>
#include "FreeRTOS.h"

int main()
{
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    BMI160_init();

    /* creating task for control  */
    xTaskCreate(BMI160_CONTROL_TASK, "main task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-3, NULL);
	/* creating task for the accelerometer's reading */
    xTaskCreate(BMI160_READ_ACC_TASK, "read acc task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL);
	/* creating task for the gyroscope's reading */
    xTaskCreate(BMI160_READ_GYRO_TASK, "read gyr task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL);
	/* creating task for serial communication */
    xTaskCreate(BMI160_SERIAL_COMM_TASK, "send task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL);

    /* start scheduler */
    vTaskStartScheduler();

    for(;;)
    {

    }

	return 0;
}
