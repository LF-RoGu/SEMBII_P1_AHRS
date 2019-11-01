/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    AHRS_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "rtos_i2c.h"
#include "rtos_uart.h"
/* TODO: insert other include files here. */
#include "FreeRTOS.h"
#include "bmi160.h"
#include "ahrs.h"
#include "task.h"
/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    //PRINTF("Hello World\n");

    /*
     * Config i2c & uart with RTOS
     */
    /**/
    rtos_i2c_config_t i2c_config_t;

    i2c_config_t.baudrate = RTOS_I2C_BAUDRATE;
    i2c_config_t.i2c_number = rtos_i2c_0;
    i2c_config_t.port = rtos_i2c_portE;
    i2c_config_t.SDA_pin = bit_25;
    i2c_config_t.SCL_pin = bit_24;
    i2c_config_t.pin_mux = rtos_mux_alt05;

    rtos_i2c_init(i2c_config_t);

    /**/
    rtos_uart_config_t uart_config_t;

    uart_config_t.uart_number = rtos_uart0;
    uart_config_t.baudrate = BD_115200;
    uart_config_t.port = rtos_uart_portB;
    uart_config_t.tx_pin = bit_17;
    uart_config_t.rx_pin = bit_16;
    uart_config_t.pin_mux = rtos_mux_alt3;

    rtos_uart_init(uart_config_t);


    /* BMI160_NORMAL_MODE task creation */
    xTaskCreate(bmi160_normal_mode_config, "bmi160_normal_mode", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);

    /* BMI160_READ_ACC_GYR task creation */
    xTaskCreate(bmi160_read_acc_gyr, "bmi160_read_acc", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL);
    /* start scheduler */
    vTaskStartScheduler();

    /* Enter an infinite loop*/
    for(;;)
    {
    }
    return 0 ;
}
