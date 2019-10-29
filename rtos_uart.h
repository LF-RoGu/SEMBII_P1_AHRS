/*
 * rtos_uart.h
 *
 *  Created on: Sep 12, 2018
 *      Author:
 */

#ifndef RTOS_UART_H_
#define RTOS_UART_H_

#include <stdint.h>

typedef enum {rtos_uart0,rtos_uart1} rtos_uart_number_t;
typedef enum {rtos_uart_portA,rtos_uart_portB,rtos_uart_portC,rtos_uart_portD,rtos_uart_portE} rtos_uart_port_t;
typedef enum {rtos_uart_sucess,rtos_uart_fail} rtos_uart_flag_t;

typedef struct
{
	uint32_t  baudrate;
	rtos_uart_number_t uart_number;
	rtos_uart_port_t port;
	uint8_t rx_pin;
	uint8_t tx_pin;
	uint8_t pin_mux;
}rtos_uart_config_t;

rtos_uart_flag_t rtos_uart_init(rtos_uart_config_t config);
rtos_uart_flag_t rtos_uart_send(rtos_uart_number_t uart_number,uint8_t * buffer, uint16_t lenght);
rtos_uart_flag_t rtos_uart_receive(rtos_uart_number_t uart_number, uint8_t * buffer, uint16_t lenght);

#endif /* RTOS_UART_H_ */
