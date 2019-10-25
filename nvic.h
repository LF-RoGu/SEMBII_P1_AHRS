/*
 * NVIC.h
 *
 *  Created on: 11/08/2017
 *      Author: jlpe
 */

#ifndef NVIC_H_
#define NVIC_H_

#include <stdint.h>
#include "MK64F12.h"

#define NVIC_global_enable_interrupts __enable_irq()
#define NVIC_disable_interrupts __disable_irq()

/** enum type that defines the priority levels for the NVIC.
 * The highest priority is PRIORITY_0 and the lowest PRIORITY_15 */
typedef enum {PRIORITY_0, PRIORITY_1, PRIORITY_2, PRIORITY_3, PRIORITY_4, PRIORITY_5, PRIORITY_6,
			  PRIORITY_7, PRIORITY_8, PRIORITY_9, PRIORITY_10, PRIORITY_11, PRIORITY_12, PRIORITY_13,
			  PRIORITY_14, PRIORITY_15 } priority_level_t;

/** enum type that defines the IRQs for the NVIC.*/
typedef enum {
	DMA_CH0_IRQ, //0
	DMA_CH1_IRQ, //1
	DMA_CH2_IRQ, //2
	DMA_CH3_IRQ, //3
	DMA_CH4_IRQ, //4
	DMA_CH5_IRQ, //5
	DMA_CH6_IRQ, //6
	DMA_CH7_IRQ, //7
	DMA_CH8_IRQ, //8
	DMA_CH9_IRQ, //9
	DMA_CH10_IRQ, //10
	DMA_CH11_IRQ, //11
	DMA_CH12_IRQ, //12
	DMA_CH13_IRQ, //13
	DMA_CH14_IRQ, //14
	DMA_CH15_IRQ, //15
	DMA_CH_ALL_IRQ, //16
	MCM_ISR_IRQ, //17
	FM0_ISR_IRQ, //18
	FM1_ISR_IRQ, //19
	MC_ISR_IRQ, //20
	LLWU_ISR_IRQ, //21
	WDOG_OR_EWM_IRQ, //22
	RNG_IRQ, //23
	I2C0_IRQ, //24
	I2C1_IRQ, //25
	SPI0_IRQ,//26
	SPI1_IRQ,//27
	I2S0_TX_IRQ, //28
	I2S0_RX_IRQ,//29
	NONE_IRQ, //30
	UART0_IRQ,//31
	UART0_ERR, //32
	UART1_IRQ,//33
	UART1_ERR_IRQ,//34
	UART2_IRQ, //35
	UART2_ERR_IRQ, //36
	UART3_IRQ, //37
	UART3_ERR_IRQ, //38
	ADC0_IRQ, //39
	CMP0_IRQ, //40
	CMP1_IRQ, //41
	FTM0_IRQ,//42
	FTM1_IRQ,//43
	FTM2_IRQ,//44
	CMT_IRQ, //45
	RTC_ALARM_IRQ, //46
	RTC_SEC_IRQ, //47
	PIT_CH0_IRQ,//48
	PIT_CH1_IRQ,//49
	PIT_CH2_IRQ,//50
	PIT_CH3_IRQ,//51
	PDB_IRQ, //52
	USB_OTG_IRQ, //53
	USB_CHARGER_DETECT_IRQ,//54
	NONE2, //55
	DAC0_IRQ, //56
	MCG_IRQ,//57
	LOW_POWER_TIMER_IRQ,//58
	PORTA_IRQ, //59
	PORTB_IRQ, //60
	PORTC_IRQ, //61
	PORTD_IRQ, //62
	PORTE_IRQ, //63
	SOFTWARE_IRQ, //64
	SPI2_IRQ, //65
	UART4_IRQ,//66
	UART4ERR_IRQ, //67
	UART5_IRQ, //68
	UART5_ERR_IRQ, //69
	CMP2_IRQ,//70
	FTM3_IRQ, //71
	DAC1_IRQ, //72
	ADC1_IRQ, //73
	I2C2_IRQ, //74
	CAN0_IRQ, //75
	CAN0_BUS_OFF_IRQ, //76
	CAN0_ERR_IRQ, //77
	CAN0_TX_IRQ, //78
	CAN0_RX_IRQ, //79
	CAN0_WP_IRQ, //80
	SDHC_ISR_IRQ, //81
	ETHERNET_MAC0_IRQ,//82
	ETHERNET_MAC1_IRQ, //83
	ETHERNET_MAC2_IRQ, //84
	ETHERNET_MAC3_IRQ //85
} interrupt_t;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function enables a IRQ in the NVIC and establishes its priority.

 	 \param[in] interruptNumber is the desired IRQ to be activivated.
 	 \param[in] priority establishes the priority of the IRQ
 	 \return void
 */
void NVIC_enable_interrupt_and_priotity(interrupt_t interrupt_number, priority_level_t priority);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function establishes the threshold level to interrupt the MCU.

 	 \param[in]  priority threshold to be established.
 	 \return void
 	 \todo Implement a mechanism to clear interrupts by a specific pin.
 */
void NVIC_set_basepri_threshold(priority_level_t priority);


#endif /* NVIC_H_ */
