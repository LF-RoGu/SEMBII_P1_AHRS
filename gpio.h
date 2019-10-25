/**
	\file
	\brief
		This is the source file for the GPIO device driver for Kinetis K64.
		It contains all the implementation for configuration functions and runtime functions.
		i.e., this is the application programming interface (API) for the GPIO peripheral.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	18/02/2019
	\todo
	    Interrupts are not implemented in this API implementation.
 */
#ifndef GPIO_H_
#define GPIO_H_


#include "stdint.h"
#include "bits.h"


/** Constant that represent the clock enable for GPIO A */
#define GPIO_CLOCK_GATING_PORTA 0x00000200
/** Constant that represent the clock enable for GPIO B */
#define GPIO_CLOCK_GATING_PORTB 0x00000400
/** Constant that represent the clock enable for GPIO C */
#define GPIO_CLOCK_GATING_PORTC 0x00000800
/** Constant that represent the clock enable for GPIO D  */
#define GPIO_CLOCK_GATING_PORTD 0x00001000
/** Constant that represent the clock enable for GPIO E */
#define GPIO_CLOCK_GATING_PORTE 0x00002000


/** Selects a pullup resistor */
#define GPIO_PS    0x00000001
/** Enables the pulldown or pullup resistors*/
#define GPIO_PE    0x00000002
/** Selects slow or fast slew rate */
#define GPIO_SRE   0x00000004
/** Enables the passive filter */
#define GPIO_PFE   0x00000010
/** Enables the open drain  */
#define GPIO_ODE   0x00000020
/** Selects between low drive strength and high drive strength */
#define GPIO_DSE   0x00000040
/** Selects alternative function 1 (GPIO) */
#define GPIO_MUX1  0x00000100
/** Selects alternative function 2 */
#define GPIO_MUX2  0x00000200
/** Selects alternative function 3 */
#define GPIO_MUX3  0x00000300
/** Selects alternative function 4 */
#define GPIO_MUX4  0x00000400
/** Selects alternative function 5 */
#define GPIO_MUX5  0x00000500
/** Selects alternative function 6 */
#define GPIO_MUX6  0x00000600
/** Selects alternative function 7 */
#define GPIO_MUX7  0x00000700
/** Sets DMA request on rising edge.*/
#define DMA_RISING_EDGE    0x00010000
/** Sets DMA request on falling edge.*/
#define DMA_FALLING_EDGE   0x00020000
/** Sets DMA request on either edge.*/
#define DMA_EITHER_EDGE    0x00030000
/** Sets Interrupt when logic 0.*/
#define INTR_LOGIC0        0x00080000
/** Sets Interrupt on rising-edge.*/
#define INTR_RISING_EDGE   0x00090000
/** Sets Interrupt on falling-edge.*/
#define INTR_FALLING_EDGE  0x000A0000
/** Sets Interrupt on either edge.*/
#define INTR_EITHER_EDGE   0x000B0000
/** Sets Interrupt when logic 1.*/
#define INTR_LOGIC1        0x000C0000

typedef struct
{
	uint8_t flag_port_a : 1;
	uint8_t flag_port_b : 1;
	uint8_t flag_port_c : 1;
	uint8_t flag_port_d : 1;
	uint8_t flag_port_e : 1;
} gpio_interrupt_flags_t;

/*! This definition is used to configure whether a pin is an input or an output*/
typedef enum {GPIO_INPUT,/*!< Definition to configure a pin as input */
			  GPIO_OUTPUT /*!< Definition to configure a pin as output */
			 }gpio_port_direction_t;


/*! These constants are used to select an specific port in the different API functions*/
typedef enum{GPIO_A, /*!< Definition to select GPIO A */
			 GPIO_B, /*!< Definition to select GPIO B */
			 GPIO_C, /*!< Definition to select GPIO C */
			 GPIO_D, /*!< Definition to select GPIO D */
			 GPIO_E, /*!< Definition to select GPIO E */
			 GPIO_F  /*!< Definition to select GPIO F */
			} gpio_port_name_t;

/*! This data type is used to configure the pin control register*/
typedef const uint32_t gpio_pin_control_register_t;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function clears all interrupts that were sensed by the GPIO.

 	 \param[in]  port_name Port to clear interrupts.
 	 \return void
 	 \todo Implement a mechanism to clear interrupts by a specific pin.
 */
void GPIO_clear_interrupt(gpio_port_name_t port_name);



/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function enables the GPIO clock by configuring the corresponding bit
 	 	 and register in the System Clock Gating Control Register.

 	 \param[in]  port_name Port to be configured.
 	 \return 1 if the port_name is valid else return 0
 */
uint8_t GPIO_clock_gating(gpio_port_name_t port_name);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 This function configure different characteristics in an specific GPIO:
 	 	 pullup or pulldown resistor,slew rate, drive strength, passive filter,open drain pin,alternative functions in the GPIO
 	 \param[in] port_name Port to be configured.
 	 \param[in]  pin Specific pin to be configured.
 	 \param[in]  pinControlRegister Pointer to a constant configuration value that configures the pin characteristics. In particular this function
 	 uses the definitions GPIO_PS, GPIO_PE, GPIO_MUX1 etc. For example, in order to configure the pullup resistor ans the pin as GPIO it is need to
 	 declare the type in following way:
 	 gpio_pin_control_register_t PinControlRegister = GPIO_MUX1|GPIO_PS|GPIO_PE;
 	 \return 1 if the port_name is valid else return 0
 */
uint8_t GPIO_pin_control_register(gpio_port_name_t port_name, uint8_t pin, gpio_pin_control_register_t* pinControlRegister);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 This function configure all the GPIO port as input when 1 logic is written or output when 0 logic is written.
 	 \param[in] port_name Port to configure
 	 \param[in] direction Input value to specify the port as input or output.
 	 \return void

 */
void GPIO_data_direction_port(gpio_port_name_t port_name, uint32_t direction);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief  This function configure specific pins of a GPIO port as input when 1 logic is written or output when 0 logic is written.
 	 \param[in] port_name Port to configure.
 	 \param[in] state Value to specify if the pin behaves as input or output.
 	 \param[in] pin Input value to specify the pin number.
 	 \return void
 */
void GPIO_data_direction_pin(gpio_port_name_t port_name, uint8_t state, uint8_t pin);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function reads all the GPIO port.
 	 \param[in] port_name Port to be read.
 	 \return  It is the value read from a GPIO. It is a 32-bit value.

 */
uint32_t GPIO_read_port(gpio_port_name_t port_name);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function reads a specific GPIO pin.
	 \param[in] port_name Port to be read.
 	 \param[in] pin Pin to be read.
 	 \return This function return 0 if the value of the pin is 0 logic or 1 is the value the pin is 1 logic.
 */
uint8_t GPIO_read_pin(gpio_port_name_t port_name, uint8_t pin);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This function writes all the GPIO port.
 	 \param[in] port_name Port to be written.
 	 \param[in] data Value to be written.
 	 \return void
 */
void GPIO_write_port(gpio_port_name_t port_name, uint32_t data);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief This set an specific pin in a GPIO port, it uses GPIO_PSOR register.
 	\param[in] port_name Port to be selected.
 	\param[in] pin Pin to be set.
 	\return void
 */
void GPIO_set_pin(gpio_port_name_t port_name, uint8_t pin);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This clear an specific pin in a GPIO port, it uses GPIO_PCOR register.
 	 \param[in] port_name Selected Port.
 	 \param[in] pin Pin to be clear.
 	 \return void
 */
void GPIO_clear_pin(gpio_port_name_t port_name, uint8_t pin);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This toggle the value of a specific pin in a GPIO port, it uses GPIO_PTOR register.
 	 \param[in] port_name Selected Port.
 	 \param[in] pin Pin to be toggled.
 	 \return void
 */
void GPIO_toogle_pin(gpio_port_name_t port_name, uint8_t pin);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This funct set the flag of the ISR to FALSE.
 	 \param[in] port_name Selected Port.
 	 \return void
 */
void GPIO_clear_irq_status(gpio_port_name_t gpio);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief Returns to be read the flag of the ISR.
 	 \param[in] port_name Selected Port.
 	 \return flag_status value
 */
uint8_t GPIO_get_irq_status(gpio_port_name_t gpio);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief Callback Funct, to access other funct in an upper level.
 	 \param[in] port_name Selected Port.
 	 \param[in] funct to be called back.
 	 \return void
 */
void GPIO_callback_init(gpio_port_name_t port_name,void (*handler)(void));

#endif /* GPIO_H_ */
