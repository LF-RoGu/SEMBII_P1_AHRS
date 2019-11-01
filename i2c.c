/**
	\file 	  i2c.c
	\brief
			  This is the header file for the i2c device driver for Kinetis K64.
	\authors: César Villarreal Hernández, ie707560
	          José Luis Rodríguez Gutierrez,ie705694
	\date	  10/05/2019
 */

#include <i2c.h>

void I2C0_IRQHandler()
{
	/*Interrupt Flag
	 * This bit sets when an interrupt is pending
	 * 0 No interrupt pending
	 * 1 Interrupt pending
	 * */
	I2C0->S |= I2C_S_IICIF_MASK;
}

void I2C_init(i2c_channel_t channel, uint32_t system_clock, uint16_t baud_rate)
{
	/*PCR config*/
	static const gpio_pin_control_register_t i2c_config = GPIO_MUX2|GPIO_PS;

	SIM->SCGC5 = SIM_SCGC5_PORTB_MASK;

	/*Set the configuration for i2c_config_Tx*/
	GPIO_pin_control_register(GPIO_B, bit_2, &i2c_config);
	/*Set the configuration for i2c_config_Rx*/
	GPIO_pin_control_register(GPIO_B, bit_3, &i2c_config);
	/*Set the config fot the pin that it will send the i2c protocol*/
	GPIO_data_direction_pin(GPIO_B,GPIO_OUTPUT,bit_2);
	/*Set the config fot the pin that it will receive the i2c protocol*/
	GPIO_data_direction_pin(GPIO_B,GPIO_INPUT,bit_3);

	uint32_t valueSCL;

	/*I2C baud rate = I2C module clock speed (Hz)/(mul × SCL divider)*/
	valueSCL = system_clock/(baud_rate*MULT);

	switch(channel)
	{
	case I2C_0:
		SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
		/*ClockRate*/
		I2C0->F = I2C_F_ICR(valueSCL);
		/*Multiplier Factor
		 * 00 mul = 1
		 * 01 mul = 2
		 * 10 mul = 4
		 * 11 Reserved
		 * */
		I2C0->F = I2C_F_MULT(2);
		/*Enable I2C module*/
		I2C0->C1 = I2C_C1_IICEN_MASK;
		break;
	case I2C_1:
		SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;
		/*ClockRate*/
		I2C1->F = I2C_F_ICR(valueSCL);
		/*Multiplier Factor
		 * 00 mul = 1
		 * 01 mul = 2
		 * 10 mul = 4
		 * 11 Reserved
		 * */
		I2C1->F = I2C_F_MULT(2);
		/*Enable I2C mo0dule*/
		I2C1->C1 = I2C_C1_IICEN_MASK;
		break;
	case I2C_2:
		SIM->SCGC1 |= SIM_SCGC1_I2C2_MASK;
		/*ClockRate*/
		I2C2->F = I2C_F_ICR(valueSCL);
		/*Multiplier Factor
		 * 00 mul = 1
		 * 01 mul = 2
		 * 10 mul = 4
		 * 11 Reserved
		 * */
		I2C2->F = I2C_F_MULT(2);
		/*Enable I2C module*/
		I2C2->C1 = I2C_C1_IICEN_MASK;
		break;
	default:
		break;
	}
}
uint8_t I2C_busy()
{
	/*In this case, the I2C is free*/
	if (FALSE == (I2C0->S & I2C_S_BUSY_MASK)) {
		return TRUE; //I2C is idle
	}
	/*In this case the I2C is busy*/
	else {
		return FALSE; //I2C is busy
	}
}
void I2C_mst_or_slv_mode(uint8_t mst_or_slv)
{
	if(TRUE == mst_or_slv)
	{
		/*Master mode*/
		I2C0->C1 |= I2C_C1_MST_MASK;
	}
	else
	{
		/*Slave mode*/
		I2C0->C1 &= ~(I2C_C1_MST_MASK);
	}
}
void I2C_tx_rx_mode(uint8_t tx_or_rx)
{
	if(FALSE == tx_or_rx)
	{
		/*Recieve mode*/
		I2C0->C1 &= ~I2C_C1_TX_MASK;
		I2C0->C1 &= ~I2C_C1_TXAK_MASK;
	}
	else
	{
		/*Transmit mode*/
		I2C0->C1 |= I2C_C1_TX_MASK;
	}
}
void I2C_nack(void)
{
	/*Mask for not acknowledge*/
	I2C0->C1 |= I2C_C1_TXAK_MASK;
}
void I2C_repeted_start(void)
{
	/*Repeat START
	 *Writing 1 to this bit generates a repeated START condition provided it is the current master.*/
	I2C0->C1 |= I2C_C1_RSTA_MASK;
}
void I2C_write_byte(uint8_t data)
{
	/*Data I/O register
	 * In master transmit mode, when data is written to this register, a data transfer is initiated.*/
	I2C0->D = data;
}
uint8_t I2C_read_byte(void)
{
	/*The value in the register ->D is send to a variable*/
	return (I2C0->D);
}
void I2C_start(void)
{
	/*Generate the Start Signal*/
	I2C0->C1 |= I2C_C1_TX_MASK;
	I2C0->C1 |= I2C_C1_MST_MASK;
}
void I2C_stop(void)
{
	/*Generates the Stop Signal*/
	I2C0->C1 &= ~I2C_C1_TX_MASK;
	I2C0->C1 &= ~I2C_C1_MST_MASK;
}
void I2C_wait(void)
{
	/*Waits until the process TCF(Transfer Complete Flag) changes*/
	while(FALSE == ((I2C0->S && I2C_S_IICIF_MASK)));
	I2C0->S |= I2C_S_IICIF_MASK;
}
uint8_t I2C_get_ack(void)
{
	/*It returns if there is Acknowledge or not*/
	while(FALSE != (I2C0->S & I2C_S_RXAK_MASK));
}
