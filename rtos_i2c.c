
#include "rtos_i2c.h"

#include "fsl_i2c.h"
#include "fsl_clock.h"
#include "fsl_port.h"

#include "FreeRTOS.h"
#include "semphr.h"

/**Defines number of serial ports*/
#define NUMBER_OF_SERIAL_PORTS (3)

static inline void enable_i2c_clock(rtos_i2c_number_t i2c_number);
/**Enables Port Clock*/
static inline void enable_port_clock(rtos_i2c_port_t);
/**Gets the I2C to use*/
static inline I2C_Type * get_i2c_base(rtos_i2c_number_t);
/**Gets Port to use I2C*/
static inline PORT_Type * get_port_base(rtos_i2c_port_t);
/**Callback function when I2C is finished*/
static void fsl_i2c_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData);

/**
 * \brief Arguments to handle I2C operations
 */
typedef struct
{
	uint8_t is_init;								/**Flag to detect I2C initialization*/
	i2c_master_handle_t fsl_i2c_master_handle;		/**I2C callback handle*/
	SemaphoreHandle_t mutex_tx_rx;					/**Mutex to protect access through I2C*/
	SemaphoreHandle_t tx_rx_sem;					/**Binary to signal status of actions through I2C*/
}rtos_i2c_hanlde_t;

/**Array of handles to control I2C's*/
static rtos_i2c_hanlde_t i2c_handles[NUMBER_OF_SERIAL_PORTS] = {0};

/**Callback function when I2C is finished*/
static void fsl_i2c_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
	/**Callback control flag*/
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/**Returns to corresponding event if use of I2C was successful*/
	if (kStatus_Success == status)
	{
		/**Return with I2C 0*/
		if(I2C0 == base)
		{
			/**Gives binary semaphore of IC2 0*/
			xSemaphoreGiveFromISR(i2c_handles[rtos_i2c_0].tx_rx_sem, &xHigherPriorityTaskWoken);
		}

		/**Return with I2C 1*/
		else if(I2C1 == base)
		{
			/**Gives binary semaphore of IC2 1*/
			xSemaphoreGiveFromISR(i2c_handles[rtos_i2c_1].tx_rx_sem, &xHigherPriorityTaskWoken);
		}

		/**Return with I2C 2*/
		else if(I2C2 == base)
		{
			/**Gives binary semaphore of IC2 2*/
			xSemaphoreGiveFromISR(i2c_handles[rtos_i2c_2].tx_rx_sem, &xHigherPriorityTaskWoken);
		}
	}
	/**Ends callback*/
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**Initializes I2C*/
rtos_i2c_flag_t rtos_i2c_init(rtos_i2c_config_t config)
{

	/**Flag to determine if initialization was successful*/
	rtos_i2c_flag_t retval = rtos_i2c_fail;

	/**Configuration as master*/
	i2c_master_config_t fsl_config;

	port_pin_config_t config_i2c =
	{
			kPORT_PullUp,
			kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength,
			kPORT_MuxAlt5,
			kPORT_UnlockRegister, };

	/**Checks if configuration corresponds to existing port*/
	if(config.i2c_number < NUMBER_OF_SERIAL_PORTS)
	{
		/**Checks if its first time on initialize I2C*/
		if(!i2c_handles[config.i2c_number].is_init)
		{

			NVIC_SetPriority(I2C0_IRQn,5);
			/**Creates mutex to protect access through I2C*/
			i2c_handles[config.i2c_number].mutex_tx_rx = xSemaphoreCreateMutex();
			/**Creates binary to check status of I2C operation*/
			i2c_handles[config.i2c_number].tx_rx_sem = xSemaphoreCreateBinary();

			/**Enables I2C port*/
			enable_port_clock(config.port);
			enable_i2c_clock(config.i2c_number);

			PORT_SetPinConfig(get_port_base(config.port), config.SCL_pin, &config_i2c);
			PORT_SetPinConfig(get_port_base(config.port), config.SDA_pin, &config_i2c);

			/**Sets SCL pin*/
			//PORT_SetPinMux(get_port_base(config.port), config.SCL_pin, config.pin_mux);
			/**Sets SDA pin*/
			//PORT_SetPinMux(get_port_base(config.port), config.SDA_pin, config.pin_mux);

			I2C_MasterGetDefaultConfig(&fsl_config);
			fsl_config.baudRate_Bps = config.baudrate;
			I2C_MasterInit(get_i2c_base(config.i2c_number), &fsl_config, CLOCK_GetFreq(kCLOCK_BusClk));

/**Creates I2C Handle*/
			I2C_MasterTransferCreateHandle(get_i2c_base(config.i2c_number),
					&i2c_handles[config.i2c_number].fsl_i2c_master_handle, fsl_i2c_callback, NULL);

			/**Sets handles as initialized*/
			i2c_handles[config.i2c_number].is_init = 1;
			/**Returns successful initialization*/
			retval = rtos_i2c_sucess;
		}
	}
	return retval;




}

/**Transfer data through I2C*/
rtos_i2c_flag_t rtos_i2c_transfer(rtos_i2c_number_t i2c_number, uint8_t * buffer, uint16_t length, uint16_t slave_addr, uint16_t subaddr, uint8_t subsize)
{

	/**Variable to set transfer parameters*/
	i2c_master_transfer_t  xfer;
	/**Flag for successful transfer*/
	rtos_i2c_flag_t flag = rtos_i2c_fail;

	/**Check if I2C was initialized first*/
	if(i2c_handles[i2c_number].is_init)
	{
		xfer.data = buffer;						/**Data to transfer*/
		xfer.dataSize = length;					/**Length of data*/
		xfer.direction = kI2C_Write;			/**Set operation as Write*/
		xfer.slaveAddress = slave_addr;			/**Slave address*/
		xfer.subaddress = subaddr;				/**Salve sub address*/
		xfer.subaddressSize = subsize;			/**Sub address size*/
		xfer.flags = kI2C_TransferDefaultFlag;	/**Transfer flag*/

		/**Mutex to protect transfer (start)*/
		xSemaphoreTake(i2c_handles[i2c_number].mutex_tx_rx, portMAX_DELAY);

		/**Transfer data through I2C*/
		I2C_MasterTransferNonBlocking(get_i2c_base(i2c_number), &i2c_handles[i2c_number].fsl_i2c_master_handle, &xfer);
		/**Semaphore to wait successful transfer*/
		//while(!g_flag);
		//g_flag = 0;
		xSemaphoreTake(i2c_handles[i2c_number].tx_rx_sem, portMAX_DELAY);

		/**Mutex to protect transfer (end)*/
		xSemaphoreGive(i2c_handles[i2c_number].mutex_tx_rx);

		/**Successful transfer*/
		flag = rtos_i2c_sucess;
	}
	/**Return result of operation*/
	return flag;

}

/**Receives data from I2C*/
rtos_i2c_flag_t rtos_i2c_receive(rtos_i2c_number_t i2c_number, uint8_t * buffer, uint16_t length, uint16_t slave_addr, uint16_t subaddr, uint8_t subsize)
{

	/**Variable to set transfer parameters*/
	i2c_master_transfer_t  xfer;
	/**Flag for successful reception*/
	rtos_i2c_flag_t flag = rtos_i2c_fail;

	/**Check if I2C was initialized first*/
	if(i2c_handles[i2c_number].is_init)
	{
		xfer.data = buffer;						/**Data to transfer*/
		xfer.dataSize = length;					/**Length of data*/
		xfer.direction = kI2C_Read;				/**Set operation as Write*/
		xfer.slaveAddress = slave_addr;			/**Slave address*/
		xfer.subaddress = subaddr;				/**Salve sub address*/
		xfer.subaddressSize = subsize;			/**Sub address size*/
		xfer.flags = kI2C_TransferDefaultFlag;	/**Transfer flag*/

		/**Mutex to protect transfer (start)*/
		xSemaphoreTake(i2c_handles[i2c_number].mutex_tx_rx, portMAX_DELAY);

		/**Transfer data through I2C*/
		I2C_MasterTransferNonBlocking(get_i2c_base(i2c_number), &i2c_handles[i2c_number].fsl_i2c_master_handle, &xfer);
		/**Semaphore to wait successful transfer*/
		xSemaphoreTake(i2c_handles[i2c_number].tx_rx_sem, portMAX_DELAY);

		/**Mutex to protect transfer (end)*/
		xSemaphoreGive(i2c_handles[i2c_number].mutex_tx_rx);

		/**Successful transfer*/
		flag = rtos_i2c_sucess;
	}
	/**Return result of operation*/
	return flag;

}

static inline void enable_i2c_clock(rtos_i2c_number_t i2c_number)
{
	/**I2C to use*/

	switch(i2c_number)
	{
	/**I2C 0*/
	case rtos_i2c_0:
		CLOCK_EnableClock(kCLOCK_I2c0);
		break;
		/**I2C 1*/
	case rtos_i2c_1:
		CLOCK_EnableClock(kCLOCK_I2c1);
		break;
		/**I2C 2*/
	case rtos_i2c_2:
		CLOCK_EnableClock(kCLOCK_I2c2);
		break;
	}
}

/**Enables Port Clock*/
static inline void enable_port_clock(rtos_i2c_port_t port)
{
	/**Selects Port*/
	switch(port)
	{
	/**Port A*/
	case rtos_i2c_portA:
		CLOCK_EnableClock(kCLOCK_PortA);
		break;
		/**Port B*/
	case rtos_i2c_portB:
		CLOCK_EnableClock(kCLOCK_PortB);
		break;
		/**Port C*/
	case rtos_i2c_portC:
		CLOCK_EnableClock(kCLOCK_PortC);
		break;
		/**Port D*/
	case rtos_i2c_portD:
		CLOCK_EnableClock(kCLOCK_PortD);
		break;
		/**Port E*/
	case rtos_i2c_portE:
		CLOCK_EnableClock(kCLOCK_PortE);
		break;
	}
}

/**Gets the I2C to use*/
static inline I2C_Type * get_i2c_base(rtos_i2c_number_t i2c_number)
{
	/**I2C to use*/
	I2C_Type * retval = I2C0;
	switch(i2c_number)
	{
	/**I2C 0*/
	case rtos_i2c_0:
		retval = I2C0;
		break;
		/**I2C 1*/
	case rtos_i2c_1:
		retval = I2C1;
		break;
		/**I2C 2*/
	case rtos_i2c_2:
		retval = I2C2;
		break;
	}
	/**Returns I2C to use*/
	return retval;
}

/**Gets Port to use I2C*/
static inline PORT_Type * get_port_base(rtos_i2c_port_t port)
{
	/**Variable to get Port*/
	PORT_Type * port_base = PORTA;
	switch(port)
	{
	/**Port A*/
	case rtos_i2c_portA:
		port_base = PORTA;
		break;
		/**Port B*/
	case rtos_i2c_portB:
		port_base = PORTB;
		break;
		/**Port C*/
	case rtos_i2c_portC:
		port_base = PORTC;
		break;
		/**Port D*/
	case rtos_i2c_portD:
		port_base = PORTD;
		break;
		/**Port E*/
	case rtos_i2c_portE:
		port_base = PORTE;
		break;
	}
	/**Returns port to use*/
	return port_base;
}
