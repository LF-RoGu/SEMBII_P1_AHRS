/*
	\file     bmi160.c
	\brief    This is the source file for the BMI160 driver
	
	\authors: César Villarreal Hernández, ie707560
	          Luis Fernando Rodríguez Gutiérrez,ie705694
	\date	  06/11/2019
 */

#include "bmi160.h"

/* Buffer to read acc & gyr*/
static int8_t g_read_gyr[6] = {0};
static int8_t g_read_acc[6] = {0};

static int16_t g_data_axis_acc[3] = {0};
static int16_t g_data_axis_gyr[3] = {0};

static struct bmi160_device sensor;
static struct comm_msg_acc_t acc_device;
static struct comm_msg_acc_t gyr_device;
static struct comm_msg_mahony_t uart_euler;
static struct rtos_i2c_bmi160_package sensor_package;

static EventGroupHandle_t g_time_events;

static SemaphoreHandle_t acc_semaphore;
static SemaphoreHandle_t gyr_semaphore;
static SemaphoreHandle_t send_semaphore;
static SemaphoreHandle_t mutex1_semaphore;
static SemaphoreHandle_t mutex2_semaphore;
static SemaphoreHandle_t mutex3_semaphore;

MahonyAHRSEuler_t euler_package;

int8_t device_arr[4] = {0};

/*
 	 \brief	    This function works as the control task for the operation of the BMI160.
 	 \param[in]
 	 \return    void
 */
void BMI160_CONTROL_TASK()
{
	EventBits_t ev;
	TickType_t xDelay;

	xDelay = pdMS_TO_TICKS(300); //300 miliseconds delay

	bmi160_normal_mode_config(); //configure BMI160 with normal mode

	xEventGroupSetBits(g_time_events, mEventAccelRead); //force acceleration read event

	for(;;)
	{
		/* get event group's state */
		ev = xEventGroupGetBits(g_time_events);

		if(ev & mEventAccelRead)
		{
			/* release semaphore for acceleration reading */
			xSemaphoreGive(acc_semaphore);
			/* use another semaphore to ensure that the data is read */
			xSemaphoreTake(mutex1_semaphore, portMAX_DELAY);
			/* trigger gyroscope read event */
			xEventGroupSetBits(g_time_events, mEventGyroRead);
			/* clear acceleration read event */
			xEventGroupClearBits(g_time_events, mEventAccelRead);
		}
		else
		{

		}

		if(ev & mEventGyroRead)
		{
			xSemaphoreGive(gyr_semaphore);
			xSemaphoreTake(mutex2_semaphore, portMAX_DELAY);
			xEventGroupSetBits(g_time_events, mEventAccelRead);
			xEventGroupClearBits(g_time_events, mEventGyroRead);
		}
		else
		{

		}

		/* release semaphore for reading data */
		xSemaphoreGive(send_semaphore);
		//vTaskDelay(xDelay);
		xSemaphoreTake(mutex3_semaphore, portMAX_DELAY);

	}
}

/*
 	 \brief	     
 	 \param[in]     
 	 \return     
 */
void BMI160_READ_ACC_TASK(void)
{
	uint8_t index;
	int8_t data_temp;

	index = 0;

	for(;;)
	{
		/* wait until semaphore event is called */
		xSemaphoreTake(acc_semaphore, portMAX_DELAY);

		/* Sensor package*/
		/* ACC*/
		sensor_package.i2c_number = rtos_i2c_0;
		sensor_package.buffer = &data_temp;
		sensor_package.length = BMI160_DATA_SIZE;
		sensor_package.slave_addr = BMI160_I2C_ADDR;
		sensor_package.subaddr = BMI160_READ_ACC_X_L;
		sensor_package.subsize = BMI160_SUBADDRESS_SIZE;

		for(index = X_LOW; index <= Z_HIGH; index++)
		{
			rtos_i2c_receive
			(
				sensor_package.i2c_number,
				sensor_package.buffer,
				sensor_package.length,
				sensor_package.slave_addr,
				sensor_package.subaddr,
				sensor_package.subsize
			);

			//BMI160_RECEIVE_PKG();

			g_read_acc[index] = *sensor_package.buffer;

			sensor_package.subaddr++;
		}

		data_axis_acc();
		xSemaphoreGive(mutex1_semaphore);
	}
}


void BMI160_RECEIVE_PKG()
{
	rtos_i2c_receive
	(
		sensor_package.i2c_number,
		sensor_package.buffer,
		sensor_package.length,
		sensor_package.slave_addr,
		sensor_package.subaddr,
		sensor_package.subsize
	);
}

/*
 	 \brief	     
 	 \param[in]     
 	 \return     
 */
void BMI160_READ_GYRO_TASK(void)
{
	uint8_t index = 0;

	int8_t data_temp;

	for(;;)
	{
		xSemaphoreTake(gyr_semaphore, portMAX_DELAY);
		/* Sensor package*/
		/* ACC*/
		sensor_package.i2c_number = rtos_i2c_0;
		sensor_package.buffer = &data_temp;
		sensor_package.length = BMI160_DATA_SIZE;
		sensor_package.slave_addr = BMI160_I2C_ADDR;
		sensor_package.subaddr = BMI160_READ_GYR_X_L;
		sensor_package.subsize = BMI160_SUBADDRESS_SIZE;

		for(index = X_LOW; index <= Z_HIGH; index++)
		{
			rtos_i2c_receive(
							sensor_package.i2c_number,
							sensor_package.buffer,
							sensor_package.length,
							sensor_package.slave_addr,
							sensor_package.subaddr,
							sensor_package.subsize);

			g_read_gyr[index] = *sensor_package.buffer;

			sensor_package.subaddr++;
		}
	/*
	 *
	 */
	data_axis_gyr();
	xSemaphoreGive(mutex2_semaphore);
	}
}

/*
 	 \brief	     
 	 \param[in]     
 	 \return     
 */
void bmi160_normal_mode_config(void)
{
	/* Configure the header for all devices */
	acc_device.header = BMI160_HEADER;
	gyr_device.header = BMI160_HEADER;
	uart_euler.header = BMI160_HEADER;

	/* Generate a soft reset */
	/* SOFT RESET*/
	sensor_package.i2c_number = rtos_i2c_0;
	sensor_package.buffer =  BMI160_SOFT_RESET_CMD;
	sensor_package.length = BMI160_DATA_SIZE;
	sensor_package.slave_addr = BMI160_I2C_ADDR;
	sensor_package.subaddr = BMI160_COMMAND_REG_ADDR;
	sensor_package.subsize = BMI160_SUBADDRESS_SIZE;

	rtos_i2c_transfer
	(
		sensor_package.i2c_number,
		sensor_package.buffer,
		sensor_package.length,
		sensor_package.slave_addr,
		sensor_package.subaddr,
		sensor_package.subsize
	);

	/* Set accelerometer's configuration for normal mode operation */
	sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;     //accelerometer's output data rate
	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;     //accelerometer's range (in g's)
	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;  //accelerometer's power mode

	/* Set gyroscope's configuration for normal mode operation */
	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;       //gyroscope's output data rate
	sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS; //gyroscope's range (in g's)
	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;    //gyroscope's power mode

	/* Sensor package configuration */
	sensor_package.i2c_number = rtos_i2c_0;
	sensor_package.buffer = &sensor.accel_cfg.power;
	sensor_package.length = BMI160_DATA_SIZE;
	sensor_package.slave_addr = BMI160_I2C_ADDR;
	sensor_package.subaddr = BMI160_COMMAND_REG_ADDR;
	sensor_package.subsize = BMI160_SUBADDRESS_SIZE;

	/* SET POWER MODE*/
	rtos_i2c_transfer
	(
		sensor_package.i2c_number,
		sensor_package.buffer,
		sensor_package.length,
		sensor_package.slave_addr,
		sensor_package.subaddr,
		sensor_package.subsize
	);

	/* GYR*/
	sensor_package.i2c_number = rtos_i2c_0;
	sensor_package.buffer = &sensor.gyro_cfg.power;
	sensor_package.length = BMI160_DATA_SIZE;
	sensor_package.slave_addr = BMI160_I2C_ADDR;
	sensor_package.subaddr = BMI160_COMMAND_REG_ADDR;
	sensor_package.subsize = BMI160_SUBADDRESS_SIZE;

	/* SET POWER MODE*/
	rtos_i2c_transfer
	(
		sensor_package.i2c_number,
		sensor_package.buffer,
		sensor_package.length,
		sensor_package.slave_addr,
		sensor_package.subaddr,
		sensor_package.subsize
	);
}

/*
 	 \brief	     
 	 \param[in]     
 	 \return     
 */
void data_axis_acc(void)
{
	int16_t data_temp;

	/* Data axis X*/
	data_temp = ((uint16_t) g_read_acc[X_LOW] << 8) | g_read_acc[X_HIGH];
	g_data_axis_acc[X_REG] = data_temp;
	/* clear temp*/
	data_temp = 0;

	/* Data axis Y*/
	data_temp = ((uint16_t)g_read_acc[Y_LOW] << 8) | g_read_acc[Y_HIGH];
	g_data_axis_acc[Y_REG] = data_temp;
	/* clear temp*/
	data_temp = 0;

	/* Data axis Z*/
	data_temp = ((uint16_t)g_read_acc[Z_LOW] << 8) | g_read_acc[Z_HIGH];
	g_data_axis_acc[Z_REG] = data_temp;
	/* clear temp*/
	data_temp = 0;

	/*
	 * Call for the function so it can get the values to floating point.
	 */
	convert_value_acc(g_data_axis_acc);
}

/*
 	 \brief	     
 	 \param[in]     
 	 \return     
 */
void data_axis_gyr(void)
{
	int16_t data_temp;

	/* Data axis X*/
	data_temp = ((int16_t) g_read_gyr[X_LOW] << 8) | g_read_gyr[X_HIGH];

	g_data_axis_gyr[X_REG] = data_temp;
	/* clear temp*/
	data_temp = 0;
	/* Data axis Y*/
	data_temp = ((int16_t)g_read_gyr[Y_LOW] << 8) | g_read_gyr[Y_HIGH];

	g_data_axis_gyr[Y_REG] = data_temp;
	/* clear temp*/
	data_temp = 0;
	/* Data axis Z*/
	data_temp = ((int16_t)g_read_gyr[Z_LOW] << 8) | g_read_gyr[Z_HIGH];

	g_data_axis_gyr[Z_REG] = data_temp;
	/* clear temp*/
	data_temp = 0;

	/*
	 * Call for teh function so it can get the values to floating point.
	 */

	convert_value_gyr(g_data_axis_gyr);
}

/*
 	 \brief	     
 	 \param[in]     
 	 \return     
 */
void convert_value_acc(int16_t *acc_axis_data)
{
	uint8_t index;

	for(index = X_REG; index <= Z_REG; index++)
	{
		switch(index)
		{
			case X_REG:
				acc_device.x = (BMI160_ACC_UNITS_RANGE)*acc_axis_data[X_REG]/BMI160_MAX_VALUE;
			break;
			case Y_REG:
				acc_device.y = (BMI160_ACC_UNITS_RANGE)*acc_axis_data[Y_REG]/BMI160_MAX_VALUE;
			break;
			case Z_REG:
				acc_device.z = (BMI160_ACC_UNITS_RANGE)*acc_axis_data[Z_REG]/BMI160_MAX_VALUE;
			break;
			default:
			break;
		}
	}
}

/*
 	 \brief	     
 	 \param[in]     
 	 \return     
 */
void convert_value_gyr(int16_t *gyr_axis_data)
{
	uint8_t index;

	for(index = X_REG; index <= Z_REG; index++)
	{
		switch(index)
		{
			case X_REG:
				gyr_device.x = (BMI160_GYR_UNITS_RANGE)*gyr_axis_data[X_REG]/BMI160_MAX_VALUE;
			break;
			case Y_REG:
				gyr_device.y = (BMI160_GYR_UNITS_RANGE)*gyr_axis_data[Y_REG]/BMI160_MAX_VALUE;
			break;
			case Z_REG:
				gyr_device.z = (BMI160_GYR_UNITS_RANGE)*gyr_axis_data[Z_REG]/BMI160_MAX_VALUE;
			break;
			default:
				/*
				 * Do Nothing
				 */
			break;
		}
	}
}

/*
 	 \brief	     
 	 \param[in]     
 	 \return     
 */
void bmi160_varianza(void)
{
}

/*
 	 \brief	     
 	 \param[in]     
 	 \return     
 */
void BMI160_SERIAL_COMM_TASK(void)
{
	uint8_t* struct_ptr;

	for(;;)
	{
		xSemaphoreTake(send_semaphore, portMAX_DELAY);

		euler_package = MahonyAHRSupdateIMU(gyr_device.x,gyr_device.y,gyr_device.z,acc_device.x,acc_device.y,acc_device.z);

		uart_euler.pitch_mahony = euler_package.pitch;
		uart_euler.roll_mahony = euler_package.roll;
		uart_euler.yaw_mahony = euler_package.yaw;

		struct_ptr = &uart_euler;

		/* Send uart package*/
		rtos_uart_send(rtos_uart0,struct_ptr,sizeof(uart_euler));
		xSemaphoreGive(mutex3_semaphore);
	}
}

/*
 	 \brief	     
 	 \param[in]     
 	 \return     
 */
void BMI160_init()
{
    rtos_i2c_config_t i2c_config_t;
	rtos_uart_config_t uart_config_t;

	/* configuring i2c protocol for the bmi160 */
    i2c_config_t.baudrate = RTOS_I2C_BAUDRATE;
    i2c_config_t.i2c_number = rtos_i2c_0;
    i2c_config_t.port = rtos_i2c_portE;
    i2c_config_t.SDA_pin = bit_25;
    i2c_config_t.SCL_pin = bit_24;
    i2c_config_t.pin_mux = rtos_mux_alt05;
    rtos_i2c_init(i2c_config_t);

	/* configuring uart protocol for the bmi160 */
    uart_config_t.uart_number = rtos_uart0;
    uart_config_t.baudrate = BD_115200;
    uart_config_t.port = rtos_uart_portB;
    uart_config_t.tx_pin = bit_17;
    uart_config_t.rx_pin = bit_16;
    uart_config_t.pin_mux = rtos_mux_alt3;
    rtos_uart_init(uart_config_t);

    /* create event group */
     g_time_events = xEventGroupCreate();
	/* creating semaphore for the accelerometer's reading */
     acc_semaphore = xSemaphoreCreateBinary();
	 /* creating semaphore for the gyroscope's reading */
     gyr_semaphore = xSemaphoreCreateBinary();
	 /* creating semaphore for serial communication */
     send_semaphore = xSemaphoreCreateBinary();
	 /* creating mutex semaphore 1 */
     mutex1_semaphore = xSemaphoreCreateBinary();
	 /* creating mutex semaphore 2 */
     mutex2_semaphore = xSemaphoreCreateBinary();
	 /* creating mutex semaphore 3 */
     mutex3_semaphore = xSemaphoreCreateBinary();
}

