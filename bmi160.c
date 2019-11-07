/*
 * bmi160.c
 *
 *  Created on: Oct 29, 2019
 *      Author: LuisFernando
 */

#include "bmi160.h"

/* declaration of static functions */
static void bmi160_init(void);
static void bmi160_normal_mode_config(void);
static void bmi160_read(void);
static void data_axis_acc(void);
static void data_axis_gyr(void);
static void convert_value_acc(int16_t *acc_axis_data);
static void bmi160_varianza(void);
static void bmi160_send_mahony(void);
static void convert_value_gyr(int16_t *gyr_axis_data);
static void vTimerCallback(TimerHandle_t xTimer);

/* Buffer to read acc & gyr*/
int8_t g_read_gyr[6] = {0};
int8_t g_read_acc[6] = {0};

int16_t g_data_axis_acc[3] = {0};
int16_t g_data_axis_gyr[3] = {0};

uint8_t header_counter = 0;

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

struct bmi160_device sensor;
struct comm_msg_acc_t acc_device;
struct comm_msg_acc_t gyr_device;
struct comm_msg_mahony_t uart_euler;
struct rtos_i2c_bmi160_package sensor_package;

static EventGroupHandle_t g_time_events;

static SemaphoreHandle_t acc_semaphore;
static SemaphoreHandle_t gyr_semaphore;
static SemaphoreHandle_t send_semaphore;
static SemaphoreHandle_t mutex1_semaphore;
static SemaphoreHandle_t mutex2_semaphore;
static SemaphoreHandle_t mutex3_semaphore;

MahonyAHRSEuler_t euler_package;

int8_t device_arr[4] = {0};

void bmi160_vtask()
{
	EventBits_t ev;
	TickType_t xDelay;

	xDelay = pdMS_TO_TICKS(200); //300 miliseconds delay

	bmi160_normal_mode_config(); //configure BMI160 with normal mode

	xEventGroupSetBits(g_time_events, mEventAccelRead);

	for(;;)
	{
		/* get event group's state */
		ev = xEventGroupGetBits(g_time_events);

		if(ev & mEventAccelRead)
		{
			xSemaphoreGive(acc_semaphore);
			xSemaphoreTake(mutex1_semaphore, portMAX_DELAY);
			xEventGroupSetBits(g_time_events, mEventGyroRead);
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

		xSemaphoreGive(send_semaphore);
		//vTaskDelay(xDelay);
		xSemaphoreTake(mutex3_semaphore, portMAX_DELAY);

	}
}

/*!
 * @brief This API is for the startup of the device, so it can run on NORMAL MODE as standard.
 * Use RTOS system.
 */
static void bmi160_normal_mode_config(void)
{
	/* Header conf*/
	acc_device.header = BMI160_HEADER;
	gyr_device.header = BMI160_HEADER;
	uart_euler.header = BMI160_HEADER;

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
	/*
	 * ACC
	 */
	/* Select the Output data rate, range of accelerometer sensor */
	sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
	/* Select the power mode of accelerometer sensor */
	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
	/*
	 * GYRO
	 */
	/* Select the Output data rate, range of Gyroscope sensor */
	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	/* Select the power mode of Gyroscope sensor */
	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	/* Sensor package*/
	/* ACC*/
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

/*!
 * @brief This API if for the read the registers corresponding to the accelerometer.
 */
void bmi160_read_acc(void)
{
	uint8_t index = 0;
	int8_t data_temp;


	for(;;)
	{
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

			g_read_acc[index] = *sensor_package.buffer;

			sensor_package.subaddr++;
		}

		data_axis_acc();
		xSemaphoreGive(mutex1_semaphore);
	}
}
/*!
 * @brief This API is for the read of the registers corresponding to the gyroscope.
 */
void bmi160_read_gyr(void)
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

/*!
 * @brief This API is for the RTOS system so it can call the read of the registers for the accelerometer & the gyroscope.
 * Use RTOS system.
 */
static void bmi160_read(void)
{
	while(TRUE)
	{
		/* Read values and stores it in the acc and gyt structure*/
		bmi160_read_acc();

		bmi160_read_gyr();

		/* Get the var of both data*/
		//bmi160_varianza();

		/* Send data to the mahony*/
		bmi160_send_mahony();
	}
}

/*!
 * @brief This API is for the conversion of the registers of 8 bits each, to 16 bits each for the accelerometer.
 * Only for reading and processing purpose.
 */
static void data_axis_acc(void)
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

/*!
 * @brief This API is for the conversion of the register of 8 bits each, to 16 bits each for the accelerometer.
 * Only for reading and processing purpose.
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

/*!
 * @brief This API is for the conversion of the value of the register corresponding to the accelerometer to floating point.
 * Processing data only
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
				/*
				 * Do Nothing
				 */
			break;
		}
	}
}


/*!
 * @brief This API is for the conversion of the value of the register corresponding to the gyroscope to floating point.
 * Processing data only
 */
static void convert_value_gyr(int16_t *gyr_axis_data)
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

/*!
 * @brief This API is used to calculate the deviation of the values so it can prevent the wrong lectures of the values.
 * Processing data only.
 */
static void bmi160_varianza(void)
{
}

/*!
 * @brief This API is used and apply of the mahony function provided by the teacher so it can read the AHRS system.
 * Use RTOS system.
 */
static void bmi160_send_mahony(void)
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
		/**/

		/* Send uart package*/
		rtos_uart_send(rtos_uart0,struct_ptr,sizeof(uart_euler));
		xSemaphoreGive(mutex3_semaphore);
	}
}

int main()
{
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

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

    /* time event instantiation **/
     g_time_events = xEventGroupCreate();

     acc_semaphore = xSemaphoreCreateBinary();
     gyr_semaphore = xSemaphoreCreateBinary();
     send_semaphore = xSemaphoreCreateBinary();
     mutex1_semaphore = xSemaphoreCreateBinary();
     mutex2_semaphore = xSemaphoreCreateBinary();
     mutex3_semaphore = xSemaphoreCreateBinary();

    /* BMI160 task creation */
    xTaskCreate(bmi160_vtask, "main task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(bmi160_read_acc, "read acc task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(bmi160_read_gyr, "read gyr task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(bmi160_send_mahony, "send task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-3, NULL);

    /* start scheduler */
    vTaskStartScheduler();

    for(;;)
    {

    }

	return 0;
}
