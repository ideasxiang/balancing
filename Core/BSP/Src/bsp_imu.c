/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.c
 * @brief      mpu6500 module driver, configurate MPU6500 and Read the Accelerator
 *             and Gyrometer data using SPI interface
 * @note
 * @Version    V1.0.0
 * @Date       Jan-30-2018
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

/**	Notes for Devboard A IMU + Magnetometers
 *
 * Dev board A uses the MPU6500 6 axis IMU by Invensense (no longer continued)
 * It also includes a Magnetometer IST8310, connected to the devboard through the
 * IMU's I2C Aux port
 *
 * Pin mappings to Devboard A's STM427
 * CS			PF6		SPI5_NSS		slave select port
 * SCK			PF7		SPI5_SCK		replaced with a timer on DJI's devC code...why?
 * MISO			PF8   	SPI5_MISO		miso soup? :D
 * MOSI			PF9		SPI5_MOSI
 * MPU INT		PB8		IMU DRDY INT	IMU sends a signal here when data is ready
 * IST INT		PE3		IST DRDY INT	Magnetometer sends a signal here when data is ready
 * Heater		PB5		TIM3_CH2 		for temperature control, it's literally a PWM resistive heater
 *
 * Relevant registers
 * EVERYTHING
 *
 *
 *
 *
 */
#include "board_lib.h"
#include "bsp_imu.h"
#include "ist8310_reg.h"
#include "mpu6500_reg.h"
#include "imu_processing_task.h"

#define IMU_HSPI			hspi1
#define IST_I2C				hi2c3
//#define BOARD_DOWN (1)
#define IMU_ORIENTATION 1
//#define IST8310
#define MPU_HSPI hspi5
#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

#define ACCEL_MAX_RANGE		32768
#define ACCEL_MAX_GS		6
#define GYRO_MAX_RANGE 		32768
#define GYRO_MAX_W			1000

static imu_raw_t imu_data;
static uint8_t imu_init_status = 0;
//																															*/
//volatile float        	q0 = 1.0f;
//volatile float       	q1 = 0.0f;
//volatile float        	q2 = 0.0f;
//volatile float        	q3 = 0.0f;
//volatile float        	exInt, eyInt, ezInt;                   /* error integral */
//static volatile float 	gx, gy, gz, ax, ay, az, mx, my, mz;
//volatile uint32_t     	last_update, now_update;               /* Sampling cycle count, ubit ms */
//static uint8_t        	tx, rx;
//static uint8_t        	tx_buff[14] = { 0xff };
uint8_t               	mpu_buff[14];                          /* buffer to save imu raw data */
uint8_t               	ist_buff[6];                           /* buffer to save IST8310 raw data */
//imu_data_t            	mpu_data;
//imu_t				  	imu_heading={0};
//uint8_t					data_ready_flag;
//uint8_t					imu_init_status = 0;
//int32_t					gx_offset = 0;
//int32_t					gy_offset = 0;
//int32_t					gz_offset = 0;
//extern osEventFlagsId_t	gimbal_data_flag;


/**
  * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
  * @param  x: the number need to be calculated
  * @retval 1/Sqrt(x)
  * @usage  call in imu_ahrs_update() function
  */
float inv_sqrt(float x)
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;

	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}

/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval
  * @usage  call in ist_reg_write_by_mpu(),
  *                 ist_reg_read_by_mpu(),
  *                 mpu_master_i2c_auto_read_config(),
  *                 ist8310_init(),
  *                 mpu_set_gyro_fsr(),
  *                 mpu_set_accel_fsr(),
  *                 mpu_device_init() function
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;
	uint8_t tx, rx;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 100);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 100);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval
  * @usage  call in ist_reg_read_by_mpu(),
  *                 mpu_device_init() function
  */
uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
	uint8_t rx, tx;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1,100);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1 , 100);
    MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval
  * @usage  call in ist8310_get_data(),
  *                 mpu_get_data(),
  *                 mpu_offset_call() function
  */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
	uint8_t tx, rx;
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1,100);
    HAL_SPI_Receive(&MPU_HSPI, pData, len,100);
    MPU_NSS_HIGH;
    return 0;
}

uint8_t mpu_read_bytes_DMA(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
	uint8_t tx;
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
    if (HAL_SPI_Transmit(&MPU_HSPI, &tx , 1, 100) != HAL_OK)
    {}
    if (HAL_SPI_Receive_DMA(&MPU_HSPI, pData, len) != HAL_OK)
    {
        MPU_NSS_HIGH;}
    return 0;
}

/**
  * @brief  write IST8310 register through MPU6500's I2C master
  * @param  addr: the address to be written of IST8310's register
  *         data: data to be written
  * @retval
  * @usage  call in ist8310_init() function
  */
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* turn off slave 1 at first */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    /* turn on slave 1 with one byte transmitting */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    MPU_DELAY(10);
}

/**
	* @brief  write IST8310 register through MPU6500's I2C Master
	* @param  addr: the address to be read of IST8310's register
	* @retval
  * @usage  call in ist8310_init() function
	*/
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

/**
	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
* @param    device_address: slave device address, Address[6:0]
	* @retval   void
	* @note
	*/
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /*
	   * configure the device address of the IST8310
     * use slave1, auto transmit single measure mode
	   */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    /* use slave0,auto read data */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    /* every eight mpu6500 internal samples one i2c master read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    /* enable slave 0 and 1 access delay */
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    /* enable slave 1 auto transmit */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    MPU_DELAY(6);
    /* enable slave 0 with data_num bytes reading */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}

/**
	* @brief  Initializes the IST8310 device
	* @param
	* @retval
  * @usage  call in mpu_device_init() function
	*/
uint8_t ist8310_init()
{
	  /* enable iic master mode */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    MPU_DELAY(10);
	  /* enable iic 400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d);
    MPU_DELAY(10);

    /* turn on slave 1 for ist write and slave 4 to ist read */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    MPU_DELAY(10);

    /* IST8310_R_CONFB 0x01 = device rst */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

		/* soft reset */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);

		/* config as ready mode to access register */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    MPU_DELAY(10);

		//interrupts enabled
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x0C);		//1100
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x0C)	//enables both DRDY and active HIGH interrupt
        return 3;
    MPU_DELAY(10);

    /* config low noise mode, x,y,z axis 16 time 1 avg */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    MPU_DELAY(10);

    /* Set/Reset pulse duration setup,normal mode */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    MPU_DELAY(10);

    /* turn off slave1 & slave 4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);



    /* configure and turn on slave 0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}

/**
	* @brief  get the data of IST8310
  * @param  buff: the buffer to save the data of IST8310
	* @retval
  * @usage  call in mpu_get_data() function
	*/
void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
}


/**
	* @brief  get the data of imu
  * @param
	* @retval
  * @usage  call in main() function
	*/
void mpu_get_data()
{
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
}


/**
  * @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
  * @retval
  * @usage  call in mpu_device_init() function
  */
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}


/**
  * @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
  * @retval
  * @usage  call in mpu_device_init() function
  */
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3);
}


void reset_imu_data()
{
}

uint8_t id;

/**
  * @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param
  * @retval
  * @usage  call in main() function
	*/
uint8_t imu_config(void)
{
	MPU_DELAY(100);

	id                               = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t i                        = 0;
	uint8_t MPU6500_Init_Data[9][2] = {{ MPU6500_PWR_MGMT_1, 		0x03 },		// Clock Source - auto select???
										{ MPU6500_PWR_MGMT_2, 		0x00 },     // Enable Acc & Gyro
										{ MPU6500_CONFIG, 			0x01 },		// Gyro @ 250Hz, temp at 4000Hz try 0x07 for gyro @ 3600hz/
										{ MPU6500_GYRO_CONFIG, 		0x18 },		// Gyro +-2000dps */
										{ MPU6500_ACCEL_CONFIG, 	0x10 },		// +-8G */
										{ MPU6500_ACCEL_CONFIG_2, 	0x01 },		// Accl LPF 2 @ 460Hz */
										{ MPU6500_USER_CTRL, 		0x20 },		// Enable AUX */
										{ MPU6500_INT_PIN_CFG,		0x00 },		// Reset interrupt settings
										{ MPU6500_INT_ENABLE,		0x01 },		// Enable interrupts on data ready
										};
	mpu_write_byte(MPU6500_PWR_MGMT_1, 0x80);			//Reset device
	HAL_Delay(200);
	mpu_write_byte(MPU6500_PWR_MGMT_1, 0x80);			//Reset device
	HAL_Delay(200);
	mpu_write_byte(MPU6500_PWR_MGMT_1, 0x80);			//Reset device
	HAL_Delay(200);
	mpu_write_byte(MPU6500_SIGNAL_PATH_RESET, 0x07);	//Signal path reset
	HAL_Delay(200);
	//write configuration registers
	for (i = 0; i < 9; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		MPU_DELAY(2);
	}

	mpu_set_gyro_fsr(3);
	mpu_set_accel_fsr(2);

	ist8310_init();
	mpu_offset_call();
	reset_imu_data();
	imu_init_status = 1;
	return 0;
}

/**
	* @brief  get the offset data of MPU6500
  * @param
	* @retval
  * @usage  call in main() function
	*/
void mpu_offset_call(void)
{
	int i;
	for (i=0; i<300;i++)
	{
		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
		imu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
		imu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		imu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

		MPU_DELAY(2);
	}
	imu_data.gx_offset= imu_data.gx_offset / 300;
	imu_data.gy_offset= imu_data.gy_offset / 300;
	imu_data.gz_offset= imu_data.gz_offset / 300;
}



void get_imu_data(uint16_t trig_pin)
{
	if (imu_init_status == 1)
	{
		mpu_get_data();
		process_ist_data();
		process_mpu_data();
	}
}

void process_ist_data()
{
	memcpy(&imu_data.mag_data.mx, ist_buff, 6);
}

void process_mpu_data()
{
	int16_t mpu_temp;
	mpu_temp   			= mpu_buff[0] << 8 | mpu_buff[1];
    imu_data.accel_data.ax 	= mpu_temp * 1;
	mpu_temp   			= mpu_buff[2] << 8 | mpu_buff[3];
    imu_data.accel_data.ay 	= mpu_temp * 1;
	mpu_temp   			= mpu_buff[4] << 8 | mpu_buff[5];
    imu_data.accel_data.az 	= mpu_temp * 1;
	//temperature = mpu_buff[6] << 8 | mpu_buff[7];
    accel_data_ready(imu_data.accel_data);

	mpu_temp = ((mpu_buff[8]  << 8 | mpu_buff[9])  - imu_data.gx_offset);
	imu_data.gyro_data.gx = mpu_temp/ 16.384f / 57.3f;
	mpu_temp = ((mpu_buff[10] << 8 | mpu_buff[11]) - imu_data.gy_offset);
	imu_data.gyro_data.gy = mpu_temp/ 16.384f / 57.3f;
	mpu_temp = ((mpu_buff[12] << 8 | mpu_buff[13]) - imu_data.gz_offset);
	imu_data.gyro_data.gz = mpu_temp/ 16.384f / 57.3f;
	gyro_data_ready(imu_data.gyro_data);
	//imu_heading.temp = 21 + mpu_data.temp / 333.87f;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if((GPIO_Pin == IST_INT_Pin) || (GPIO_Pin == MPU_INT_Pin))
	{
		get_imu_data(GPIO_Pin);
		//todo: add semaphore so gimbals only react if there's a new IMU reading
	}
}

//void imu_start_ints() {
//	gyro_write_byte(BMI088_GYRO_CTRL, BMI088_DRDY_ON);
//	vTaskDelay(10);
//	accel_write_byte(BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT);
//	vTaskDelay(10);
//	mag_write_single_reg(0x0B, 0x08); //enable drdy pin, pull to low on drdy
//	imu_init_status = 1;
//}
