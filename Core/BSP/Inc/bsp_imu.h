/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_imu.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __MPU_H__
#define __MPU_H__


#define MPU_DELAY(x) HAL_Delay(x)



//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void init_quaternion(void);
void mpu_get_data(void);
void mpu_offset_call(void);
void get_imu_data(uint16_t trig_pin);
void process_ist_data();
void process_mpu_data();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

float inv_sqrt(float x);
void reset_imu_data();
void gyro_offset_cali();
uint8_t imu_config(void);
void accel_get_data();
void accel_process_data();
void gyro_get_data();
void gyro_process_data();
uint8_t ist8310_init();
void ist8310_get_data();
void imu_start_ints();
HAL_StatusTypeDef gyro_txrx_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);

HAL_StatusTypeDef accel_txrx_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);


#endif


