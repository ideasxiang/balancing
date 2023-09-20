/**
 * bsp_usart.h
 *
 * Created on: Mar 2 2020
 *     Author: Raghav Bhardwaj
 */

/** Instructions
 * 1) Configure USARTx and the corresponding DMA in CubeMX. Refer to instructions on GitHub.
 * 2) Include this header file.
 * 3) Start circular DMA process for detecting received data using usart_start(). Note
 *    that this function is blocking so this should be started in a freeRTOS task. Call
 *    this after all peripherals have been initialized.
 * 4) Define usart_ISR() somewhere in the code to respond to received data on USARTx.
 * 5) Whenever the DMA received data, the ISR will be triggered. To get the buffer of
 *    raw received data, use the usart_get_data() function.
 * 6) To sent data on the USARTx port, use the usart_send_data() function.
 */

#ifndef INC_BSP_USART_H_
#define INC_BSP_USART_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"

#define XAVIER_DATA_SIZE 8


#define START_MAGIC_NUMBER	69
#define END_MAGIC_NUMBER	420

void xavier_ISR(DMA_HandleTypeDef *UartHandle);
void usart_ISR(UART_HandleTypeDef *UartHandle);

/**
 * Transmit a fixed number of bytes on the USART port.
 *
 * @param huart        The pointer to the USART port handle that is to transmit data.
 * @param tx_buffer    The buffer of size buffer_size which contains the data to send.
 * @param buffer_size  The size of tx_buffer in bytes.
 * @param timeout      How long the execution should block while waiting for the data
 * 					   to send. HAL_MAX_DELAY will wait indefinitely. 0 will not wait
 * 					   at all and will continue with execution.
 */
HAL_StatusTypeDef usart_send_data(UART_HandleTypeDef *huart, uint8_t *tx_buffer,
		uint16_t buffer_size, uint32_t timeout);

/**
 * Reads the latest value received at the USART port. In DMA interrupt mode,
 * call this function upon triggering the usart_ISR() function for a particular
 * USART port.
 */
uint8_t* usart_get_data(UART_HandleTypeDef *UartHandle);

/**
 * This function starts the circular DMA that reads from the USARTx port to memory.
 * This function is blocking, so it is advised to only call this inside a freeRTOS
 * task so that execution flow can continue thanks to preemption.
 *
 * @param huart        Pointer to the USART port handle that is to start.
 * @param rx_buffer    Pointer to the buffer of size buffer_size, where the bytes to
 *                     be read from the USART port will be stored.
 * @param buffer_size  Size in bytes of the buffer to store data received at the USART
 *                     port.
 */
void usart_start(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t buffer_size);

HAL_StatusTypeDef xavier_usart_start(uint8_t *pData, uint16_t Size);

#endif
