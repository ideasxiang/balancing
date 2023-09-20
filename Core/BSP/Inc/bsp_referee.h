/*
 * bsp_referee.h
 *
 *  Created on: Jun 28, 2021
 *      Author: wx
 */

#ifndef BSP_INC_BSP_REFEREE_H_
#define BSP_INC_BSP_REFEREE_H_


HAL_StatusTypeDef referee_usart_start(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

void referee_ISR();

#endif /* BSP_INC_BSP_REFEREE_H_ */
