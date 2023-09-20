/*
 * bsp_dbus_input.c
 *
 * Created on: Mar 2 2020
 *     Author: Raghav Bhardwaj
 */
#include "board_settings.h"
#include "bsp_dbus_input.h"

uint8_t remote_raw_data[REMOTE_DATA_SIZE] = {0};

/**
 * This function starts the circular DMA for receiving on a UART port. It is specifically
 * written for the UART1 port for DBUS interface from the controller.
 */
HAL_StatusTypeDef dbus_remote_start(uint8_t *pData)
{
	UART_HandleTypeDef *huart = &DBUS_UART;
	uint32_t *tmp;

	/* Check that a Rx process is not already ongoing */
	if (huart->RxState == HAL_UART_STATE_READY) {
		if ((pData == NULL) || (REMOTE_DATA_SIZE == 0U)) {
			return HAL_ERROR;
		}

		/* Process Locked */
		__HAL_LOCK(huart);

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = REMOTE_DATA_SIZE;

		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_BUSY_RX;

		/* Set the UART DMA transfer complete callback */
		huart->hdmarx->XferCpltCallback = dbus_remote_ISR;

		/* Set the DMA abort callback */
		huart->hdmarx->XferAbortCallback = NULL;

		/* Enable the DMA stream */
		tmp = (uint32_t *)&pData;
		HAL_DMA_Start_IT(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t *)tmp, REMOTE_DATA_SIZE);

		/* Clear the Overrun flag just before enabling the DMA Rx request: can be mandatory for the second transfer */
		__HAL_UART_CLEAR_OREFLAG(huart);

		/* Process Unlocked */
		__HAL_UNLOCK(huart);

		/* Enable the UART Parity Error Interrupt */
		SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);

		/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
		SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
	    in the UART CR3 register */
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	} else {
		return HAL_BUSY;
	}
}
