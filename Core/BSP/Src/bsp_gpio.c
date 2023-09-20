/*
 * bsp_gpio.c
 *
 *  Created on: Sep 7, 2021
 *      Author: wx
 */
#include "board_lib.h"
#include "bsp_gpio.h"


void laser_on()
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
}

void laser_off()
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
}
