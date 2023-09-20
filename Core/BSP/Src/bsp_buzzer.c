/*
 * bsp_buzzer.c
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "bsp_buzzer.h"

void buzzer(uint16_t freq)
{
	if (freq == 0)
	{
		htim12.Instance->CCR1 = 0;
	}
	else
	{
		__HAL_TIM_SET_PRESCALER(&htim12, (84 * 500/freq));
		htim12.Instance->CCR1 = 500;
	}
}


void buzzer_init()
{
	  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
	  htim12.Instance->CCR1 = 0;
}


