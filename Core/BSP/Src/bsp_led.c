/*
 * bsp_led.c
 *
 *  Created on: Jan 19, 2020
 *      Author: Kai Yang
 */

#include "bsp_led.h"

void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  /* points to address of GPIOx register and does bitwise XOR with bit GPIO_Pin */
  GPIOx->ODR ^= GPIO_Pin;
}

/* calls HAL library function to directly write pin.             */
/* LEDs are active low, so SET (1) == off, while RESET (0) == on */
/* Definitions for LED port and pins are given in main.h file    */
void led_green_off(void){
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}

void led_green_on(void){
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

}

void led_green_toggle(void){
    GPIO_ToggleBits(GPIOF, GPIO_PIN_14);
}

void led_red_off(void){
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
}

void led_red_on(void){
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
}

void led_red_toggle(void){
    GPIO_ToggleBits(GPIOE, GPIO_PIN_11);
}

void led_off(void){
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}

void led_on(void){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
}
void led_toggle(void){
	GPIO_ToggleBits(GPIOE, GPIO_PIN_11);
	GPIO_ToggleBits(GPIOF, GPIO_PIN_14);
}

void status_led(uint16_t led_no, uint8_t led_state)
{
	assert_param(IS_GPIO_ALL_PERIPH(GPIOG));

	  /* points to address of GPIOx register and does bitwise XOR with bit GPIO_Pin */

	if (led_no > 0 && led_no <= 8)
	{
		if (led_state == toggle_led)
		{
			GPIOG->ODR ^= 1 << (1 << led_no);
		}
		else if (led_state == on_led)
		{
			GPIOG->ODR &= ~(1 << led_no);
		}
		else if (led_state == off_led)
		{
			GPIOG->ODR |= (1 << led_no);
		}
	}
	if (led_no == 9)
	{
		if (led_state == toggle_led)
		{
			GPIOG->ODR ^= 0xFF;
		}
		else if (led_state == on_led)
		{
			GPIOG->ODR &= ~(0xFF);
		}
		else if (led_state == off_led)
		{
			GPIOG->ODR |= (0xFF);
		}
	}

}
