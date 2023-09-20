/*
 * calibration_task.c
 *
 *  Created on: 10 May 2021
 *      Author: wx
 */

/*
 * HELLHOLE TIME
 * calibration tasks
 * 1) initialise IMU
 * 2) check motor statuses (check until all values are set)?
 * 3) gimbal calibration**** (find the offset)
 * 4) set all the KP KI and KD values
 * 5) set all the max current values
 * 6) IMU calibration**** (find gimbal angle to acceleration readings)
 * 7) find appropriate chassis motor rpm vs yaw gimbal rpm
 * ratio for rotating hell
 * 8) SAVE ALL THE VALUES
 * 9) check if saved before, and escape if yes (well this goes on top
 * of course)
 * specific key combination for activating this calibration
 * THIS NEEDS TO BEEP AT YOU UNTIL IT'S SAFE TO CARRY ON
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_control.h"
#include "startup_task.h"
#include "buzzing_task.h"
#include <can_msg_processor.h>

extern uint8_t imu_triggered;
extern remote_cmd_t remote_cmd;
extern float pitch;
extern float yaw;
uint16_t error = 0b111111111;

void startup_task() {
	led_on();
	buzzer_init();
	imu_config();
	HAL_Delay(STARTUP_DELAY);
	led_green_off();
	start_micros_timer();
	//motor online check, code does not continue if any fails
	ok_buzzer(1, 1);
	led_toggle();

}

void err_buzzer(uint8_t low, uint8_t high) {
	for (int8_t i = 0; i < low; i++) {
		//PWM
		buzzer(LOW_FREQ);
		HAL_Delay(BUZZER_DELAY);
		buzzer(0);
		HAL_Delay(BUZZER_DELAY);
	}
	for (int8_t i = 0; i < high; i++) {
		buzzer(HIGH_FREQ);
		HAL_Delay(BUZZER_DELAY);
		buzzer(0);
		HAL_Delay(BUZZER_DELAY);
	}
	HAL_Delay(BUZZER_DELAY * 2);
}

void ok_buzzer(uint8_t high, uint8_t low) {
	for (int8_t i = 0; i < high; i++) {
		//PWM
		buzzer(HIGH_FREQ);
		HAL_Delay(BUZZER_DELAY);
		buzzer(0);
		HAL_Delay(BUZZER_DELAY);
	}
	for (int8_t i = 0; i < low; i++) {
		buzzer(LOW_FREQ);
		HAL_Delay(BUZZER_DELAY);
		buzzer(0);
		HAL_Delay(BUZZER_DELAY);
	}
	HAL_Delay(BUZZER_DELAY * 2);
}

