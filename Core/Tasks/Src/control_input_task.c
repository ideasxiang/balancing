/*
 * control_input_task.c
 *
 *  Created on: 4 Jul 2021
 *      Author: wx
 */
#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "control_input_task.h"
#include "motor_control.h"

#define JOYSTICK_OFFSET 1024
#define CONTROL_DEFAULT 0			//0 for remote, 1 for keyboard

extern TaskHandle_t buzzing_task_handle;
extern TaskHandle_t movement_control_task_handle;
extern TaskHandle_t control_input_task_handle;

extern motor_data_t can_motors[24];
extern referee_limit_t referee_limiters;
extern orientation_data_t imu_heading;

extern QueueHandle_t buzzing_task_msg;

remote_cmd_t remote_cmd = { 0 };

chassis_control_t chassis_ctrl_data;

pid_data_t aimbot_y_pid;
pid_data_t aimbot_x_pid;

xavier_packet_t aimbot_data;
uint8_t control_mode = CONTROL_DEFAULT;
uint8_t aimbot_mode;
int spinspin_mode = 0;
uint8_t safety_toggle = 1;

uint8_t aimbot_dma_buffer[2] = {0};
uint8_t aimbot_proc_buffer[OBC_DATA_SIZE];
buffer_t aimbot_buffer;
#define XAVIER_MSG_DELAY 30
#define AIMBOT_MSG_SIZE 8
#define AIMBOT_FRAME_HEADER 0x5A
#define AIMBOT_FRAME_END	0x69
uint8_t aimbot_start_frame;
/**
 * ISR function for receiving Xavier data
 *
 */
void aimbot_hlf_ISR(DMA_HandleTypeDef *hdma) {
	uint8_t time_gap = (HAL_GetTick() - aimbot_buffer.last_time > XAVIER_MSG_DELAY) ? 1 : 0;
	append_buffer(&aimbot_buffer, aimbot_dma_buffer[0]);
	if (time_gap) {
		aimbot_start_frame = 255;
		aimbot_buffer.stored_bytes = 0;
	}
	if (aimbot_dma_buffer[0] == AIMBOT_FRAME_HEADER) {
		aimbot_start_frame = aimbot_buffer.curr_byte-1;
		aimbot_buffer.stored_bytes = 1;
	}
	if (aimbot_buffer.stored_bytes >= AIMBOT_MSG_SIZE) {
		process_aimbot_data(aimbot_proc_buffer);
	}
}

void aimbot_ISR(DMA_HandleTypeDef *hdma) {
	uint8_t time_gap = (HAL_GetTick() - aimbot_buffer.last_time > XAVIER_MSG_DELAY) ? 1 : 0;
	append_buffer(&aimbot_buffer, aimbot_dma_buffer[1]);
	if (time_gap) {
		aimbot_start_frame = 255;
		aimbot_buffer.stored_bytes = 0;
	}
	if (aimbot_dma_buffer[1] == AIMBOT_FRAME_HEADER) {
			aimbot_start_frame = aimbot_buffer.curr_byte-1;
			aimbot_buffer.stored_bytes = 1;
		}
	if (aimbot_buffer.stored_bytes >= AIMBOT_MSG_SIZE) {
		process_aimbot_data(aimbot_proc_buffer);
	}
}
//


void process_aimbot_data()
{
	if (aimbot_buffer.stored_bytes >= AIMBOT_MSG_SIZE) {
		if (aimbot_buffer.curr_byte > aimbot_start_frame) {
			if(aimbot_buffer.curr_byte - aimbot_start_frame >= AIMBOT_MSG_SIZE) {
			memcpy(aimbot_proc_buffer, aimbot_buffer.buffer + aimbot_start_frame, AIMBOT_MSG_SIZE);
			} else {
				aimbot_buffer.stored_bytes = 0;
				return;
			}
		} else {
			uint8_t data_offset = BUFFER_SIZE - aimbot_start_frame;
			if (data_offset >= AIMBOT_MSG_SIZE) {
				aimbot_buffer.stored_bytes = 0;
				return;
			}
			memcpy(aimbot_proc_buffer, aimbot_buffer.buffer+aimbot_start_frame, data_offset);
			memcpy(aimbot_proc_buffer + data_offset, aimbot_buffer.buffer, AIMBOT_MSG_SIZE - data_offset);
		}
	} else {
		aimbot_buffer.stored_bytes = 0;
		return;
	}
	aimbot_data.frame_header = aimbot_proc_buffer[0];
	aimbot_data.x_pos = ((aimbot_proc_buffer[2] << 8) | aimbot_proc_buffer[3]);
	aimbot_data.y_pos = (aimbot_proc_buffer[4] << 8) | aimbot_proc_buffer[5];
	aimbot_data.end_check = aimbot_proc_buffer[7];
	aimbot_buffer.stored_bytes = 0;
	if (aimbot_data.frame_header != AIMBOT_FRAME_HEADER || aimbot_data.end_check != AIMBOT_FRAME_END) {
		aimbot_data.y_pos = 0;
		aimbot_data.x_pos = 0;
	} else {
		aimbot_data.y_pos -= 500;
		aimbot_data.x_pos -= 500;
		aimbot_data.last_time = HAL_GetTick();
	}
	status_led(1, on_led);
}


void dbus_remote_ISR(DMA_HandleTypeDef *hdma) {
	remote_cmd.right_x = (remote_raw_data[0] | remote_raw_data[1] << 8) & 0x07FF;
	remote_cmd.right_x -= JOYSTICK_OFFSET;
	remote_cmd.right_y = (remote_raw_data[1] >> 3 | remote_raw_data[2] << 5) & 0x07FF;
	remote_cmd.right_y -= JOYSTICK_OFFSET;
	remote_cmd.left_x = (remote_raw_data[2] >> 6 | remote_raw_data[3] << 2
			| remote_raw_data[4] << 10) & 0x07FF;
	remote_cmd.left_x -= JOYSTICK_OFFSET;
	remote_cmd.left_y = (remote_raw_data[4] >> 1 | remote_raw_data[5] << 7) & 0x07FF;
	remote_cmd.left_y -= JOYSTICK_OFFSET;
	//Left switch position
	remote_cmd.left_switch = ((remote_raw_data[5] >> 4) & 0x000C) >> 2;
	remote_cmd.right_switch = (remote_raw_data[5] >> 4) & 0x0003;
	remote_cmd.mouse_x = ((int16_t) remote_raw_data[6] | ((int16_t) remote_raw_data[7] << 8));
	remote_cmd.mouse_y = ((int16_t) remote_raw_data[8] | ((int16_t) remote_raw_data[9] << 8));
	remote_cmd.mouse_z = ((int16_t) remote_raw_data[10] | ((int16_t) remote_raw_data[11] << 8));
	remote_cmd.mouse_left = (remote_raw_data[12]);
	remote_cmd.mouse_right = (remote_raw_data[13]);
	remote_cmd.keyboard_keys = (remote_raw_data[14]);
	remote_cmd.side_dial = ((int16_t) remote_raw_data[16]) | ((int16_t) remote_raw_data[17] << 8);
	remote_cmd.side_dial -= JOYSTICK_OFFSET;
	remote_cmd.last_time = HAL_GetTick();
	if ((remote_cmd.keyboard_keys & KEY_OFFSET_Q) && (remote_cmd.keyboard_keys & KEY_OFFSET_SHIFT)
			&& (remote_cmd.keyboard_keys & KEY_OFFSET_CTRL)) {
		control_mode = 1 - control_mode;
	}

	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(control_input_task_handle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken)
}

void control_input_task(void *argument) {
	TickType_t start_time;
	control_reset();
	buffer_init(&aimbot_buffer);
	xavier_usart_start(aimbot_dma_buffer, 2);

	dbus_remote_start(remote_raw_data);
	while (1) {
		uint8_t rc_check = ulTaskNotifyTake(pdTRUE, 500);
		if (rc_check) {
			status_led(1, on_led);
			start_time = xTaskGetTickCount();
			if (remote_cmd.right_switch == all_off) {
				kill_can();
				control_reset();
				safety_toggle = 0;
				control_mode_change(remote_cmd.side_dial);
			} else {
				laser_on();
				//			referee_compensation();
				chassis_control_input();
			}
			status_led(1, off_led);
			if (remote_cmd.left_switch == aimbot_enable) {
			} else {

			}
		} else {
			kill_can();
			control_reset();

		}
		vTaskDelayUntil(&start_time, CONTROL_DELAY);
	}
	osThreadTerminate(NULL);
}

void control_reset() {
	chassis_ctrl_data.forward = 0;
	chassis_ctrl_data.horizontal = 0;
	chassis_ctrl_data.yaw = 0;
	chassis_ctrl_data.enabled = 0;
	laser_off();
	aimbot_data.x_offset = 0;
	aimbot_data.y_offset = 0;
	spinspin_mode = 0;
}

void control_mode_change(int16_t left_dial_input) {
	static uint8_t mode_change_status;
	static uint32_t last_trig_time;
	uint8_t temp_msg;
	if (control_mode != 1) {
		if (left_dial_input > 330) {
			if (mode_change_status == 2) {
				if (HAL_GetTick() - last_trig_time > 1000) {
					control_mode = 1;
					temp_msg = control_keyboard;
					xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);
				}
			}
		} else {
			mode_change_status = 2;
			last_trig_time = HAL_GetTick();
		}
	} else if (control_mode != 0) {
		if (left_dial_input < -330) {
			if (mode_change_status == 1) {
				if (HAL_GetTick() - last_trig_time > 1000) {
					control_mode = 0;
					temp_msg = control_control;
					xQueueSendToBack(buzzing_task_msg, &temp_msg, 0);
				}
			} else {
				mode_change_status = 1;
				last_trig_time = HAL_GetTick();
			}
		}
	} else {
		mode_change_status = 0;
	}
}


void chassis_control_input() {
	float forward_input =0;
	float yaw_input = 0;
	if (safety_toggle || remote_cmd.right_switch != all_on) {
		chassis_ctrl_data.enabled = 0;
	} else {
		chassis_ctrl_data.enabled = 1;
		if (remote_cmd.right_switch == all_on) {
			if (control_mode == 1) {
			} else {
				forward_input = (float) remote_cmd.left_y / RC_LIMITS;
				yaw_input = (float) remote_cmd.left_x / RC_LIMITS;
			}

			//rotation matrix of the initial vectorrrr wow MA1513 is relevant?!
			chassis_ctrl_data.forward =forward_input;
			chassis_ctrl_data.yaw = yaw_input;
		}
	}
}

void keyboard_mvt_ctrl(float *forward_ctrl, float *horizontal_ctrl) {
	if (remote_cmd.keyboard_keys & KEY_OFFSET_W) {
		*forward_ctrl += KEYBD_MAX_SPD;
	}
	if (remote_cmd.keyboard_keys & KEY_OFFSET_S) {
		*forward_ctrl -= KEYBD_MAX_SPD;
	}

	if (remote_cmd.keyboard_keys & KEY_OFFSET_A) {
		*horizontal_ctrl -= KEYBD_MAX_SPD;
	}
	if (remote_cmd.keyboard_keys & KEY_OFFSET_D) {
		*horizontal_ctrl += KEYBD_MAX_SPD;
	}
}


void dbus_reset() {
	remote_cmd.right_switch = all_off;
	remote_cmd.right_x = 0;
	remote_cmd.right_y = 0;
	remote_cmd.left_x = 0;
	remote_cmd.left_y = 0;
	remote_cmd.left_switch = 0;
	remote_cmd.mouse_x = 0;
	remote_cmd.mouse_y = 0;
	remote_cmd.mouse_z = 0;
	remote_cmd.mouse_left = 0;
	remote_cmd.mouse_right = 0;
}

