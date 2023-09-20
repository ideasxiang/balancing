/*
 * control_input_task.h
 *
 *  Created on: 4 Jul 2021
 *      Author: wx
 */

#ifndef TASKS_INC_CONTROL_INPUT_TASK_H_
#define TASKS_INC_CONTROL_INPUT_TASK_H_

void aimbot_ISR(DMA_HandleTypeDef *hdma);
void aimbot_hlf_ISR(DMA_HandleTypeDef *hdma);
void dbus_reset();
void control_input_task(void *argument);
void gimbal_control_input();
void chassis_control_input();
void launcher_control_input();
void control_reset();
void keyboard_mvt_ctrl(float *forward_ctrl, float *horizontal_ctrl);
void aimbot_pid_init();
void start_buzz();
void control_mode_change(int16_t left_dial_input);
void process_aimbot_data();

#endif /* TASKS_INC_CONTROL_INPUT_TASK_H_ */
