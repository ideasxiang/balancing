/*
 * actuator_feedback_task.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "can_msg_processor.h"

extern EventGroupHandle_t gimbal_event_group;
extern EventGroupHandle_t chassis_event_group;
extern EventGroupHandle_t launcher_event_group;
#define ANGLE_LPF 0
#define SPEED_LPF 0
#ifndef CHASSIS_MCU
motor_data_t can_motors[24];
#else
motor_data_t can_motors[12];
#endif

/**
 * CAN ISR function, triggered upon RX_FIFO0_MSG_PENDING
 * converts the raw can data to the motor_data struct form as well
 */
void can_ISR(CAN_HandleTypeDef *hcan) {

	CAN_RxHeaderTypeDef rx_msg_header;
	uint8_t rx_buffer[CAN_BUFFER_SIZE];
	if (hcan->Instance == CAN1) {
		HAL_CAN_DeactivateNotification(hcan,
				CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN);
		can1_get_msg(&hcan1, &rx_msg_header, rx_buffer);
		convert_raw_can_data(can_motors, rx_msg_header.StdId, rx_buffer);
		HAL_CAN_ActivateNotification(hcan,
				CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN);
	}
#ifndef CHASSIS_MCU
	else if (hcan->Instance == CAN2) {
		HAL_CAN_DeactivateNotification(hcan,
				CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN);
		can2_get_msg(&hcan2, &rx_msg_header, rx_buffer);
		convert_raw_can_data(can_motors, rx_msg_header.StdId + 12, rx_buffer);
		HAL_CAN_ActivateNotification(hcan,
				CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN);
	}
#else
	else if (hcan->Instance == CAN2)
	{
		can_get_msg(&hcan2, &rx_msg_header, rx_buffer);
		process_chassis_can_msg(rx_msg_header.StdId, rx_buffer);
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL	| CAN_IT_RX_FIFO0_OVERRUN);
	}
#endif
}

/*
 * Converts raw CAN data over to the motor_data_t struct
 * 7 bytes of CAN data is sent from the motors:
 * High byte for motor angle data
 * Low byte for motor angle data
 * High byte for RPM
 * Low byte for RPM
 * High byte for Torque
 * Low byte for Torque
 * 1 byte for temperature
 *
 * This function combines the respective high and low bytes into 1 single 16bit integer, then stores them
 * in the struct for the motor.
 *
 * For GM6020 motors, it recenters the motor angle data and converts it to radians.
 */

void convert_raw_can_data(motor_data_t *can_motor_data, uint16_t motor_id, uint8_t *rx_buffer) {
	BaseType_t xHigherPriorityTaskWoken, xResult;
	xHigherPriorityTaskWoken = pdFALSE;
	uint16_t idnum = motor_id - 0x201;
	if (idnum > 24)
	{
		return;
	}
	if (can_motor_data[idnum].motor_type > 0) {
		can_motor_data[idnum].id = motor_id;
		can_motor_data[idnum].raw_data.angle[0] = (rx_buffer[0] << 8) | rx_buffer[1];
		int16_t temp_rpm					=(rx_buffer[2] << 8) | rx_buffer[3];
		can_motor_data[idnum].raw_data.rpm = can_motor_data[idnum].raw_data.rpm * SPEED_LPF + temp_rpm * (1-SPEED_LPF);
		can_motor_data[idnum].raw_data.torque = (rx_buffer[4] << 8) | rx_buffer[5];
		can_motor_data[idnum].raw_data.temp = (rx_buffer[6]);
		can_motor_data[idnum].last_time[1] = can_motor_data[idnum].last_time[0];
		can_motor_data[idnum].last_time[0] = get_microseconds();
		switch (can_motor_data[idnum].motor_type) {
		case TYPE_GM6020:
			angle_offset(&can_motor_data[idnum].raw_data, &can_motor_data[idnum].angle_data);
			break;
		case TYPE_M2006:
		case TYPE_M3508:
			break;
		case TYPE_M2006_STEPS:
		case TYPE_M3508_STEPS:
//			motor_calc_odometry(&can_motor_data[idnum].raw_data, &can_motor_data[idnum].angle_data,
//					can_motor_data[idnum].last_time);
			break;
		case TYPE_M2006_ANGLE:
		case TYPE_M3508_ANGLE:
		case TYPE_GM6020_720:
			motor_calc_odometry(&can_motor_data[idnum].raw_data, &can_motor_data[idnum].angle_data,
					can_motor_data[idnum].last_time);
			angle_offset(&can_motor_data[idnum].raw_data, &can_motor_data[idnum].angle_data);
			break;
		default:
			break;

		}

		switch (idnum + 1) {
				#ifndef CHASSIS_MCU
						case FR_MOTOR_ID:
							xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b1000,
									&xHigherPriorityTaskWoken);
							break;
						case FL_MOTOR_ID:
							xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b0100,
									&xHigherPriorityTaskWoken);
							break;
						case BL_MOTOR_ID:
							xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b0010,
									&xHigherPriorityTaskWoken);
							break;
						case BR_MOTOR_ID:
							xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b0001,
									&xHigherPriorityTaskWoken);
							break;
				#endif
						case LFRICTION_MOTOR_ID:
							xResult = xEventGroupSetBitsFromISR(launcher_event_group, 0b010,
									&xHigherPriorityTaskWoken);
							break;
						case RFRICTION_MOTOR_ID:
							xResult = xEventGroupSetBitsFromISR(launcher_event_group, 0b001,
									&xHigherPriorityTaskWoken);
							break;
						case FEEDER_MOTOR_ID:
							xResult = xEventGroupSetBitsFromISR(launcher_event_group, 0b100,
									&xHigherPriorityTaskWoken);
							break;
						case PITCH_MOTOR_ID:
							xResult = xEventGroupSetBitsFromISR(gimbal_event_group, 0b01,
									&xHigherPriorityTaskWoken);
							break;
						case YAW_MOTOR_ID:
							xResult = xEventGroupSetBitsFromISR(gimbal_event_group, 0b10,
									&xHigherPriorityTaskWoken);
							break;
						default:
							//error handler
							break;
						}
						if (xResult != pdFAIL) {
							portYIELD_FROM_ISR(xHigherPriorityTaskWoken); //forces current task to yield if higher priority task is called
						}
					} else {
						//error handler
					}

}

void process_chassis_can_msg(uint16_t msg_id, uint8_t rx_buffer[]) {
	//for future use
}

/**
 * Centers the raw motor angle to between -Pi to +Pi
 */
void angle_offset(raw_data_t *motor_data, angle_data_t *angle_data) {
	int32_t temp_ang = 0;

	if (angle_data->gearbox_ratio > 0) {
		temp_ang = angle_data->ticks - angle_data->center_ang;
		if (temp_ang > (4096 * angle_data->gearbox_ratio)) {
			temp_ang -= (8192 * angle_data->gearbox_ratio);
		} else if (temp_ang < (-4096 * angle_data->gearbox_ratio)) {
			temp_ang += 8192 * angle_data->gearbox_ratio;
		}
		angle_data->adj_ang = (float) temp_ang * PI / (8192 * angle_data->gearbox_ratio);
	} else {
		temp_ang = (int32_t) (motor_data->angle[0]) - angle_data->center_ang;
		if (temp_ang > 4096) {
			temp_ang -= 8192;
		} else if (temp_ang < -4096) {
			temp_ang += 8192;
		}
		angle_data->adj_ang = (angle_data->adj_ang * ANGLE_LPF) + (float) (temp_ang * PI / 4096) * (1 - ANGLE_LPF); // convert to radians
	}
}

void motor_calc_odometry(raw_data_t *motor_data,
		angle_data_t *angle_data,
		uint32_t feedback_times[]) {
	int8_t int_round_passed = 0;
	if (feedback_times[0] - feedback_times[1] >= 1) {
		float rounds_passed = (((float)(feedback_times[0] - feedback_times[1]) * motor_data->rpm)/(60 * TIMER_FREQ));
		if (fabs(rounds_passed) >= 1) {
			int_round_passed = rounds_passed;
		} else {
			int_round_passed = 0;
		}
	}
	int16_t abs_angle_diff;
	abs_angle_diff = motor_data->angle[0] - motor_data->angle[1];
	if (abs_angle_diff > 4096) {
		abs_angle_diff -= 8192;
	} else if (abs_angle_diff < -4096) {
		abs_angle_diff += 8192;
	}
	//add for the case to just get the magnitude and check for any overflow
	/*
	 else if (angle_data->dir == 1) {
	 if (motor_data->angle[0] > motor_data->angle[1]) {
	 abs_angle_diff = motor_data->angle[0] - motor_data->angle[1];
	 } else {
	 abs_angle_diff = motor_data->angle[1] + (8192-motor_data->angle[1]);
	 }
	 } else {
	 if (motor_data->angle[0] < motor_data->angle[1]) {
	 abs_angle_diff = motor_data->angle[1] - motor_data->angle[0];
	 } else {
	 abs_angle_diff = (8192-motor_data->angle[0]) - motor_data->angle[1];
	 }
	 }
	 */
	uint16_t gear_ticks = 8192 * angle_data->gearbox_ratio;
	angle_data->ticks += (int_round_passed * 8192) + abs_angle_diff;
	angle_data->dist = angle_data->ticks * angle_data->wheel_circ / gear_ticks;
	angle_data->adj_ang = (float) (angle_data->ticks % gear_ticks ) * 2*PI/gear_ticks;
	angle_data->adj_ang = (angle_data->adj_ang > PI) ? (angle_data->adj_ang - 2*PI) : (angle_data->adj_ang < -PI) ? angle_data->adj_ang+2*PI : angle_data->adj_ang;
	motor_data->angle[1] = motor_data->angle[0];
}
