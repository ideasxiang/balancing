/*
 * motor_feedback_task.c
 *
 *  Created on: Sep 16, 2022
 *      Author: wx
 */


#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "arm_math.h"
#include "motor_feedback_task.h"
#include "can_msg_processor.h"

extern chassis_control_t chassis_ctrl_data;
extern EventGroupHandle_t chassis_event_group;

extern motor_data_t can_motors[24];
extern pid_data_t b_angle_pid;
extern pid_data_t b_orient_pid;
static volatile float fl_speed = 0;
static volatile float fr_speed = 0;
static volatile float rotation;
extern long current_orient;
long target_orient;
static uint8_t lastturn = 0;


void motor_feedback_task(void *argument) {
	TickType_t start_time;
	while (1) {
		start_time = xTaskGetTickCount();
		xEventGroupWaitBits(chassis_event_group, 0b1100, pdTRUE, pdTRUE, portMAX_DELAY); //wait for all motors to connect

		if (chassis_ctrl_data.enabled) {   // RE ENABLE!!!
			chassis_feedback_loop(can_motors + FR_MOTOR_ID - 1, can_motors + FL_MOTOR_ID - 1);
		} else {
			can_motors[FR_MOTOR_ID - 1].rpm_pid.output = 0;  //can_motors[13-1]
			can_motors[FL_MOTOR_ID - 1].rpm_pid.output = 0;  //can_motors[14-1]
			fl_speed = 0;
			fr_speed = 0;
			motor_send_can(can_motors, FR_MOTOR_ID, FL_MOTOR_ID, 0, 0);
		}

		//delays task for other tasks to run
		status_led(3, off_led);

		xEventGroupClearBits(chassis_event_group, 0b1111);
		vTaskDelayUntil(&start_time, CHASSIS_DELAY);
	}
	osThreadTerminate(NULL);
}

void chassis_feedback_loop(motor_data_t *motorfr,
		motor_data_t *motorfl){
	motor_calc_odometry(&can_motors[13].raw_data, &can_motors[13].angle_data,  //note: is disabled in can_msg_processor.c
								can_motors[13].last_time);
	motor_calc_odometry(&can_motors[12].raw_data, &can_motors[12].angle_data,
								can_motors[12].last_time);
	if (fabs(chassis_ctrl_data.yaw) >= 0.05) {
		lastturn = 1 ;
		target_orient = current_orient - chassis_ctrl_data.yaw*30000; // 30000
		}

	if (fabs(chassis_ctrl_data.yaw) < 0.05 && lastturn == 1) {
		target_orient = current_orient;
		lastturn = 0 ;
		}
	speed_pid(target_orient, current_orient,&b_orient_pid);


//	rotation = chassis_ctrl_data.yaw * 1000;
	rotation = b_orient_pid.output;
	fl_speed = +b_angle_pid.output - rotation;  // motors run in opp directions
	fr_speed = -b_angle_pid.output - rotation;


	speed_pid(fl_speed, motorfl->raw_data.rpm,&motorfl->rpm_pid);
	speed_pid(fr_speed, motorfr->raw_data.rpm,&motorfr->rpm_pid);

	motor_send_can(can_motors, FR_MOTOR_ID, FL_MOTOR_ID, 0, 0); // 0 0  since only 2 motors
}


