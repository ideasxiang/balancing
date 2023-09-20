/*
 * movement_control_task.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "arm_math.h"
#include "movement_control_task.h"

extern EventGroupHandle_t chassis_event_group;

extern chassis_control_t chassis_ctrl_data;
extern motor_data_t can_motors[24];
extern referee_limit_t referee_limiters;
extern orientation_data_t imu_heading;

extern remote_cmd_t remote_cmd;   // for reading switch value

pid_data_t b_angle_pid;
pid_data_t b_orient_pid;
static pid_data_t b_accel_pid;
static float target_speed;
static float avg_wheel_speed;
static float angular_vel;
static float velocity;
static float accel;
static float last_angles[2];
static float last_vel[2];
static uint32_t last_angle_time[2];
static int counter;
static float currentangle;
static float anglebreakpt = 7;
static int8_t first_run = 1 , first_rebalance = 1;
static long zero_orient[3];
long current_orient;
extern long target_orient;
static int32_t last_ticks[3];
//static int debugswitch;
static float k1 = 600;
static float k2 = 2500;
static float k3 = 10000;
static float k4 = 9000;
//static float k1 = 6673;   // 6673 1727 100 601
//static float k2 = 1727;
//static float k3 = 100;  //14321.560
//static float k4 = 317;
//static float LQRmat[4][4] = {{0,    1.0000,  0,    0}, {-12.2348,  -14.5780,   88.1321,   25.5993}, {0,         0,         0,    1.0000},{ -18.7578,  -22.3503,  150.1450 ,  39.2476}};
//static float Cmat[4] = {1, 0 , 0, 0};


void movement_control_task(void *argument) {
	TickType_t start_time;
	b_angle_pid.kp 			=	B_ANGLE_KP;
	b_angle_pid.ki 			=	B_ANGLE_KI;
	b_angle_pid.kd 			= 	B_ANGLE_KD;
	b_angle_pid.int_max		= 	B_ANGLE_INT_MAX;
	b_angle_pid.max_out		= 	B_ANGLE_MAX_OUT;
	b_angle_pid.min_out		=	-B_ANGLE_MAX_OUT;
//	b_angle_pid.physical_max=	1000;

	b_accel_pid.kp 			= 	B_ACCEL_KP;
	b_accel_pid.ki 			= 	B_ACCEL_KI;
	b_accel_pid.kd 			=	B_ACCEL_KD;
	b_accel_pid.int_max		=	B_ACCEL_INT_MAX;
	b_accel_pid.max_out		= 	B_ACCEL_MAX_OUT;
	b_accel_pid.min_out		= 	-B_ACCEL_MAX_OUT;

	b_orient_pid.kp 			= 	B_ORIENT_KP;
	b_orient_pid.ki 			= 	B_ORIENT_KI;
	b_orient_pid.kd 			=	B_ORIENT_KD;
	b_orient_pid.int_max		=	B_ORIENT_INT_MAX;
	b_orient_pid.max_out		= 	B_ORIENT_MAX_OUT;
	b_orient_pid.min_out		= 	-B_ORIENT_MAX_OUT;

	while (1) {
		start_time = xTaskGetTickCount();

		if (chassis_ctrl_data.enabled) {   // RE_ENABLE!!!!
			chassis_motion_control(can_motors + FR_MOTOR_ID - 1, can_motors + FL_MOTOR_ID - 1);
		} else { //kill motors
			can_motors[FR_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[FL_MOTOR_ID - 1].rpm_pid.output = 0;
		}

		//delays task for other tasks to run
		status_led(3, off_led);
		vTaskDelayUntil(&start_time, BALANCE_DELAY);
	}
	osThreadTerminate(NULL);
}
void chassis_MCU_send_CAN() {

	uint8_t CAN_send_data[8] = { 0, };
	memcpy(CAN_send_data, &chassis_ctrl_data, sizeof(CAN_send_data));
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint32_t send_mail_box;
	CAN_tx_message.IDE = CAN_ID_STD;
	CAN_tx_message.RTR = CAN_RTR_DATA;
	CAN_tx_message.DLC = 0x08;

	if (chassis_ctrl_data.enabled) {
		CAN_tx_message.StdId = 0x111;
		HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data, &send_mail_box);
	} else {
		CAN_tx_message.StdId = 0x100;
		HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data, &send_mail_box);
	}
}

void chassis_motion_control(motor_data_t *motorfr,
		motor_data_t *motorfl) {

	currentangle = imu_heading.pit* 57.2958 - ANGLE_OFFSET;

	if (first_run==1){  //orientation intialise
		zero_orient[1] =motorfl->angle_data.ticks;
		zero_orient[2] = motorfr->angle_data.ticks;
		zero_orient[0] = zero_orient[1]-zero_orient[2];
		target_orient = current_orient;
		first_run = 0;

	}
	last_ticks[1] = -(motorfl->angle_data.ticks-zero_orient[1]);   //orientation calc
	last_ticks[2] = motorfr->angle_data.ticks-zero_orient[2];
	current_orient = last_ticks[1] - last_ticks[2];

	if (remote_cmd.left_switch == 2){   // if tipped over, manually toggle to standback up
//		if (first_rebalance == 1){
//		b_angle_pid.integral*=0;
//		first_rebalance = 0;
//		}
		b_angle_pid.kp 			=	B_ANGLE_KP*0.5;
		b_angle_pid.kd 			=	B_ANGLE_KD *2;
		speed_pid(0, imu_heading.pit* 57.2958 - ANGLE_OFFSET, &b_angle_pid);

	}
	else{


		//debugswitch = remote_cmd.left_switch;
		target_speed = chassis_ctrl_data.forward * MAX_SPEED;
		avg_wheel_speed = (motorfr->raw_data.rpm - motorfl->raw_data.rpm)/38.4   //motors running in opposite directions, so (L+(-R))/2 / gear ratio 19.2
											* WHEEL_CIRC/100 *1/60;		//rpm * wheel circ in cm /100 * 1/60  = m/s

		last_angle_time[0] = HAL_GetTick(); //ms
		last_angles[0] = imu_heading.pit;
		float dt = ((float) (last_angle_time[0] - last_angle_time[1]) / 1000.0f);
		angular_vel = (last_angles[0] - last_angles[1]) / dt;
		velocity = avg_wheel_speed - CG_radius * angular_vel * cos(last_angles[0]);   // v = wheel vel - r* w* cos( theta )
		last_vel[0] = velocity;
		accel = ((float) (last_vel[0] - last_vel[1]) / 1000.0f);

		last_angles[1] = last_angles[0];
		last_vel[1] = last_vel[0];
		last_angle_time[1] = last_angle_time[0];

		if (remote_cmd.left_switch == 1){  //left switch up for Cascade PID control
//			target_speed += 0.4;   // ADD 0.2 speed
			b_angle_pid.kp 			=	B_ANGLE_KP;  //reset
			b_angle_pid.ki 			=	B_ANGLE_KI;  //reset
			b_angle_pid.kd 			=	B_ANGLE_KD;
			b_accel_pid.max_out		= 	B_ACCEL_MAX_OUT;
			b_accel_pid.kp 			= 	B_ACCEL_KP;
//			if (fabs(currentangle)> anglebreakpt ){ //increase for more stability at higher speed
//				//b_angle_pid.kp 			=	B_ANGLE_KP*2;
//				b_angle_pid.kd 			=	B_ANGLE_KD*0.3;
//				b_accel_pid.max_out		= 	B_ACCEL_MAX_OUT*2;
//				//b_accel_pid.kp 			= 	B_ACCEL_KP*3;
//				}
			if (counter == 10){
				if (fabs(chassis_ctrl_data.forward) <0.05){  //decay integral faster to stop if minimal fwd input
						b_accel_pid.integral*=0.8;
					}
				speed_pid(target_speed, velocity, &b_accel_pid); //velocity PID calc
				counter = 0;
			}
			else {
				counter+=1;
			}

			speed_pid(b_accel_pid.output, imu_heading.pit* 57.2958 - ANGLE_OFFSET, &b_angle_pid); //angle PID calc
		}

		else if (remote_cmd.left_switch == 3){   //left switch middle  for State Space control
//			ssfb
			b_angle_pid.output = - (currentangle * k1 + angular_vel * k2 + (velocity-target_speed*0.8 )* k3 + accel * k4);

///////////LQR
//			b_angle_pid.output = - (currentangle * k1 + angular_vel * k2 + (velocity)* k3 + accel * k4);
//			float x[4] = { currentangle, angular_vel , velocity, accel };
//			float xdot[4] = {0,0,0,0};
//			for ( int i = 0; i <4 ; ++i){
//				float tempsum = 0;
//				for ( int j = 0; j <4 ; ++j){
//					tempsum+=LQRmat[i][j] * x[j];
//				xdot[i] = tempsum;
//				}
//			}
//
//			float tempsum2 = 0;
//			for ( int i = 0; i <4 ; ++i){
//				tempsum2+=xdot[i] * Cmat[i];
//				}
//			b_angle_pid.output = -tempsum2*50000;

//  to try
//  -1000         -1592.04532834368          14321.5603625769          4859.61253037228
//  -1000         -1951.57855823567          16119.1737592225          5221.19616328436     100 100 0.001 1000
//  -1000        -2402.08107336884            25685.56412169          11994.1198290544 		100 100 0.001 1000

			}



	}

}


