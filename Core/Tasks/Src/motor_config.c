/*
 * motor_config.c
 *
 *  Created on: 5 Jan 2022
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"


extern motor_data_t can_motors[24];

void config_motors()
{
	for (uint8_t i = 0; i < 24; i++)
	{
		can_motors[i].motor_type = 0;
		can_motors[i].rpm_pid.output 	= 0;
		can_motors[i].rpm_pid.integral 	= 0;
		can_motors[i].angle_pid.output 	= 0;
		can_motors[i].angle_pid.integral = 0;
		can_motors[i].angle_data.ticks 	= 0;
	}

#ifdef FR_MOTOR_ID
	uint8_t motor_id = FR_MOTOR_ID -1;
	can_motors[motor_id].motor_type = TYPE_M3508_STEPS;
	can_motors[motor_id].angle_data.gearbox_ratio = M3508_GEARBOX_RATIO;
	can_motors[motor_id].angle_data.center_ang 	= 0;
	can_motors[motor_id].angle_data.max_ang 	= 0;
	can_motors[motor_id].angle_data.min_ang 	= 0;
	can_motors[motor_id].angle_data.phy_max_ang =  2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_pid.kp 			= 0;
	can_motors[motor_id].angle_pid.ki			= 0;
	can_motors[motor_id].angle_pid.kd			= 0;
	can_motors[motor_id].angle_pid.int_max		= 0;
	can_motors[motor_id].angle_pid.max_out		= 0;
	can_motors[motor_id].angle_pid.min_out		= 0;
	can_motors[motor_id].angle_pid.physical_max	= M3508_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp 			= CHASSIS_KP;
	can_motors[motor_id].rpm_pid.ki				= CHASSIS_KI;
	can_motors[motor_id].rpm_pid.kd				= CHASSIS_KD;
	can_motors[motor_id].rpm_pid.int_max		= CHASSIS_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out		= CHASSIS_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.min_out		= -CHASSIS_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.physical_max	= M3508_MAX_OUTPUT;
#endif

#ifdef FL_MOTOR_ID
	motor_id = FL_MOTOR_ID -1;
	can_motors[motor_id].motor_type = TYPE_M3508;
	can_motors[motor_id].angle_data.gearbox_ratio = M3508_GEARBOX_RATIO;
	can_motors[motor_id].angle_data.center_ang 	= 0;
	can_motors[motor_id].angle_data.max_ang 	= 0;
	can_motors[motor_id].angle_data.min_ang 	= 0;
	can_motors[motor_id].angle_data.phy_max_ang =  2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_pid.kp 			= 0;
	can_motors[motor_id].angle_pid.ki			= 0;
	can_motors[motor_id].angle_pid.kd			= 0;
	can_motors[motor_id].angle_pid.int_max		= 0;
	can_motors[motor_id].angle_pid.max_out		= 0;
	can_motors[motor_id].angle_pid.min_out		= 0;
	can_motors[motor_id].angle_pid.physical_max	= M3508_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp 			= CHASSIS_KP;
	can_motors[motor_id].rpm_pid.ki				= CHASSIS_KI;
	can_motors[motor_id].rpm_pid.kd				= CHASSIS_KD;
	can_motors[motor_id].rpm_pid.int_max		= CHASSIS_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out		= CHASSIS_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.min_out		= -CHASSIS_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.physical_max	= M3508_MAX_OUTPUT;
#endif

#ifdef BL_MOTOR_ID
	motor_id = BL_MOTOR_ID -1;
	can_motors[motor_id].motor_type = TYPE_M3508;
	can_motors[motor_id].angle_data.gearbox_ratio = M3508_GEARBOX_RATIO;
	can_motors[motor_id].angle_data.center_ang 	= 0;
	can_motors[motor_id].angle_data.max_ang 	= 0;
	can_motors[motor_id].angle_data.min_ang 	= 0;
	can_motors[motor_id].angle_data.phy_max_ang =  2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_pid.kp 			= 0;
	can_motors[motor_id].angle_pid.ki			= 0;
	can_motors[motor_id].angle_pid.kd			= 0;
	can_motors[motor_id].angle_pid.int_max		= 0;
	can_motors[motor_id].angle_pid.max_out		= 0;
	can_motors[motor_id].angle_pid.min_out		= 0;
	can_motors[motor_id].angle_pid.physical_max	= M3508_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp 			= CHASSIS_KP;
	can_motors[motor_id].rpm_pid.ki				= CHASSIS_KI;
	can_motors[motor_id].rpm_pid.kd				= CHASSIS_KD;
	can_motors[motor_id].rpm_pid.int_max		= CHASSIS_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out		= CHASSIS_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.min_out		= 0;
	can_motors[motor_id].rpm_pid.physical_max	= M3508_MAX_OUTPUT;
#endif

#ifdef BR_MOTOR_ID

	motor_id = BR_MOTOR_ID -1;
	can_motors[motor_id].motor_type = TYPE_M3508;
	can_motors[motor_id].angle_data.gearbox_ratio = M3508_GEARBOX_RATIO;
	can_motors[motor_id].angle_data.center_ang 	= 0;
	can_motors[motor_id].angle_data.max_ang 	= 0;
	can_motors[motor_id].angle_data.min_ang 	= 0;
	can_motors[motor_id].angle_data.phy_max_ang =  2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_pid.kp 			= 0;
	can_motors[motor_id].angle_pid.ki			= 0;
	can_motors[motor_id].angle_pid.kd			= 0;
	can_motors[motor_id].angle_pid.int_max		= 0;
	can_motors[motor_id].angle_pid.max_out		= 0;
	can_motors[motor_id].angle_pid.min_out		= 0;
	can_motors[motor_id].angle_pid.physical_max	= M3508_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp 			= CHASSIS_KP;
	can_motors[motor_id].rpm_pid.ki				= CHASSIS_KI;
	can_motors[motor_id].rpm_pid.kd				= CHASSIS_KD;
	can_motors[motor_id].rpm_pid.int_max		= CHASSIS_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out		= CHASSIS_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.min_out		= 0;
	can_motors[motor_id].rpm_pid.physical_max	= M3508_MAX_OUTPUT;
#endif

#ifdef LFRICTION_MOTOR_ID
	motor_id = LFRICTION_MOTOR_ID -1;
	can_motors[motor_id].motor_type = TYPE_M3508;
	can_motors[motor_id].angle_data.gearbox_ratio = M3508_GEARBOX_RATIO;
	can_motors[motor_id].angle_data.center_ang 	= 0;
	can_motors[motor_id].angle_data.max_ang 	= 0;
	can_motors[motor_id].angle_data.min_ang 	= 0;
	can_motors[motor_id].angle_data.phy_max_ang =  2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_pid.kp 			= 0;
	can_motors[motor_id].angle_pid.ki			= 0;
	can_motors[motor_id].angle_pid.kd			= 0;
	can_motors[motor_id].angle_pid.int_max		= 0;
	can_motors[motor_id].angle_pid.max_out		= 0;
	can_motors[motor_id].angle_pid.min_out		= 0;
	can_motors[motor_id].angle_pid.physical_max	= M3508_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp 			= FRICTION_KP;
	can_motors[motor_id].rpm_pid.ki				= FRICTION_KI;
	can_motors[motor_id].rpm_pid.kd				= FRICTION_KD;
	can_motors[motor_id].rpm_pid.int_max		= FRICTION_MAX_INT;
	can_motors[motor_id].rpm_pid.max_out		= FRICTION_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.min_out		= 0;
	can_motors[motor_id].rpm_pid.physical_max	= M3508_MAX_OUTPUT;
#endif

#ifdef RFRICTION_MOTOR_ID
	motor_id = RFRICTION_MOTOR_ID -1;
	can_motors[motor_id].motor_type = TYPE_M3508;
	can_motors[motor_id].angle_data.gearbox_ratio = M3508_GEARBOX_RATIO;
	can_motors[motor_id].angle_data.center_ang 	= 0;
	can_motors[motor_id].angle_data.max_ang 	= 0;
	can_motors[motor_id].angle_data.min_ang 	= 0;
	can_motors[motor_id].angle_data.phy_max_ang =  2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_pid.kp 			= 0;
	can_motors[motor_id].angle_pid.ki			= 0;
	can_motors[motor_id].angle_pid.kd			= 0;
	can_motors[motor_id].angle_pid.int_max		= 0;
	can_motors[motor_id].angle_pid.max_out		= 0;
	can_motors[motor_id].angle_pid.min_out		= 0;
	can_motors[motor_id].angle_pid.physical_max	= M3508_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp 			= FRICTION_KP;
	can_motors[motor_id].rpm_pid.ki				= FRICTION_KI;
	can_motors[motor_id].rpm_pid.kd				= FRICTION_KD;
	can_motors[motor_id].rpm_pid.int_max		= FRICTION_MAX_INT;
	can_motors[motor_id].rpm_pid.max_out		= FRICTION_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.min_out		= 0;
	can_motors[motor_id].rpm_pid.physical_max	= M3508_MAX_OUTPUT;
#endif

#ifdef FEEDER_MOTOR_ID
	motor_id = FEEDER_MOTOR_ID -1;
	can_motors[motor_id].motor_type = TYPE_M2006;
	can_motors[motor_id].angle_data.gearbox_ratio = M2006_GEARBOX_RATIO;
	can_motors[motor_id].angle_data.center_ang 	= 0;
	can_motors[motor_id].angle_data.max_ang 	= PI*5000;	//so it can rotate 5000 times oops
	can_motors[motor_id].angle_data.min_ang 	= -PI*5000;
	can_motors[motor_id].angle_data.phy_max_ang =  2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_pid.kp 			= FEEDER_ANGLE_KP;
	can_motors[motor_id].angle_pid.ki			= FEEDER_ANGLE_KP;
	can_motors[motor_id].angle_pid.kd			= FEEDER_ANGLE_KP;
	can_motors[motor_id].angle_pid.int_max		= FEEDER_ANGLE_INT_MAX;
	can_motors[motor_id].angle_pid.max_out		= FEEDER_MAX_RPM;
	can_motors[motor_id].angle_pid.min_out		= 0;
	can_motors[motor_id].angle_pid.physical_max	= M2006_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp 			= FEEDER_KP;
	can_motors[motor_id].rpm_pid.ki				= FEEDER_KI;
	can_motors[motor_id].rpm_pid.kd				= FEEDER_KD;
	can_motors[motor_id].rpm_pid.int_max		= FEEDER_MAX_INT;
	can_motors[motor_id].rpm_pid.max_out		= FEEDER_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.min_out		= 0;
	can_motors[motor_id].rpm_pid.physical_max	= M2006_MAX_OUTPUT;
#endif

#ifdef PITCH_MOTOR_ID
	motor_id = PITCH_MOTOR_ID -1;
	can_motors[motor_id].motor_type = TYPE_GM6020;
	can_motors[motor_id].angle_data.gearbox_ratio = 0;
	can_motors[motor_id].angle_data.center_ang 	= PITCH_CENTER;
	can_motors[motor_id].angle_data.max_ang 	= PITCH_MAX_ANG;
	can_motors[motor_id].angle_data.min_ang 	= PITCH_MIN_ANG;
	can_motors[motor_id].angle_data.phy_max_ang =  2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI;
	can_motors[motor_id].angle_pid.kp 			= PITCH_ANGLE_KP;
	can_motors[motor_id].angle_pid.ki			= PITCH_ANGLE_KI;
	can_motors[motor_id].angle_pid.kd			= PITCH_ANGLE_KD;
	can_motors[motor_id].angle_pid.int_max		= PITCH_ANGLE_INT_MAX;
	can_motors[motor_id].angle_pid.max_out		= PITCH_MAX_RPM;
	can_motors[motor_id].angle_pid.min_out		= 0;
	can_motors[motor_id].angle_pid.physical_max	= GM6020_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp 			= PITCHRPM_KP;
	can_motors[motor_id].rpm_pid.ki				= PITCHRPM_KI;
	can_motors[motor_id].rpm_pid.kd				= PITCHRPM_KD;
	can_motors[motor_id].rpm_pid.int_max		= PITCHRPM_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out		= PITCH_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.min_out		= 0;
	can_motors[motor_id].rpm_pid.physical_max	= GM6020_MAX_OUTPUT;
#endif

#ifdef YAW_MOTOR_ID
	motor_id = YAW_MOTOR_ID -1;
	can_motors[motor_id].motor_type = TYPE_GM6020;
	can_motors[motor_id].angle_data.gearbox_ratio = 0;//1;
	can_motors[motor_id].angle_data.center_ang 	= YAW_CENTER;
	can_motors[motor_id].angle_data.max_ang 	= YAW_MAX_ANG;
	can_motors[motor_id].angle_data.min_ang 	= YAW_MIN_ANG;
	can_motors[motor_id].angle_data.phy_max_ang =  2 * PI;
	can_motors[motor_id].angle_data.phy_min_ang = -2 * PI; //angle before it overflows
	can_motors[motor_id].angle_pid.kp 			= YAW_ANGLE_KP;
	can_motors[motor_id].angle_pid.ki			= YAW_ANGLE_KI;
	can_motors[motor_id].angle_pid.kd			= YAW_ANGLE_KD;
	can_motors[motor_id].angle_pid.int_max		= YAW_ANGLE_INT_MAX;
	can_motors[motor_id].angle_pid.max_out		= YAW_MAX_RPM;
	can_motors[motor_id].angle_pid.min_out		= 0;
	can_motors[motor_id].angle_pid.physical_max	= GM6020_MAX_RPM;
	can_motors[motor_id].rpm_pid.kp 			= YAWRPM_KP;
	can_motors[motor_id].rpm_pid.ki				= YAWRPM_KI;
	can_motors[motor_id].rpm_pid.kd				= YAWRPM_KD;
	can_motors[motor_id].rpm_pid.int_max		= YAWRPM_INT_MAX;
	can_motors[motor_id].rpm_pid.max_out		= YAW_MAX_CURRENT;
	can_motors[motor_id].rpm_pid.min_out		= 0;
	can_motors[motor_id].rpm_pid.physical_max	= GM6020_MAX_OUTPUT;
#endif
}


void motor_calib_task(void* argument)
{
	can_start(&hcan1, 0x00000000, 0x00000000);
	can_start(&hcan2, 0x00000000, 0x00000000);
	config_motors();
	//insert can tester?
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		//future calibration code, if any
		// task takes highest priority over....everything so make sure to kill all motors first!

	}
	//write in task here to calibrate then stop lol
}
