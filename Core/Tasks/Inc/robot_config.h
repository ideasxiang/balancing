/*
 * robot_config.h
 *
 *  Created on: 19 Jan 2021
 *      Author: Hans Kurnia
 */

#ifndef TASKS_INC_ROBOT_CONFIG_H_
#define TASKS_INC_ROBOT_CONFIG_H_


#define MOTOR_ONLINE_CHECK 	0

/*********************** REFEREE SYSTEM CONFIGURATION *******************/

#define MOTOR_CURRENT_RATIO		(CHASSIS_MAX_CURRENT / 10000) 	// 10000 mA at max current
#define PROJECTILE_SPEED_RATIO	360								//rpm per m/s of the friction wheels ish don't think this will work well lmao
#define FEEDER_SPEED_RATIO		5								//projectiles per round of the feeder

#define LV1_FEEDER				60//480
#define	LV1_PROJECTILE			0//20//12
#define LV1_POWER				50

#define LV2_FEEDER				240
#define	LV2_PROJECTILE			0//12
#define LV2_POWER				50

#define LV3_FEEDER				240
#define	LV3_PROJECTILE			0//12
#define LV3_POWER				50

#define CHASSIS_POWER_MULT		1


/*********************** MANUAL CONTROL CONFIGURATION *******************/
//Inverts for both keyboard and mouse controls
#define YAW_INVERT  			-1				//1 to invert control -1 to disable
#define PITCH_INVERT  			1				//1 to invert control -1 to disable
#define YAW_SPEED 	 			0.05 			//Speed of gimbal yaw turning
#define PITCH_SPEED 	 		0.05//0.005		//Speed of gimbal pitch turning

#define MOUSE_X_SENSITIVITY		100				//Speed of yaw turning with mouse, dependent on above speed
#define MOUSE_X_INVERT			1				//Set to -1 if it needs to be inverted
#define MOUSE_Y_SENSITIVITY 	100				//Speed of pitch turning with mouse,  dependent on above speed
#define	MOUSE_Y_INVERT			-1				//Set to -1 if it needs to be inverted

#define MOUSE_LIMIT 			200

#define KEYBD_MAX_SPD 			0.5				//% of max speed


#define GIMBAL_MODE 			1				//1 for IMU control, 0 for absolute angle based control

/*********************** AIMBOT CONFIGURATION *******************/
#define AIMBOT_YAW_MULT 		1				//Multiplier for X axis
#define AIMBOT_PIT_MULT 		0.5625			//Multiplier for Y axis
#define XAVIER_TIMEOUT 			100				//Time before robot returns to manual control

#define AIMBOT_Y_OFFSET			500				//Y point for the robot to aim at
#define AIMBOT_Y_KP				1
#define AIMBOT_Y_KI				0
#define AIMBOT_Y_KD				0

#define AIMBOT_X_OFFSET			0				//X Point for the robot to aim at
#define AIMBOT_X_KP				1
#define AIMBOT_X_KI				0
#define AIMBOT_X_KD				0
#define FOV_MULT				(0.747/2)		//FOV of the camera in radians, change depending on lens specs


#define OBC_DATA_SIZE			8				//Packet size


/* PID TUNING GUIDE
 * For all motors, there are 2 different PID values, angle PID and speed PID
 * For motors that require position control, both values have to be set
 *
 * Speed PID is the main control loop which determines how much current
 * to send to the motors. i.e. it sets the target speed for the motors
 * Angle PID calculates the RPM the motor should be running at, then runs the
 * target values through the speed PID loop
 *
 * Generally, the speed control loop should be PID,
 * while the angle control loop can just be a P control
 *
 * TO TUNE PID
 * Tune speed loop FIRST, if not the angle loop might resonate and cause it to oscillate instead
 */



/*********************** FEEDBACK LOOP PERIODS *********************/
#define CONTROL_DELAY 			2
#define CHASSIS_DELAY 			2
#define BALANCE_DELAY 			10


/*********************** CHASSIS CONFIGURATION ***********************/

#define B_ANGLE_KP			600  //900 0 500	400 20 250  // 600 60 1200
#define B_ANGLE_KI 			60
#define B_ANGLE_KD  		1200
#define B_ANGLE_INT_MAX 	5000  // change maybe?
#define B_ANGLE_MAX_OUT		10000
#define ANGLE_OFFSET		-5   // offset for actual balance point of robot  //-4.2
#define CG_radius			0.047  // CG_height from wheel axis

#define B_ACCEL_KP			3//0.0000001	//2 0 0.05  //1 0 0.7 // 1.5 0.01 0.06 // 1 0.8 0.5
#define B_ACCEL_KI 			0.2//0.00000001
#define B_ACCEL_KD			0.1
#define B_ACCEL_INT_MAX		2   // Max angle
#define B_ACCEL_MAX_OUT		15

#define B_ORIENT_KP			0.1 // 0.1 0 0.0001
#define B_ORIENT_KI 			0
#define B_ORIENT_KD			0.0001 // gives oscillations at P=1 D=0.5
#define B_ORIENT_INT_MAX		1000   // Max angle
#define B_ORIENT_MAX_OUT		1000

#define CHASSIS_KP  		1					// |
#define CHASSIS_KI  		0//0.05				// | - CHASSIS WHEELS PID VALUES
#define CHASSIS_KD  		0.1 //5				// |
#define CHASSIS_INT_MAX  	10000				// |
#define CHASSIS_MAX_CURRENT 10000
#define CHASSIS_MIN_CURRENT 0

#define CHASSIS_YAW_MAX_RPM	20					//pls make sure this rpm * gear box does not exceed m3508 max RPM
#define CHASSIS_YAW_KP 		10
#define CHASSIS_YAW_KI		0
#define CHASSIS_YAW_KD 		1
#define TURNING_SPEED		100 				//Rotation speed of robot
#define MAX_SPEED 			1 				//Max speed of robot in m/s


/*********************** LAUNCHER CONFIGURATION ***********************/
#define FEEDER_KP 			10				// |
#define FEEDER_KI  			0.02				// | - FEEDER PID VALUES
#define FEEDER_KD  			5			// |
#define FEEDER_MAX_INT		10000

#define FEEDER_ANGLE_KP 			1			// |
#define FEEDER_ANGLE_KD  			0			// | - FEEDER_ANGLE PID VALUES
#define FEEDER_ANGLE_KI  			0			// |
#define FEEDER_ANGLE_INT_MAX  		0			// |
#define FEEDER_MAX_RPM				100			// |
#define FEEDER_JAM_TORQUE  		9800			//Before feeder deemed to be jammed
#define FEEDER_UNJAM_SPD  		-50				//Reverse unjam
#define FEEDER_UNJAM_TIME		100
#define FEEDER_MAX_CURRENT		10000
#define FEEDER_CUTOFF_TEMP  	60
#define STEPPER_ANGLE			1.8


#define FRICTION_KP  			1			// |
#define FRICTION_KI  			0.002			// | - FRICTION WHEELS PID VALUES
#define FRICTION_KD  			10		// |
#define FRICTION_MAX_CURRENT 	16384
#define FRICTION_MAX_INT		10000
#define FRICTION_INVERT			-1
#define LAUNCHER_MARGIN			200
#define LAUNCHER_DIFF_MARGIN	200

#define CLEAR_DELAY				1000



/* To configure centers, start the boards in debug mode with all motors
 * powered *but in safe mode* (i.e. remotes off)
 * Physically push the motors to the desired centers
 * and put a breakpoint/live expression on their respective real_ang variables
 * from their raw_data structs
 * The centers should be from 0 to 8192, it should be the value directly from
 * the motors
 */
/*********************** GIMBAL CONFIGURATION ***********************/
#define PITCH_ANGLE_KP	  		200
#define PITCH_ANGLE_KD  		00
#define PITCH_ANGLE_KI  		0
#define PITCH_ANGLE_INT_MAX		1000

#define PITCHRPM_KP				700
#define PITCHRPM_KI				0.1
#define PITCHRPM_KD				0
#define PITCHRPM_INT_MAX		10000
#define PITCH_MAX_RPM			200
#define PITCH_MAX_CURRENT		20000

#define PITCH_CENTER			7500
#define PITCH_MAX_ANG			0.5
#define PITCH_MIN_ANG			-0.40

#define YAW_ANGLE_KP			200//300//100
#define YAW_ANGLE_KD			0
#define YAW_ANGLE_KI			0
#define YAW_ANGLE_INT_MAX		100

#define YAWRPM_KP				400
#define YAWRPM_KI				0.01
#define YAWRPM_KD				0
#define YAWRPM_INT_MAX			5000
#define YAW_MAX_RPM				400
#define YAW_MAX_CURRENT			20000

#define YAW_CENTER 				4096
#define YAW_MAX_ANG				(4*PI)
#define YAW_MIN_ANG				(-4*PI)



#define STARTUP_DELAY 		100
#define LOW_FREQ 			440
#define HIGH_FREQ			880
#define BUZZER_DELAY 		70

#define REMOTE_TIMEOUT 		200


/*********************** MOTOR CONFIGURATION *******************/
//CAN ids for the motors, for motors on the CAN2 bus, add 12
//ADD 4 TO GM6020 IDS i.e. flashing 5 times = ID 9
//#define CHASSIS_MCU

#ifndef CHASSIS_MCU
#define FR_MOTOR_ID 		13
#define FL_MOTOR_ID 		14
#define BL_MOTOR_ID 		15
#define BR_MOTOR_ID 		16
#endif
#define FEEDER_MOTOR_ID		7
#define LFRICTION_MOTOR_ID	5
#define RFRICTION_MOTOR_ID	6

//NOTE: two motors CANNOT have the same __flashing__ number (i.e. GM6020 id 9 cannot be used
//with any id 6 motors
#define PITCH_MOTOR_ID 		9
#ifndef CHASSIS_MCU
#define YAW_MOTOR_ID 		22
#endif



/* MECANUM WHEEL PROPERTIES */
#define WHEEL_CIRC			30    // change to 300mm wheel circumference

#define FR_ANG_X			-PI/4
#define FR_ANG_Y 			-PI/2
#define FR_ANG_PASSIVE		PI/4
#define FR_DIST				312
#define FR_VX_MULT			-1		//-cos(FR_ANG_Y - FR_ANG_PASSIVE)/sin(FR_ANG_PASSIVE)
#define FR_VY_MULT			-1		//-sin(FR_ANG_Y - FR_ANG_PASSIVE)/sin(FR_ANG_PASSIVE)
#define FR_YAW_MULT			5.861		//((-FR_DIST * sin(FR_ANG_Y - FR_ANG_PASSIVE - FR_ANG_X)) / (sin(FR_ANG_PASSIVE) * WHEEL_CIRC))


#define FL_ANG_X			PI/4
#define FL_ANG_Y 			PI/2
#define FL_ANG_PASSIVE		-PI/4
#define FL_DIST				316
#define FL_VX_MULT			-1 		//-cos(FL_ANG_Y - FL_ANG_PASSIVE)/sin(FL_ANG_PASSIVE)
#define FL_VY_MULT			1		//-sin(FL_ANG_Y - FL_ANG_PASSIVE)/sin(FL_ANG_PASSIVE)
#define FL_YAW_MULT			5.861	//((-FL_DIST * sin(FL_ANG_Y - FL_ANG_PASSIVE - FL_ANG_X)) / (sin(FL_ANG_PASSIVE) * WHEEL_CIRC))

#define BL_ANG_X			(3*PI/4)
#define BL_ANG_Y 			PI/2
#define BL_ANG_PASSIVE		PI/4
#define BL_DIST				312
#define BL_VX_MULT			1		//-cos(BL_ANG_Y - BL_ANG_PASSIVE)/sin(BL_ANG_PASSIVE)
#define BL_VY_MULT			1		//-sin(BL_ANG_Y - BL_ANG_PASSIVE)/sin(BL_ANG_PASSIVE)
#define BL_YAW_MULT			5.786	//((-BL_DIST * sin(BL_ANG_Y - BL_ANG_PASSIVE - BL_ANG_X)) / (sin(BL_ANG_PASSIVE) * WHEEL_CIRC))

#define BR_ANG_X			-(3*PI/4)
#define BR_ANG_Y 			-PI/2
#define BR_ANG_PASSIVE		-PI/4
#define	BR_DIST				312
#define BR_VX_MULT			1		//-cos(BR_ANG_Y - BR_ANG_PASSIVE)/sin(BR_ANG_PASSIVE)
#define BR_VY_MULT			-1		//-sin(BR_ANG_Y - BR_ANG_PASSIVE)/sin(BR_ANG_PASSIVE)
#define BR_YAW_MULT			5.786	//((-BR_DIST * sin(BR_ANG_Y - BR_ANG_PASSIVE - BR_ANG_X)) / (sin(BR_ANG_PASSIVE) * WHEEL_CIRC)) * 19



/*********************** OTHERS ***********************/

#define MAX_RC_VALUE 		1320 //660 * 2
#define RC_LIMITS			660

//microsecond timer used for PIDs
#define TIMER_FREQ			10000 //Cannot be too high if not the ISRs overload the CPU

//switch goes from 1-3-2 from top to down
enum left_switch
{
	launcher_on = 1,
	launcher_off = 2,
	aimbot_enable = 3
};

enum right_switch
{
	all_off = 1,
	all_on = 2,
	gimbal_on = 3
};


#define KEY_OFFSET_W        ((uint16_t)0x01<<0)
#define KEY_OFFSET_S        ((uint16_t)0x01<<1)
#define KEY_OFFSET_A 		((uint16_t)0x01<<2)
#define KEY_OFFSET_D        ((uint16_t)0x01<<3)
#define KEY_OFFSET_Q        ((uint16_t)0x01<<6)
#define KEY_OFFSET_E        ((uint16_t)0x01<<7)
#define KEY_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define KEY_OFFSET_CTRL     ((uint16_t)0x01<<5)

#endif /* TASKS_INC_ROBOT_CONFIG_H_ */
