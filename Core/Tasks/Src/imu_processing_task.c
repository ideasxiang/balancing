/*
 * imu_processing_task.c
 *
 *  Created on: 9 Jan 2022
 *      Author: wx
 */
#include "board_lib.h"
#include "bsp_imu.h"
#include "robot_config.h"
#include "imu_processing_task.h"
//#define BOARD_DOWN
#define IMU_ORIENTATION	1
#define IMU_TARGET_TEMP	50
#define IMU_YAW_INVERT		-1
#define IMU_PITCH_INVERT	-1
#define IMU_ROLL_INVERT		1
//#define IST8310
static volatile float q0 = 1.0f;
static volatile float q1 = 0.0f;
static volatile float q2 = 0.0f;
static volatile float q3 = 0.0f;
static volatile float exInt, eyInt, ezInt; /* error integral */
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;

static volatile float gyrAngleX = 0;
static volatile float accAngleX = 0;
static volatile float compAngleX = 0;
//static uint32_t last_time[2];
static float debugangle;

#define AHRSKp 0.2f   // complementary factor
/*
 * proportional gain governs rate of
 * convergence to accelerometer/magnetometer
																*/
#define AHRSKi 0.0f                                             /*
 * integral gain governs rate of
 * convergence of gyroscope biases
*/

orientation_data_t imu_heading;
static accel_data_t accel_proc_data;
static gyro_data_t gyro_proc_data;
static mag_data_t mag_proc_data;
//QueueHandle_t gyro_data_queue;
//QueueHandle_t accel_data_queue;
//QueueHandle_t mag_data_queue;
TaskHandle_t imu_processing_task_handle1;
static uint32_t last_proc_times[2];
static uint8_t update_flag = 0;
static uint8_t rtos_started = 0;


void imu_proc_task_notif() {
	//resets the flags
	update_flag = 0b000;
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(imu_processing_task_handle1, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void gyro_data_ready(gyro_data_t gyro_data) {
	gyro_proc_data.gx = gyro_data.gx;
	gyro_proc_data.gy = gyro_data.gy;
	gyro_proc_data.gz = gyro_data.gz;
	gyro_proc_data.last_gyro_update = gyro_data.last_gyro_update;

	update_flag |= 1; //sets bit 0 to true
	//only allows task to be run when all the data is new
	if ((update_flag == 0b111|| update_flag == 0b011) && (rtos_started == 1)) {
		imu_proc_task_notif();
	}
}

void accel_data_ready(accel_data_t accel_data) {
	accel_proc_data.ax = accel_data.ax;
	accel_proc_data.ay = accel_data.ay;
	accel_proc_data.az = accel_data.az;
	accel_proc_data.last_accel_update = accel_data.last_accel_update;

	update_flag |= 1 << 1; //sets bit 1 to true
	//only allows task to be run when accel and gyro data are new
	if ((update_flag == 0b111 || update_flag == 0b011) && (rtos_started == 1)) {
		imu_proc_task_notif();
	}
}

void mag_data_ready(mag_data_t mag_data) {
	mag_proc_data.mx = mag_data.mx;
	mag_proc_data.my = mag_data.my;
	mag_proc_data.mz = mag_data.mz;
	mag_proc_data.last_mag_update = mag_data.last_mag_update;

	update_flag |= 1 << 2;
	if (update_flag == 0b111 && rtos_started == 1) {
		//disabled as magnetometer data is not used
		//imu_proc_task_notif();
	}
}

void imu_processing_task(void *argument) {
	//imu_start_ints();
	rtos_started = 1;
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	init_quaternion();
	quat_startup();
	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		imu_ahrs_update();
		imu_attitude_update();
	}
}

void init_quaternion(void) {
	int16_t hx, hy; //hz;

	hx = mag_proc_data.mx;
	hy = mag_proc_data.my;
//	hz = mag_proc_data.mz;

#ifdef BOARD_DOWN
	if (hx < 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}

	}
	else if (hx < 0 && hy > 0)
	{
		if (fabs(hx / hy)>=1)
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}

	}
	else if (hx > 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}

	}
	else if (hx > 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}
	}
	#else
	if (hx < 0 && hy < 0) {
		if (fabs(hx / hy) >= 1) {
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		} else {
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}

	} else if (hx < 0 && hy > 0) {
		if (fabs(hx / hy) >= 1) {
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		} else {
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}

	} else if (hx > 0 && hy > 0) {
		if (fabs(hx / hy) >= 1) {
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		} else {
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}

	} else if (hx > 0 && hy < 0) {
		if (fabs(hx / hy) >= 1) {
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;
		} else {
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}
	}
#endif


}
//
//void quat_startup(void) {
////to figure out how to use quaternions
//	float norm;
//	float hx, hy, hz, bx, bz;
//	float wx, wy, wz;
//	float tempq0, tempq1, tempq2, tempq3;
//
//
//	float q0q1 = q0 * q1;
//	float q0q2 = q0 * q2;
//	float q0q3 = q0 * q3;
//	float q1q1 = q1 * q1;
//	float q1q2 = q1 * q2;
//	float q1q3 = q1 * q3;
//	float q2q2 = q2 * q2;
//	float q2q3 = q2 * q3;
//	float q3q3 = q3 * q3;
//
//	gx = gyro_proc_data.gx;
//	gy = gyro_proc_data.gy;
//	gz = gyro_proc_data.gz;
//	ax = accel_proc_data.ax;
//	ay = accel_proc_data.ay;
//	az = accel_proc_data.az;
//	mx = mag_proc_data.mx;
//	my = mag_proc_data.my;
//	mz = mag_proc_data.mz;
//
//	/* Fast inverse square-root */
//	norm = inv_sqrt(ax * ax + ay * ay + az * az);
//	ax = ax * norm;
//	ay = ay * norm;
//	az = az * norm;
//
//#ifdef IST8310
//		norm = inv_sqrt(mx*mx + my*my + mz*mz);
//		mx = mx * norm;
//		my = my * norm;
//		mz = mz * norm;
//	#else
//	mx = 0;
//	my = 0;
//	mz = 0;
//#endif
//	/* compute reference direction of flux */
//	hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
//	hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
//	hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
//	bx = sqrt((hx * hx) + (hy * hy));
//	bz = hz;
//
//	/* estimated direction of gravity and flux (v and w) */
//	wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
//	wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
//	wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);
//
//
//	/* PI */
//
//	tempq0 = (-q1 * wx - q2 * wy - q3 * wz);
//	tempq1 = (q0 * wx + q2 * wz - q3 * wy);
//	tempq2 = (q0 * wy - q1 * wz + q3 * wx);
//	tempq3 = (q0 * wz + q1 * wy - q2 * wx);
//
//	/* normalise quaternion */
//	norm = inv_sqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
//	q0 = tempq0 * norm;
//	q1 = tempq1 * norm;
//	q2 = tempq2 * norm;
//	q3 = tempq3 * norm;
//}
void quat_startup(void) { // new
//to figure out how to use quaternions
  float norm;
  float vx, vy, vz, wx, wy, wz;
  float tempq0, tempq1, tempq2, tempq3;

  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q1q1 = q1 * q1;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;

  gx = gyro_proc_data.gx;
  gy = gyro_proc_data.gy;
  gz = gyro_proc_data.gz;
  ax = accel_proc_data.ax;
  ay = accel_proc_data.ay;
  az = accel_proc_data.az;
  mx = mag_proc_data.mx;
  my = mag_proc_data.my;
  mz = mag_proc_data.mz;

  /* Fast inverse square-root */
  norm = inv_sqrt(ax * ax + ay * ay + az * az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  /* estimated direction of gravity  (v) */
  vx = 2.0f * (q1q3 - q0q2);
  vy = 2.0f * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = (ay * vz - az * vy);
  wy = (az * vx - ax * vz);
  wz = (ax * vy - ay * vx);
  /* PI */
  tempq0 =  q0 + (-q1 * wx - q2 * wy - q3 * wz);
  tempq1 =  q1 +(q0 * wx + q2 * wz - q3 * wy);
  tempq2 =  q2 +(q0 * wy - q1 * wz + q3 * wx);
  tempq3 =  q3 +(q0 * wz + q1 * wy - q2 * wx);

  /* normalise quaternion */
  norm = inv_sqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;
}

void imu_ahrs_update(void) {
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0, tempq1, tempq2, tempq3;

	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	gx = gyro_proc_data.gx;
	gy = gyro_proc_data.gy;
	gz = gyro_proc_data.gz;
	ax = accel_proc_data.ax;
	ay = accel_proc_data.ay;
	az = accel_proc_data.az;
	mx = mag_proc_data.mx;
	my = mag_proc_data.my;
	mz = mag_proc_data.mz;

	last_proc_times[0] = HAL_GetTick(); //ms
	halfT = ((float) (last_proc_times[0] - last_proc_times[1]) / 2000.0f);
	last_proc_times[1] = last_proc_times[0];

	/* Fast inverse square-root */
	norm = inv_sqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

#ifdef IST8310
		norm = inv_sqrt(mx*mx + my*my + mz*mz);
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm;
	#else
	mx = 0;
	my = 0;
	mz = 0;
#endif
	/* compute reference direction of flux */
	hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
	hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
	hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;

	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
	wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
	wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

	/*
	 * error is sum of cross product between reference direction
	 * of fields and direction measured by sensors
	 */
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	/* PI */
	if (ex != 0.0f && ey != 0.0f && ez != 0.0f) {
		exInt = exInt + ex * AHRSKi * halfT;
		eyInt = eyInt + ey * AHRSKi * halfT;
		ezInt = ezInt + ez * AHRSKi * halfT;

		gx = gx + AHRSKp * ex + exInt;
		gy = gy + AHRSKp * ey + eyInt;
		gz = gz + AHRSKp * ez + ezInt;
	}

	tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	tempq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	tempq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	tempq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

	/* normalise quaternion */
	norm = inv_sqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}

void imu_attitude_update(void) {
	if (IMU_ORIENTATION == 0) {
		/* yaw    -pi----pi */
		imu_heading.yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3,
				-2 * q2 * q2 - 2 * q3 * q3 + 1) * IMU_YAW_INVERT;
		/* pitch  -pi/2----pi/2 */
		imu_heading.pit = -asin(-2 * q1 * q3 + 2 * q0 * q2) * IMU_PITCH_INVERT;
		/* roll   -pi----pi  */
		imu_heading.rol = atan2(2 * q2 * q3 + 2 * q0 * q1,
				-2 * q1 * q1 - 2 * q2 * q2 + 1) * IMU_ROLL_INVERT;
	} else if (IMU_ORIENTATION == 1) {
		/* yaw    -pi----pi */
		imu_heading.yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3,
				-2 * q2 * q2 - 2 * q3 * q3 + 1) * IMU_YAW_INVERT * 57.2958;
		/* pitch  -pi/2----pi/2 */
		imu_heading.rol = -asin(-2 * q1 * q3 + 2 * q0 * q2) * IMU_ROLL_INVERT * 57.2958;
		/* roll   -pi----pi  */
		imu_heading.pit = atan2(2 * q2 * q3 + 2 * q0 * q1,
				-2 * q1 * q1 - 2 * q2 * q2 + 1) * IMU_PITCH_INVERT;

		debugangle = imu_heading.pit* 57.2958;
//		imu_heading.pit = (179-fabs(imu_heading.pit))*-copysign(1, imu_heading.pit);

//		gx = gyro_proc_data.gx;
//		gy = gyro_proc_data.gy;
//		gz = gyro_proc_data.gz;
//		ax = accel_proc_data.ax;
//		ay = accel_proc_data.ay;
//		az = accel_proc_data.az;
//
//		last_time[0] = HAL_GetTick(); //ms
//		float deltaT = ((float) (last_time[0] - last_time[1]) / 1000.0f);
//		gyrAngleX += gx * deltaT;
//
//		accAngleX = atan(copysign(1, az)*(ay - 45)/ sqrt(pow((ax - 15), 2) + pow(az, 2)));  // offsets from ax and ay
//
//		compAngleX = (0.98 * gyrAngleX + 0.02 * accAngleX )* 57.2958;
//		last_time[1] = last_time[0];

	}

}
