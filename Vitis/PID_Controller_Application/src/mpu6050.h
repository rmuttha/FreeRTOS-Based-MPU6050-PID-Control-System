#ifndef MPUPARA_H					/* prevent circluar	inclusions	*/
#define MPUPARA_H					/* by using protection macros */

#include  "xiic.h"

typedef struct {
	uint8_t target_angle;
	uint8_t mode;
	uint8_t valid;
	uint8_t currAngle;
	uint8_t roll;
}taskParameters_t;

#define IIC_BADDR				XPAR_IIC_0_BASEADDR

#define MPU6050_DEVICE_ID   0x68
#define MPU6050_ADDR 		0x68

// used to trigger gyroscope, accelerometer self test and configure the gyroscope and accelerometer full scale range.
#define GYRO_CONFIG 			0x1B

// store the most recent gyroscope measurements. 
#define GYRO_XOUT_H				0x43

#define TEMP_OUT_H				0x41

#define SET_AXIS_X 0x04

#define RUN		   1
#define TEST	   2
#define EXIT	   3

// to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define PWR_MGMT_1		  0x6B

// used to verify the identity of the device.
#define WHO_AM_I				0x75

// In mpu6050.h
void mpu6050_getGyroData(XIic* IICInst, u8 *angle_actual, int axis);
//void mpu6050_getGyroData(XIic* IICInst, u8 *angle_actual, int axis);
void mpu6050_gyroCfg(XIic* IICInst);
void mpu6050_init(XIic* IICInst);
void mpu6050_setSleepMode(XIic *IICInst);
void mpu6050_clearSleepMode(XIic* IICInst);
void gettempData(XIic* IICInst);

#endif

