#ifndef MPUPARA_H					/* prevent circluar	inclusions	*/
#define MPUPARA_H					/* by using protection macros */


#include <math.h>
#include <stdio.h>
#include  "xiic.h"
#include  "xiic_l.h"
#include  "xiic.h"
// #include "xiic.c"

typedef enum
{
  MPU6050_ACCEL_ONLY = 0x00,    // Accelerometer only mode             //
  MPU6050_GYRO_ONLY,            // Gyroscope only mode                 //
  MPU6050_CYCLE,                // Cycle Mode - Sample at a fixed rate //
  MPU6050_SLEEP,                // Sleep mode - Everything Disabled    //
  MPU6050_POWER_ON,             // Power ON - Everything ON            //
} MPU6050_power_conf;

typedef struct MPU6050_configuration{
  float gyro_sensitivity;
  uint8_t power_mode;
  uint8_t freq;
  uint8_t irq_enable;
} MPU6050_config_t;

typedef enum
{
  MPU6050_Gyroscope_250_deg = 0x00,  // Range is +- 250 degrees/s  //
  MPU6050_Gyroscope_500_deg = 0x01,  // Range is +- 500 degrees/s  //
  MPU6050_Gyroscope_1000_deg = 0x02, // Range is +- 1000 degrees/s //
  MPU6050_Gyroscope_2000_deg = 0x03  // Range is +- 2000 degrees/s //
} MPU6050_Gyroscope_conf;


MPU6050_config_t mpu6050_config;      // Current configuration of MPU6050 //

#define MPU6050_DEVICE_ID   0x68
#define MPU6050_ADDR 0x68

#define DEBUG_MPU6050           // Activate when more messages are required //


#define SELF_TEST_X 			0x0D
#define SELF_TEST_Y 			0x0E
#define SELF_TEST_Z 			0x0F
#define SELF_TEST_A 			0x10

//  specifies the divider from the gyroscope output rate used to generate the Sample Rate for the MPU-60X0
#define SMPRT_DIV   			0x19

//  configures the external Frame Synchronization (FSYNC) pin sampling and the Digital Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers
#define CONFIG 					0x1A

// used to trigger gyroscope, accelerometer self test and configure the gyroscope and accelerometer full scale range.
#define GYRO_CONFIG 			0x1B
#define ACCEL_CONFIG 			0x1C

// determines which sensor measurements are loaded into the FIFO buffer
#define FIFO_EN		 			0x23

// configures the auxiliary I2C bus for single-master or multi-master control.
#define I2C_MST_CTRL 			0x24

// configure the data transfer sequence for Slave 0.
#define I2C_SLV0_ADDR			0x25
#define I2C_SLV0_REG			0x26
#define I2C_SLV0_CTRL			0x27

// shows the status of the interrupt generating signals in the I2C Master within the MPU60X0.
#define I2C_MST_STATUS			0x36

// configures the behavior of the interrupt signals at the INT pins.
#define INT_PIN_CFG				0x37

//  enables interrupt generation by interrupt sources
#define INT_ENABLE				0x38

// shows the interrupt status of each interrupt generation source.
#define INT_STATUS				0x3A


// store the most recent gyroscope measurements. 
#define GYRO_XOUT_H				0x43
#define	GYRO_XOUT_L				0x44
#define GYRO_YOUT_H				0x45
#define GYRO_YOUT_L				0x46
#define GYRO_ZOUT_H				0x47
#define GYRO_ZOUT_L				0x48

// store data read from external sensors by the Slave 0, 1, 2, and 3 on the auxiliary I2C interface
#define EXT_SENS_DATA_00		0x49
#define EXT_SENS_DATA_01		0x4A
#define EXT_SENS_DATA_02		0x4B
#define EXT_SENS_DATA_03		0x4C
#define EXT_SENS_DATA_04		0x4D
#define EXT_SENS_DATA_05		0x4E
#define EXT_SENS_DATA_06		0x4F
#define EXT_SENS_DATA_07		0x50
#define EXT_SENS_DATA_08		0x51
#define EXT_SENS_DATA_09		0x52
#define EXT_SENS_DATA_10		0x53
#define EXT_SENS_DATA_11		0x54
#define EXT_SENS_DATA_12		0x55
#define EXT_SENS_DATA_13		0x56
#define EXT_SENS_DATA_14		0x57
#define EXT_SENS_DATA_15		0x58
#define EXT_SENS_DATA_16		0x59
#define EXT_SENS_DATA_17		0x5A
#define EXT_SENS_DATA_18		0x5B
#define EXT_SENS_DATA_19		0x5C
#define EXT_SENS_DATA_20		0x5D
#define EXT_SENS_DATA_21		0x5E
#define EXT_SENS_DATA_22		0x5F
#define EXT_SENS_DATA_23		0x60

// holds the output data written into Slave when the Slave is set to write mode
#define I2C_SLV0_DO				0x63
#define I2C_SLC1_DO				0x64

// specify the timing of external sensor data shadowing
#define I2C_MST_DELAY_CTRL		0x67
// reset the analog and digital signal paths of the gyroscope, accelerometer,and temperature sensors
// #define SIGNAL_PATH_RESET		0x68
//  enable and disable the FIFO buffer
#define USER_CTRL				0x6A

// to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define PWR_MGMT_1				0x6B
#define PWR_WAKE_UP       0x00
#define ENABLE_SLEEP      0x40
#define ENABLE_CYCLE      0x20
#define TEMP_DIS          0x08    // Disable temperature sensor //

#define PWR_MGMT_2				0x6C

// Gyro sensitivities in g/s //
#define GYRO_SENS_250_DEG	  ((float) 131.0)
#define GYRO_SENS_500_DEG	  ((float) 65.5)
#define GYRO_SENS_1K_DEG	  ((float) 32.8)
#define GYRO_SENS_2K_DEG	  ((float) 16.4)

// Gyro register value for sensitivity //
#define GYRO_SCALE_250_DEG    0x00
#define GYRO_SCALE_500_DEG    0x08
#define GYRO_SCALE_1K_DEG     0x10
#define GYRO_SCALE_2K_DEG     0x18



// keep track of the number of samples currently in the FIFO buffer. 
#define FIFO_COUNTH				0x72
#define FIFO_COUNTL				0x73

// used to read and write data from the FIFO buffer.
#define FIFO_R_W				0x74

// used to verify the identity of the device.
#define WHO_AM_I				0x75

// MPU6050 Return codes //
#define MPU6050_SUCCESS   0
#define MPU6050_FAILURE   1





// uint8_t MPU6050_Init(XIic* IICInst, uint8_t gyro_config, uint8_t sample_rate );


#endif

