#include "mpu6050.h"
#include  "xiic.h"
#include  "xiic_l.h"
#include  "xiic.h"
// #include "xiic.c"

#define IIC_DEVICE_ID		XPAR_AXI_IIC_0_DEVICE_ID
#define IIC_BASE_ADDR		 XPAR_AXI_IIC_0_BASEADDR
//XIic 		IICInst;

/* Global Variables ----------------------------------------------------------*/

// uint8_t gyro_config;
// uint8_t sample_rate;

static uint8_t gyro_sensitiviy_config(uint8_t config)
{
  uint8_t reg_value; // Value to be written to the register //

  // Based on the configuration chosen by the user //
  // Choose the sensitivity value and the register value for the MPU6050 //
  switch (config) {
    case MPU6050_Gyroscope_250_deg:
      reg_value = GYRO_SCALE_250_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_250_DEG;
      break;
    case MPU6050_Gyroscope_500_deg:
      reg_value = GYRO_SCALE_500_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_500_DEG;
      break;
    case MPU6050_Gyroscope_1000_deg:
      reg_value = GYRO_SCALE_1K_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_1K_DEG;
      break;
    case MPU6050_Gyroscope_2000_deg:
      reg_value = GYRO_SCALE_2K_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_2K_DEG;
      break;
    default:
      reg_value = GYRO_SCALE_250_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_250_DEG;
      break;
  }

  return reg_value;
}

uint8_t MPU6050_Init(XIic* IICInst, uint8_t gyro_config, uint8_t sample_rate ) {
    uint8_t check;
    uint8_t reg_trx[2];

    // Step 1: Check device ID WHO_AM_I register
    // Read the WHO_AM_I register to verify the device ID
    XIic_Send(IIC_BASE_ADDR,GYRO_CONFIG, WHO_AM_I, &check, 1);

    // Verify the device ID
    if (check == MPU6050_DEVICE_ID) {
        // Step 2: Reset the MPU-6050 sensor through the PWR_MGMT_1 register
        reg_trx[0] = PWR_MGMT_1;
        reg_trx[1] = PWR_WAKE_UP;
        XIic_Recv(IIC_BASE_ADDR, MPU6050_ADDR, reg_trx, 2, 0);

        // Step 3: Set the power mode
        mpu6050_config.power_mode = MPU6050_POWER_ON;

        // Step 4: Set Sample Rate by writing SMPLRT_DIV register
        reg_trx[0] = SMPRT_DIV;
        reg_trx[1] = sample_rate;
        XIic_Recv(IIC_BASE_ADDR, MPU6050_ADDR, reg_trx, 2, 0);

        // Step 5: Set Gyro configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> Â± 250 Â°/s
        reg_trx[0] = GYRO_CONFIG;
        reg_trx[1] = gyro_sensitiviy_config(gyro_config);
        XIic_Recv(IIC_BASE_ADDR, MPU6050_ADDR, reg_trx, 2, 0);

        // Printing debug messages
        #ifdef DEBUG_MPU6050
            xil_printf("Config gyro is %f and reg val is %x\n", mpu6050_config.gyro_sensitivity, reg_trx[1]);
        #endif

        // Return success
        return MPU6050_SUCCESS;
    }

    // Return failure if device ID check fails
    return MPU6050_FAILURE;
}
