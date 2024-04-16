/**
* @file PID_Project.c
*
* @author Raksha Mairpady and Ruthuja Mutta
*
* @brief 
*
******************************************************************************/
/*********** Include Files **********/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include  "platform.h"
#include  "xil_printf.h"
#include  "xparameters.h"
#include  "xstatus.h"
#include  "xiic_l.h"
#include  "xiic.h"
#include  "xtmrctr.h"
#include  "xintc.h"
#include  "nexys4IO.h"
#include  "xiic.h"
#include  "xuartlite_l.h"
#include  "xuartlite.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include  "shared_resources.h"
#include  "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "xil_exception.h"
#include "mpu6050.h"

/********** DEBUG OUTPUT FLAG **********/
#define _DEBUG  1		// uncomment to enable debug output

/*********** Peripheral-related constants **********/
// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

#define INTC_DEVICE_ID		XPAR_INTC_0_DEVICE_ID

// Definitions for peripheral NEXYS4IO
#define N4IO_DEVICE_ID		    XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR		    XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR		    XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR


/*********Definitions for peripheral AXI_INTC************/
#define INTC_ID 		XPAR_INTC_0_DEVICE_ID
#define INTC_BADDR		XPAR_INTC_0_BASEADDR
#define INTC_HADDR		XPAR_INTC_0_HIGHADDR

/*********Definitions for peripheral AXI_IIC************/
#define IIC_ID 			XPAR_IIC_0_DEVICE_ID
#define IIC_BADDR				XPAR_IIC_0_BASEADDR
#define IIC_HADDR				XPAR_IIC_0_HIGHADDR

#define MPU_SENSOR_ADDRESS	0x68
#define WHO_AM_I_REG 0x75
#define EXPECTED_WHO_AM_I 0x68

#define GYRO_CONFIG_REGISTER 0x1B
#define GYRO_FULL_SCALE_250_DEG 0x00

/*********** Application-specific constants **********/
#define NBTNS 			5		// number of buttons

/********** AXI Peripheral Instances **********/
XIntc 		INTC_Inst;		// Interrupt Controller instance
XIic 		IICInst;		//Th instance of IIC


/********** Global Variables **********/
// These are volatile because they are generated in the FIT handler which is asynchronous
// to the program. We want to make sure the current values of the variables are returned
volatile bool newbtnsSw = false; // true if the FIT handler updated global buttons and switch values
volatile uint16_t sw = 0;	// switches - set in the FIT handler
volatile uint8_t btns = 0;	// buttons - set in the FIT handler

TaskHandle_t menuTaskHandle = NULL;
TaskHandle_t dataTaskHandle = NULL;
TaskHandle_t pidTaskHandle = NULL;
TaskHandle_t exitTaskHandle = NULL;

/********** Function Prototypes **********/

void MenuTask (void *pvParameters);
void DataTask(void *pvParameters);
void initializeApplication();
void PIDTask(void *pvParameters);
void ExitTask(void *pvParameters);
int MpuSensorExample(u16 iicid, u8 MpuSensorAddress, u8 *Mpu6050Ptr);
int check_mpu6050_i2c(void);

int InitializeIIC(XIic *IicInstancePtr, u16 iicid);
void ConfigureGyroscope(XIic *IICInst);
void mpu6050_getGyroData(XIic *IICInst, MPU6050_t *DataStruct);


/*
 * The following structure contains fields that are used with the callbacks
 * (handlers) of the IIC driver. The driver asynchronously calls handlers
 * when abnormal events occur or when data has been sent or received. This
 * structure must be volatile to work when the code is optimized.
 */
volatile struct {
	int  EventStatus;
	int  RemainingRecvBytes;
	int EventStatusUpdated;
	int RecvBytesUpdated;
} HandlerInfo;

// Define a semaphore for synchronizing access to 'a'

SemaphoreHandle_t dataSemaphore;

// initialization functions
int	 do_init(void);
u32 Checks_Button(u32 btns);


// other functions
void nexys4io_selfTest(void);

/********** Main Program **********/
int main()
{
	int Status;

	// Announce that the application has started
	xil_printf("ECE 544 PROJECT 2 \r\n");
	xil_printf("By Raksha Mairpady (email:raksham@pdx.edu) and Ruthuja Muttha (email:rmuttha@pdx.edu) \r\n\n");

	init_platform();
	uint32_t sts = do_init();
	if (XST_SUCCESS != sts){
		xil_printf("FATAL(main): System initialization failed\r\n");
		return 1;
	}
	Status = InitializeIIC(&IICInst, IIC_ID);
	    if (Status != XST_SUCCESS) {
	        xil_printf("IIC initialization failed!\r\n");
	        return XST_FAILURE;
	    }

	    /* The IIC device is now initialized and ready to use. */
	    /* You can now use IIC send and receive functions here */
	    if (check_mpu6050_i2c() != XST_SUCCESS) {
	    		xil_printf("SENSOR initialization failed\r\n");
	    		return 1;
	    }

	
	// perform the self test
	nexys4io_selfTest();

ConfigureGyroscope(&IICInst);


	initializeApplication();
	// main loop
	xTaskCreate(MenuTask, "Menu Task", 1000, (void*)0, tskIDLE_PRIORITY, &menuTaskHandle );

	if (menuTaskHandle == NULL) {
	    xil_printf("Failed to create Menu Task\r\n");
	}

	xTaskCreate(DataTask, "Data Task", 1000, (void*)0, tskIDLE_PRIORITY, &dataTaskHandle);

	if (dataTaskHandle == NULL) {
	    xil_printf("Failed to create Data Task\r\n");
	}
    xTaskCreate(PIDTask, "PID Task", 400, (void*)0, tskIDLE_PRIORITY, &pidTaskHandle);

	if (pidTaskHandle == NULL) {
	    xil_printf("Failed to create PID  Task\r\n");
	}


	xTaskCreate(ExitTask, "Exit Task", 200, (void*)0, tskIDLE_PRIORITY, &exitTaskHandle);
		if (pidTaskHandle == NULL) {
			xil_printf("Failed to create Exit Task\r\n");
			}
	// Start FreeRTOS scheduler
    vTaskStartScheduler();
	 // say goodbye and exit 	`
    xil_printf("ECE 544 Nexys4IO Test Program...ending\r\n");
    cleanup_platform();
    return 0;

}

void initializeApplication() {
    // Create the semaphore
    dataSemaphore = xSemaphoreCreateBinary();

    // Check for initialization failure
    if (dataSemaphore == NULL) {
        // Handle semaphore creation failure
        // For example, print an error message and halt the system
        printf("Error: Failed to create semaphore\n");
        vTaskSuspendAll(); // Suspend all tasks
        for (;;) {} // Infinite loop to halt the system
    }

    // Other initialization steps...
}
//menu task
void MenuTask(void *pvParameters) {
    char userInput;
    int targetAngle = 0; // Initialize targetAngle

    while (1) {
        xil_printf("Hello, I am Raksha Mairpady here \r\n");
        // Prompt the user for input
        xil_printf("Enter R or r for run mode and S or s for sleep mode: ");

        // Wait until a valid byte is received
        while (1) {
            while (!XUartLite_RecvByte(XPAR_AXI_UARTLITE_0_BASEADDR)); // Wait until a byte is received
            userInput = XUartLite_RecvByte(XPAR_AXI_UARTLITE_0_BASEADDR);
            xil_printf("%c\r\n", userInput);

            // Check if the input is valid
            if (userInput == 'r' || userInput == 'R' || userInput == 's' || userInput == 'S') {
                break; // Break the loop if the input is valid
            } else {
                xil_printf("Invalid character. Please enter R or r for run mode, or S or s for sleep mode.\r\n");
            }
        }

        // If the input is 'r' or 'R', prompt the user to enter the angle
        if (userInput == 'r' || userInput == 'R') {
            // Prompt the user to enter the angle
            xil_printf("Enter the target angle: ");

            // Read digits until a non-digit character is encountered
            targetAngle = 0;
            int digitCount = 0;
            char digitInput;
            while (1) {
                while (!XUartLite_RecvByte(XPAR_AXI_UARTLITE_0_BASEADDR)); // Wait until a byte is received
                digitInput = XUartLite_RecvByte(XPAR_AXI_UARTLITE_0_BASEADDR);
                xil_printf("%c", digitInput);

                // Check if the input is a digit
                if (isdigit(digitInput)) {
                    targetAngle = targetAngle * 10 + (digitInput - '0');
                    digitCount++;
                } else {
                    // Exit loop if non-digit character is encountered or after 4 digits
                    break;
                }
                // Limit input to maximum 4 digits
                if (digitCount >= 4) {
                    break;
                }
            }
            xil_printf("\r\nTarget angle set to: %d\r\n", targetAngle);
            // Give the semaphore to signal the DataTask to execute
            xSemaphoreGive(dataSemaphore);
            // Resume the DataTask
             vTaskResume(dataTaskHandle);

             // Resume the PIDTask
             vTaskResume(pidTaskHandle);

             // Resume the ExitTask
             vTaskResume(exitTaskHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void DataTask(void *pvParameters) {
	MPU6050_t gyroData;
    while(1) {
        // Obtain the semaphore
        xSemaphoreTake(dataSemaphore, portMAX_DELAY);

        mpu6050_getGyroData(&IICInst, &gyroData);

        xil_printf("the value of Gyrodata is %d\r\n", gyroData);

        // Release the semaphore
        xSemaphoreGive(dataSemaphore);

        // Implement angle computation and any necessary processing

        // Add a small delay to avoid busy-waiting
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//pid task
void PIDTask(void *pvParameters) {
    u32 btns;
    u32 check;
    while (1) {
        // Wait until the semaphore is available
        xSemaphoreTake(dataSemaphore, portMAX_DELAY);

        // Implement PID control or button/switch processing here
        btns = NX4IO_getBtns();
        check = Checks_Button(btns);


        // Release the semaphore
        xSemaphoreGive(dataSemaphore);

        // Add a small delay to avoid busy-waiting
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//exit task
void ExitTask(void *pvParameters) {
    u32 switches, prev_switches = 0xffff;

    while (1) {
        switches = NX4IO_getSwitches();
        if (prev_switches != switches && switches != 0) {
            xil_printf("Switches value is 0x%04X\n", switches);
            prev_switches = switches;

            // Suspend data, PID, and exit tasks
            vTaskSuspend(dataTaskHandle);
            vTaskSuspend(pidTaskHandle);
            vTaskSuspend(exitTaskHandle); // Suspend the exit task itself

            // Resume menu task
            vTaskResume(menuTaskHandle);
        }

        // Add a small delay to avoid busy-waiting
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


int	 do_init(void) {
	uint32_t status;				// status from Xilinx Lib calls


	// initialize the Nexys4 driver
	status = NX4IO_initialize(N4IO_BASEADDR);
	if (status != XST_SUCCESS){
		return XST_FAILURE;
	}
	/* xil_printf("NX4IO Initialization completed!");
	status = XIic_Initialize(&IICInst, IIC_ID);
	if (status != XST_SUCCESS){
		return XST_FAILURE;

	}*/
      return XST_SUCCESS;
	//XIic_Reset(&IICInst);

}


//checking switches and buttons
u32 Checks_Button(u32 btns)
{
	static bool isInitialized = false;	// true if the function has run at least once
	static u32 prevBtns;	// previous value of button register



	// initialize the static variables the first time the function is called
	if (!isInitialized) {
		prevBtns = btns ^ 0x1F;	// invert btns to get everything started

		isInitialized = true;
	}


	if ((prevBtns^ btns) != 0) {
		u32 btnMsk = 0x00000001;
		for(int i=0 ; i < NBTNS ; i++)
		{
			if(btns & (btnMsk << i)){
				switch(i){
					// iterate through the buttons and modify global PWM duty cycle variables
					case 0:	// btnC - disable/enable PWM
							xil_printf("hello 1\r\n");
							break;

					case 1:	// btnU - Green duty cycle
							xil_printf("hello 2\r\n");
							break;

					case 2: // btnL - Set all duty cycles to 0
							xil_printf("hello 3");
							break;
					case 3:	// btnR = Blue duty cycle
							xil_printf("hello 4");
							break;

					case 4:	// btnD - duty cycle
							xil_printf("hello 5");
							break;

					default: // shouldn't get here
							break;
	                   }
                }
		}
			prevBtns=btns;
	}
	return btns;
	}


/****************************************************************************/
/**
 * nexys4io_selfTest() - performs a self test on the NexysA7 peripheral
 *
 * @brief This is mostly a visual test to confirm that LEDs hardware and drivers are operating correctly. 
 *The test does the following:
 *	o sends pattern(s) to the LEDs on the board
 */
 void nexys4io_selfTest(void) {
	xil_printf("Starting Nexys4IO self test...\r\n");

	xil_printf("\tcheck functionality of LEDs\r\n");
	uint16_t ledvalue = 0x0001;
	do {
		NX4IO_setLEDs(ledvalue);
		usleep(250 * 1000);
		ledvalue = ledvalue << 1;
	} while (ledvalue != 0);

	NX4IO_setLEDs(0x0000);
	xil_printf("...Nexys4IO self test complete\r\n");
	return;
 }


 int InitializeIIC(XIic *IicInstancePtr, u16 iicid) {
     int Status;
     XIic_Config *ConfigPtr; /* Pointer to configuration data */

     /* Initialize the IIC driver so that it's ready to use */
     /* Look up the configuration in the config table, then initialize it. */
     ConfigPtr = XIic_LookupConfig(iicid);
     if (ConfigPtr == NULL) {
         xil_printf("No config found for %d\r\n", IIC_ID);
         return XST_FAILURE;
     }

     Status = XIic_CfgInitialize(IicInstancePtr, ConfigPtr, ConfigPtr->BaseAddress);
     if (Status != XST_SUCCESS) {
         xil_printf("Initialization failed %d\r\n", Status);
         return XST_FAILURE;
     }

     /* Start the IIC driver so that the device is enabled. */
     XIic_Start(IicInstancePtr);

     /* Set the IIC serial address to communicate with the slave device */
     XIic_SetAddress(IicInstancePtr, XII_ADDR_TO_SEND_TYPE, MPU_SENSOR_ADDRESS);

     xil_printf("IIC Initialized Successfully\r\n");

     return XST_SUCCESS;
 }

 int check_mpu6050_i2c(void) {
     u8 reg_addr = WHO_AM_I_REG;
     u8 who_am_i = 0;
     XIic_Send(IIC_BADDR, MPU_SENSOR_ADDRESS, &reg_addr, 1, XIIC_REPEATED_START); // Send register address
     XIic_Recv(IIC_BADDR, MPU_SENSOR_ADDRESS, &who_am_i, 1, XIIC_STOP); // Read from the register
     xil_printf("WHO_AM_I is %d\n\r", who_am_i);
     if (who_am_i == EXPECTED_WHO_AM_I) {
         xil_printf("MPU6050 I2C communication OK.\n\r");
         return XST_SUCCESS;
     } else {
         xil_printf("MPU6050 I2C communication FAILED no 1.\n\r");
         return XST_FAILURE;
     }
 }



 void ConfigureGyroscope(XIic *IICInst) {
     u8 writeBuffer[2];
     writeBuffer[0] = GYRO_CONFIG_REGISTER; // Register address
     writeBuffer[1] = GYRO_FULL_SCALE_250_DEG; // Configuration value

     XIic_Send(IIC_BADDR, MPU_SENSOR_ADDRESS, writeBuffer, 2, XIIC_STOP);
     xil_printf("Send Successfully!!!!!!");
 }

 void mpu6050_getGyroData(XIic *IICInst, MPU6050_t *DataStruct) {
	 MPU6050_t gyroData;
	 MPU6050_config_t mpu6050_config;
     u8 regAddr = GYRO_XOUT_H; // Starting register address
     u8 Rx_Data[6]; // Buffer to hold received data

     // Send the register address to read from
     XIic_Send(IIC_BADDR, MPU_SENSOR_ADDRESS, &regAddr, 1, XIIC_REPEATED_START);
     // Receive 6 bytes of data starting from GYRO_XOUT_H
     XIic_Recv(IIC_BADDR, MPU_SENSOR_ADDRESS, Rx_Data, 6, XIIC_STOP);

     // Convert received data
     DataStruct->Gyro_X_RAW = (Rx_Data[0] << 8) | Rx_Data[1];
     DataStruct->Gyro_Y_RAW = (Rx_Data[2] << 8) | Rx_Data[3];
     DataStruct->Gyro_Z_RAW = (Rx_Data[4] << 8) | Rx_Data[5];

     // Assuming mpu6050_config.gyro_sensitivity is previously defined and set correctly
     DataStruct->Gx = DataStruct->Gyro_X_RAW / mpu6050_config.gyro_sensitivity;
     DataStruct->Gy = DataStruct->Gyro_Y_RAW / mpu6050_config.gyro_sensitivity;
     DataStruct->Gz = DataStruct->Gyro_Z_RAW / mpu6050_config.gyro_sensitivity;
 }

