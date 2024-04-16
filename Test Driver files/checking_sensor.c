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
//#include  "microblaze_sleep.h"
#include  "xiic_l.h"
#include  "xiic.h"
#include  "xtmrctr.h"
#include  "xintc.h"
#include  "nexys4IO.h"
//#include  "xiic_selftest_example.c"
#include  "xiic.h"
#include  "xuartlite_l.h"
#include  "xuartlite.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include  "shared_resources.h"
#include  "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

/********** DEBUG OUTPUT FLAG **********/
//#define _DEBUG  1		// uncomment to enable debug output

/*********** Peripheral-related constants **********/
// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ



// Definitions for peripheral NEXYS4IO
#define N4IO_DEVICE_ID		    XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR		    XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR		    XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR


/*********Definitions for peripheral AXI_INTC************/
#define INTC_ID 		XPAR_INTC_0_DEVICE_ID
#define INTC_BADDR		XPAR_INTC_0_BASEADDR
#define INTC_HADDR		XPAR_INTC_0_HIGHADDR

//#define INTC_DEVICE_ID		XPAR_INTC_0_DEVICE_ID
#define INTC_IIC_INTERRUPT_ID	XPAR_INTC_0_IIC_0_VEC_ID


/*********Definitions for peripheral AXI_IIC************/
#define IIC_ID 					XPAR_IIC_0_DEVICE_ID
#define IIC_BADDR				XPAR_IIC_0_BASEADDR
#define IIC_HADDR				XPAR_IIC_0_HIGHADDR


/*********** Application-specific constants **********/
#define NBTNS 			5		// number of buttons
#define MPU6050_ADDRESS			0x68

/********** AXI Peripheral Instances **********/
//XTmrCtr		N4IO_TimerInst;	// Timer instance for N4IO rgb clock input
XIntc 		INTC_Inst;		// Interrupt Controller instance
XIic 		IICInst;		//Th instance of IIC


/********** Global Variables **********/
// These are volatile because they are generated in the FIT handler which is asynchronous
// to the program. We want to make sure the current values of the variables are returned
volatile bool newbtnsSw = false; // true if the FIT handler updated global buttons and switch values
volatile uint16_t sw = 0;	// switches - set in the FIT handler
volatile uint8_t btns = 0;	// buttons - set in the FIT handler
//int a=2,b=3,c=4;

TaskHandle_t menuTaskHandle = NULL;
TaskHandle_t dataTaskHandle = NULL;
TaskHandle_t pidTaskHandle = NULL;
TaskHandle_t exitTaskHandle = NULL;

//xSemaphoreHandle angleUpdateSemaphore;


/********** Function Prototypes **********/

void MenuTask (void *pvParameters);
void DataTask(void *pvParameters);
void initializeApplication();
void PIDTask(void *pvParameters);
void ExitTask(void *pvParameters);

int MPU_Example(u16 IIC_ID, u8 MPU6050_ADDRESS,
						  u8 *RakshaPtr);
						  
static int SetupInterruptSystem(XIic *IicPtr);

static void RecvHandler(void *CallbackRef, int ByteCount);

static void StatusHandler(void *CallbackRef, int Status);


// Define a semaphore for synchronizing access to 'a'

SemaphoreHandle_t dataSemaphore;

// Inside your initialization code or setup function, create the semaphore
//dataSemaphore = xSemaphoreCreateBinary();

// initialization functions
int	 do_init(void);
u32 Checks_Button(u32 btns);


// other functions
void nexys4io_selfTest(void);

volatile struct {
	int  EventStatus;
	int  RemainingRecvBytes;
	int EventStatusUpdated;
	int RecvBytesUpdated;
} HandlerInfo;

/********** Main Program **********/
int main()
{
	u32 btns;
	u32 switches,prev_switches =0xffff;
	u32 check;
	int St;
	u8 RakshaPtr;
	// Announce that the application has started
	xil_printf("ECE 544 PROJECT 2 \r\n");
	xil_printf("By Raksha Mairpady (email:raksham@pdx.edu) and Ruthuja Muttha (email:rmuttha@pdx.edu) \r\n\n");

	init_platform();
	uint32_t sts = do_init();
	if (XST_SUCCESS != sts){
		xil_printf("FATAL(main): System initialization failed\r\n");
		return 1;
	}
	
	// perform the self test
		nexys4io_selfTest();
		
		St =  MPU_Example(IIC_ID, MPU6050_ADDRESS,
							&RakshaPtr);
	if (St != XST_SUCCESS) {
		xil_printf("IIC MPUExample Failed\r\n");
		return XST_FAILURE;
	}

	xil_printf("Successfully ran IIC tempsensor Example\r\n");
	return XST_SUCCESS;
		
		
		
		
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


    while(1){

    }
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
    int count = 0;
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
    int b, c, a; // Assume these variables are defined elsewhere

    while(1) {
        // Obtain the semaphore
        xSemaphoreTake(dataSemaphore, portMAX_DELAY);

        b = 10; // Example value for b
        c = 20; // Example value for c

        // Calculate the sum of b and c and store it in a
        a = b + c;
        xil_printf("the value of a is %d\r\n", a);

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
	status = XIic_Initialize(&IICInst, IIC_ID);
	if (status != XST_SUCCESS){
		return XST_FAILURE;
	}
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





int MPU_Example(u16 IIC_ID, u8 MPU6050_ADDRESS, u8 *RakshaPtr)
{
	int Risheek;
	static int Initialized = FALSE;
	XIic_Config *ConfigPtr;	/* Pointer to configuration data */

	if (!Initialized) {
		Initialized = TRUE;

		/*
		 * Initialize the IIC driver so that it is ready to use.
		 */
		ConfigPtr = XIic_LookupConfig(IIC_ID);
		if (ConfigPtr == NULL) {
			return XST_FAILURE;
		}

		Risheek = XIic_CfgInitialize(&IICInst, ConfigPtr,
						ConfigPtr->BaseAddress);
		if (Risheek != XST_SUCCESS) {
			return XST_FAILURE;
		}


		/*
		 * Setup handler to process the asynchronous events which occur,
		 * the driver is only interrupt driven such that this must be
		 * done prior to starting the device.
		 */
		XIic_SetRecvHandler(&IICInst, (void *)&HandlerInfo, RecvHandler);
		XIic_SetStatusHandler(&IICInst, (void *)&HandlerInfo,
						StatusHandler);

		/*
		 * Connect the ISR to the interrupt and enable interrupts.
		 */
		Risheek = SetupInterruptSystem(&IICInst);
		if (Risheek != XST_SUCCESS) {
			return XST_FAILURE;
		}

		/*
		 * Start the IIC driver such that it is ready to send and
		 * receive messages on the IIC interface, set the address
		 * to send to which is the temperature sensor address
		 */
		XIic_Start(&IICInst);
		XIic_SetAddress(&IICInst, XII_ADDR_TO_SEND_TYPE, MPU6050_ADDRESS);
	}

	/*
	 * Clear updated flags such that they can be polled to indicate
	 * when the handler information has changed asynchronously and
	 * initialize the status which will be returned to a default value
	 */
	HandlerInfo.EventStatusUpdated = FALSE;
	HandlerInfo.RecvBytesUpdated = FALSE;
	Risheek = XST_FAILURE;

	/*
	 * Attempt to receive a byte of data from the temperature sensor
	 * on the IIC interface, ignore the return value since this example is
	 * a single master system such that the IIC bus should not ever be busy
	 */
	(void)XIic_MasterRecv(&IICInst, RakshaPtr, 1);

	/*
	 * The message is being received from the temperature sensor,
	 * wait for it to complete by polling the information that is
	 * updated asynchronously by interrupt processing
	 */
	while(1) {
		if(HandlerInfo.RecvBytesUpdated == TRUE) {
			/*
			 * The device information has been updated for receive
			 * processing,if all bytes received (1), indicate
			 * success
			 */
			if (HandlerInfo.RemainingRecvBytes == 0) {
				Risheek = XST_SUCCESS;
			}
			break;
		}

		/*
		 * Any event status which occurs indicates there was an error,
		 * so return unsuccessful, for this example there should be no
		 * status events since there is a single master on the bus
		 */
		if (HandlerInfo.EventStatusUpdated == TRUE) {
			break;
		}
	}

	return Risheek;
}


/*****************************************************************************/
/**
* This receive handler is called asynchronously from an interrupt context and
* and indicates that data in the specified buffer was received. The byte count
* should equal the byte count of the buffer if all the buffer was filled.
*
* @param	CallBackRef is a pointer to the IIC device driver instance which
*		the handler is being called for.
* @param	ByteCount indicates the number of bytes remaining to be received of
*		the requested byte count. A value of zero indicates all requested
*		bytes were received.
*
* @return	None.
*
* @notes	None.
*
****************************************************************************/
static void RecvHandler(void *CallbackRef, int ByteCount)
{
	HandlerInfo.RemainingRecvBytes = ByteCount;
	HandlerInfo.RecvBytesUpdated = TRUE;
}

/*****************************************************************************/
/**
* This status handler is called asynchronously from an interrupt context and
* indicates that the conditions of the IIC bus changed. This  handler should
* not be called for the application unless an error occurs.
*
* @param	CallBackRef is a pointer to the IIC device driver instance which the
*		handler is being called for.
* @param	Status contains the status of the IIC bus which changed.
*
* @return	None.
*
* @notes	None.
*
****************************************************************************/
static void StatusHandler(void *CallbackRef, int Status)
{
	HandlerInfo.EventStatus |= Status;
	HandlerInfo.EventStatusUpdated = TRUE;
}



static int SetupInterruptSystem(XIic *IicPtr)
{
	int Rutuja;

	/*
	 * Initialize the interrupt controller driver so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h
	 */
	Rutuja = XIntc_Initialize(&InterruptController, INTC_DEVICE_ID);
	if (Rutuja != XST_SUCCESS) {
		return XST_FAILURE;
	}


	/*
	 * Connect a device driver handler that will be called when an interrupt
	 * for the device occurs, the device driver handler performs the
	 * specific interrupt processing for the device
	 */
	Rutuja = XIntc_Connect(&InterruptController, INTC_IIC_INTERRUPT_ID,
					XIic_InterruptHandler, IicPtr);
	if (Rutuja != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Start the interrupt controller such that interrupts are recognized
	 * and handled by the processor.
	 */
	Rutuja = XIntc_Start(&InterruptController, XIN_REAL_MODE);
	if (Rutuja != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Enable the interrupts for the IIC device.
	 */
	XIntc_Enable(&InterruptController, INTC_IIC_INTERRUPT_ID);

	/*
	 * Initialize the exception table.
	 */
	Xil_ExceptionInit();

	/*
	 * Register the interrupt controller handler with the exception table.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				 (Xil_ExceptionHandler) XIntc_InterruptHandler,
				 &InterruptController);

	/*
	 * Enable non-critical exceptions.
	 */
	Xil_ExceptionEnable();

	return XST_SUCCESS;

}

