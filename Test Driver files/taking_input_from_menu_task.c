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

/*********Definitions for peripheral AXI_IIC************/
#define IIC_ID 			XPAR_IIC_0_DEVICE_ID
#define IIC_BADDR				XPAR_IIC_0_BASEADDR
#define IIC_HADDR				XPAR_IIC_0_HIGHADDR


/*********** Application-specific constants **********/
#define NBTNS 			5		// number of buttons

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
//TaskHandle_t dataTaskHandle = NULL;
//TaskHandle_t pidTaskHandle = NULL;
//TaskHandle_t exitTaskHandle = NULL;

//xSemaphoreHandle angleUpdateSemaphore;


/********** Function Prototypes **********/

void MenuTask (void *pvParameters);
//void DataTask(void *pvParameters);
//void PIDTask(void *pvParameters);
//void ExitTask(void *pvParameters);
// Define a semaphore for synchronizing access to 'a'
//xSemaphoreHandle dataSemaphore;

// Inside your initialization code or setup function, create the semaphore
//dataSemaphore = xSemaphoreCreateBinary();

// initialization functions
int	 do_init(void);
//u32 Checks_Button(u32 btns);


// other functions
void nexys4io_selfTest(void);


/********** Main Program **********/
int main()
{
	//u32 btns;
	//u32 switches,prev_switches =0xffff;
	//u32 check;
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
	// main loop
	xTaskCreate(MenuTask, "Menu Task", 1000, (void*)0, tskIDLE_PRIORITY, &menuTaskHandle );
	//xTaskCreate(DataTask, "Data Task", 1000, (void*)0, tskIDLE_PRIORITY, &dataTaskHandle);
   // xTaskCreate(PIDTask, "PID Task", 400, (void*)0, tskIDLE_PRIORITY, &pidTaskHandle);
   // xTaskCreate(ExitTask, "Exit Task", 200, (void*)0, tskIDLE_PRIORITY, &exitTaskHandle);

	// Start FreeRTOS scheduler
    vTaskStartScheduler();


    while(1){

    }
	 // say goodbye and exit 	`
    xil_printf("ECE 544 Nexys4IO Test Program...ending\r\n");
    cleanup_platform();
    return 0;

}

//menu task
void MenuTask (void *pvParameters){
	//char runMode;
  //  int targetAngle;
	 int userInput;
int count =0;
	while(1)
	{
		xil_printf("Hello word!%d \r\n",count++);
		// Prompt the user for input
		        xil_printf("Enter a number: ");

		        while (!XUartLite_RecvByte(XPAR_AXI_UARTLITE_0_BASEADDR)); // Wait until a byte is received
		                userInput = XUartLite_RecvByte(XPAR_AXI_UARTLITE_0_BASEADDR);

		                // Echo the received character back to the user
		                xil_printf("%c\r\n", userInput);

		                vTaskDelay(pdMS_TO_TICKS(1000));
     }

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

	//XIic_Reset(&IICInst);

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


