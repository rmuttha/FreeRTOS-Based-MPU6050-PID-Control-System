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
#include  "FreeRTOSConfig.h"
//#include "task.h"
//#include "queue.h"

#include "FreeRTOS.h"


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
#define IIC_DEVICE_ID 			XPAR_IIC_0_DEVICE_ID
#define IIC_BADDR				XPAR_IIC_0_BASEADDR
#define IIC_HADDR				XPAR_IIC_0_HIGHADDR


/*********** Application-specific constants **********/
#define NBTNS 			4		// number of buttons

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


/********** Function Prototypes **********/
//int IicSelfTestExample( IIC_DEVICE_ID);

// initialization functions
int	 do_init(void);


// other functions
void nexys4io_selfTest(void);


/********** Main Program **********/
int main()
{
	u32 btns;

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
	//IicSelfTestExample( IIC_DEVICE_ID);
	// main loop

	while (1) {
		
		if (newbtnsSw) {
			// write the switches to the LEDs
			NX4IO_setLEDs(sw);
			xil_printf("New switches: 0x%04X\tNew buttons: 0x%02X\n\r", sw, btns);

			// write the buttons to the 7-segment display decimal points
			uint8_t btnMsk = 0x01;
			bool btnState = false;
			for (int i = 0; i < NBTNS; i++) {  //{BTND(BTN3), BTNL(BTN2), BTNR(BTN1), BTNU(BTN0)}
				// look at the buttons one at a time to see if the button is pressed.
				// If so, light the decimal point corresponding to the button
				btnState = btns & (btnMsk << i) ? true : false;
				switch (i) {
					 case 4:	// BTNc(BTN4)
				    //NX4IO_SSEG_setDecPt(SSEGHI, DIGIT7, btnState); 
						break;

					case 3:	// BTND(BTN3)
						NX4IO_SSEG_setDecPt(SSEGHI, DIGIT7, btnState);
						break;

					case 2:    //BTNL(BTN2)
					//X4IO_SSEG_setDecPt(SSEGHI, DIGIT6, btnState);
					break;

					case 1://BTNR(BTN1)
					//X4IO_SSEG_setDecPt(SSEGHI, DIGIT5, btnState);
			 		break;

					case 0:	// BTNU(BTN0)
						NX4IO_SSEG_setDecPt(SSEGHI, DIGIT4, btnState);
						break;

					default: // shouldn't get here
						break;
				}
			}
			newbtnsSw = false;
		}

	}   // main loop
	 // say goodbye and exit 	`
	microblaze_disable_interrupts();
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
    xil_printf("ECE 544 Nexys4IO Test Program...ending\r\n");
    cleanup_platform();
    return 0;
}





int	 do_init(void) {
	uint32_t status;				// status from Xilinx Lib calls

	
	// initialize the Nexys4 driver
	status = NX4IO_initialize(N4IO_BASEADDR);
	if (status != XST_SUCCESS){
		return XST_FAILURE;
	}

	/* // initialize the interrupt controller
	status = XIntc_Initialize(&INTC_Inst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}

	// connect the interrupt handlers to the interrupts
	status = XIntc_Connect(&INTC_Inst, FIT_INTR_NUM,
						   (XInterruptHandler)FIT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}*/

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&INTC_Inst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable/disable the interrupts
	//XIntc_Enable(&INTC_Inst, FIT_INTR_NUM);
	//return XST_SUCCESS;
	
	//initialise the IIC controler
	status= XIic_Initialize(&IICInst,IIC_DEVICE_ID);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}
	 
	/* status = IicSelfTestExample(IIC_DEVICE_ID);
	  if (status != XST_SUCCESS) {
		xil_printf("IIC selftest Example Failed\r\n");
		return XST_FAILURE;
	    }

	  xil_printf("Successfully ran IIC selftest Example\r\n");
	  return XST_SUCCESS;
	  
	  XIic_Reset(&IICInst);*/
		
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


