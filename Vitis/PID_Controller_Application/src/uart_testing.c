/**
* @file PID_Project.c
*
* @author Raksha Mairpady and Ruthuja Mutta
*
* @brief 
*
******************************************************************************/
/*********** Include Files **********/
#include <stdbool.h>
#include  "platform.h"
#include  "xil_printf.h"
#include  "xparameters.h"
#include  "xstatus.h"
#include  "xiic.h"
#include  "nexys4IO.h"
#include  "xuartlite_l.h"
#include  "xuartlite.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include  "shared_resources.h"
#include "task.h"
#include "mpu6050.h"
#include "microblaze_sleep.h"

/*********** Peripheral-related constants **********/
// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// Definitions for peripheral NEXYS4IO
#define N4IO_DEVICE_ID		    XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR		    XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR		    XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

/*********Definitions for peripheral AXI_IIC************/
#define IIC_ID 			XPAR_IIC_0_DEVICE_ID
#define IIC_BADDR		XPAR_IIC_0_BASEADDR

/*********** Application-specific constants **********/
#define NBTNS 			5		// number of buttons

/****************************** Task Handle *********************
static TaskHandle_t menuTaskHandle = NULL;
static TaskHandle_t dataTaskHandle = NULL;
static TaskHandle_t pidTaskHandle = NULL;
static TaskHandle_t exitTaskHandle = NULL;
 taskParameters_t taskParam;

/********** AXI Peripheral Instances **********/
XIic 		IICInst;		//Th instance of IIC
// bool gyroConfigured = false;

/********** Function Prototypes **********/
int	 do_init(void);
int InitializeIIC(XIic *IicInstancePtr, u16 iicid);
u32 Checks_Button(u32 btns);

/************************ TASK Declarations*************************/
//void MenuTask(void *pvParameters);
//void DataTask(void *pvParameters);
//void PIDTask(void *pvParameters);
//void ExitTask(void *pvParameters);


/************************* MAIN Program **************************/
int main()
{
	int Status;
	//taskParam.mode = RUN;
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

	 mpu6050_init(&IICInst);
	 usleep(50000);
	while(1) {
	         gettempData(&IICInst);
	     }
	return 0;
}

int	 do_init(void) {
	uint32_t status;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver
	status = NX4IO_initialize(N4IO_BASEADDR);
	if (status != XST_SUCCESS){
		return XST_FAILURE;
	}
      return XST_SUCCESS;
}

 int InitializeIIC(XIic *IicInstancePtr, u16 iicid) {

     xil_printf("IIC Initialized Successfully\r\n");

     return XST_SUCCESS;
 }



