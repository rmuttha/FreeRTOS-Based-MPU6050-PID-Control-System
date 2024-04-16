////////////
// Top-level module for ECE 544 Project #2
// Authors:Raksha Mairpady and Rutuja Mutta
//Description: defines the input and output ports of the block design
///////////
`timescale 1 ps / 1 ps

module nexysa7fpga
   (
	input logic			clk,
	input logic [15:0]	sw,
	input logic		    btnU,
	input logic		    btnR,
	input logic			btnL,
	input logic			btnD,
	input logic			btnC,			
	input logic			btnCpuReset,
	output logic [15:0]	led,
    output logic RGB1_Blue, RGB1_Green, RGB1_Red,
	output logic RGB2_Blue,RGB2_Green, RGB2_Red,	
    output logic [7:0]	an,
	output logic [6:0]	seg,
    output logic		dp,
	input logic usb_uart_rxd,
	output logic usb_uart_txd,
	inout logic sclk_io,
        inout logic sda_io,
        output logic GPIO
);
    
 // wire [31:0] control_reg, gpio_rtl_tri_o;
  
  // wrap the gpio output to the rgbPWM control register
  //assign control_reg = gpio_rtl_tri_o;

  // instantiate the embedded system
  embsys embsys_i
       (//.gpio_rtl_0_tri_o(gpio_rtl_tri_o),
        .RGB2_Blue_0(RGB2_Blue),
        .RGB2_Green_0(RGB2_Green),
        .RGB2_Red_0(RGB2_Red),
        .an_0(an),
        .btnC_0(btnC),
        .btnD_0(btnD),
        .btnL_0(btnL),
        .btnR_0(btnR),
        .btnU_0(btnU),
        //.clkPWM_0(),
        .sys_clock(clk),
        //.controlReg_0(control_reg),
        .dp_0(dp),
        .led_0(led),
        .resetn(btnCpuReset),
        .RGB1_Blue_0(RGB1_Blue),
        .RGB1_Green_0(RGB1_Green),
        .RGB1_Red_0(RGB1_Red),
        .seg_0(seg),
        .sw_0(sw),
        .sclk_io(sclk_io),
	    .sda_io(sda_io),
	    .usb_uart_rxd(usb_uart_rxd),
        .usb_uart_txd(usb_uart_txd),
        .GPIO_tri_o(GPIO));
endmodule
