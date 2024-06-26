//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2022.2 (win64) Build 3671981 Fri Oct 14 05:00:03 MDT 2022
//Date        : Sun Mar 10 23:26:57 2024
//Host        : WINDOWS-V51576L running 64-bit major release  (build 9200)
//Command     : generate_target embsys_wrapper.bd
//Design      : embsys_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module embsys_wrapper
   (GPIO_tri_o,
    RGB1_Blue_0,
    RGB1_Green_0,
    RGB1_Red_0,
    RGB2_Blue_0,
    RGB2_Green_0,
    RGB2_Red_0,
    an_0,
    btnC_0,
    btnD_0,
    btnL_0,
    btnR_0,
    btnU_0,
    dp_0,
    led_0,
    resetn,
    sclk_io,
    sda_io,
    seg_0,
    sw_0,
    sys_clock,
    usb_uart_rxd,
    usb_uart_txd);
  output [0:0]GPIO_tri_o;
  output RGB1_Blue_0;
  output RGB1_Green_0;
  output RGB1_Red_0;
  output RGB2_Blue_0;
  output RGB2_Green_0;
  output RGB2_Red_0;
  output [7:0]an_0;
  input btnC_0;
  input btnD_0;
  input btnL_0;
  input btnR_0;
  input btnU_0;
  output dp_0;
  output [15:0]led_0;
  input resetn;
  inout sclk_io;
  inout sda_io;
  output [6:0]seg_0;
  input [15:0]sw_0;
  input sys_clock;
  input usb_uart_rxd;
  output usb_uart_txd;

  wire [0:0]GPIO_tri_o;
  wire RGB1_Blue_0;
  wire RGB1_Green_0;
  wire RGB1_Red_0;
  wire RGB2_Blue_0;
  wire RGB2_Green_0;
  wire RGB2_Red_0;
  wire [7:0]an_0;
  wire btnC_0;
  wire btnD_0;
  wire btnL_0;
  wire btnR_0;
  wire btnU_0;
  wire dp_0;
  wire [15:0]led_0;
  wire resetn;
  wire sclk_io;
  wire sda_io;
  wire [6:0]seg_0;
  wire [15:0]sw_0;
  wire sys_clock;
  wire usb_uart_rxd;
  wire usb_uart_txd;

  embsys embsys_i
       (.GPIO_tri_o(GPIO_tri_o),
        .RGB1_Blue_0(RGB1_Blue_0),
        .RGB1_Green_0(RGB1_Green_0),
        .RGB1_Red_0(RGB1_Red_0),
        .RGB2_Blue_0(RGB2_Blue_0),
        .RGB2_Green_0(RGB2_Green_0),
        .RGB2_Red_0(RGB2_Red_0),
        .an_0(an_0),
        .btnC_0(btnC_0),
        .btnD_0(btnD_0),
        .btnL_0(btnL_0),
        .btnR_0(btnR_0),
        .btnU_0(btnU_0),
        .dp_0(dp_0),
        .led_0(led_0),
        .resetn(resetn),
        .sclk_io(sclk_io),
        .sda_io(sda_io),
        .seg_0(seg_0),
        .sw_0(sw_0),
        .sys_clock(sys_clock),
        .usb_uart_rxd(usb_uart_rxd),
        .usb_uart_txd(usb_uart_txd));
endmodule
