# ECE 544: FreeRTOS-Based MPU6050 PID Control System

## Overview

This project implements a closed-loop PID control system using FreeRTOS on a Microblaze-based platform to accurately measure and control the roll angle of an MPU-6050 gyroscope module. The system interfaces with the MPU-6050 via I2C, capturing gyroscopic data for real-time angle control and optimization.

## Background
### Microblaze-based FPGA System:
- Microblaze: This is a soft processor core designed by Xilinx for its range of FPGA (Field-Programmable Gate Array) boards.
  A soft processor means that the processor is implemented using the FPGA logic elements, which allows for high customization in terms of the processor's capabilities and the connected peripherals.
- FPGA: An FPGA is a type of digital logic chip that can be programmed to perform a wide variety of digital computing tasks.
  It is "field-programmable," which means it can be configured by the user after manufacturing.

### FreeRTOS:
- RTOS Basics: A Real-Time Operating System is used in environments where the timing of processor operations is crucial. RTOS systems are designed to manage the hardware resources of a computer and the hosting applications, ensuring timely and deterministic responses to external events.
- FreeRTOS: It is an open-source, miniaturized RTOS designed for embedded devices. It allows developers to prioritize tasks effectively and handle concurrency, which is particularly useful in embedded systems where multiple operations need to occur simultaneously without causing delays in critical processing tasks.

###  PID Control System:
- PID Controller: Stands for Proportional, Integral, Derivative controller. It is a type of feedback control loop mechanism widely used in industrial control systems. A PID controller calculates an "error" value as the difference between a measured process variable and a desired setpoint. The controller attempts to minimize this error by adjusting process control inputs.
- Application in the Project: In this project, the PID controller is used to manage the position of a gyroscope, attempting to achieve a specific orientation or 'roll' based on the user inputs. The control system adjusts in real-time to match the target roll angle with the angle reported by the sensor.

### MPU-6050 Gyroscope/Accelerometer Sensor:
- Sensor Overview: The MPU-6050 sensor integrates a 3-axis gyroscope and a 3-axis accelerometer on a single chip. It communicates with the Microblaze processor via the I2C interface, providing it with real-time orientation data.
- Use in the Project: This sensor's data is crucial for the PID control system's functionality. The gyroscope provides the current orientation in terms of roll, pitch, and yaw, which is then used by the PID controller to compute how much adjustment is needed to reach the desired orientation.

## Learning Objectives

- Develop proficiency with PID control and I2C peripheral driver development.
- Implement tasks under FreeRTOS for data handling and control logic.
- Lay the groundwork for integration into larger scale projects.

## Hardware Requirements

- Digilent Nexys A7 (Nexys4 DDR) or RealDigital Boolean Board
- MPU-6050 Gyroscope/Accelerometer Sensor
- Necessary cables and connectors for interfacing

## Software Requirements

- Xilinx Vivado for FPGA programming
- FreeRTOS for task management
- Terminal software such as Putty or TeraTerm for debugging

## Setup and Configuration

1. **FPGA Configuration:**
   - Create a project in Vivado using the provided XDC constraints.
   - Configure the FPGA with Microblaze, AXI peripherals, and integrate the MPU-6050 over I2C.

2. **Sensor Setup:**
   - Solder the MPU-6050 sensor and connect it to the FPGA board:
     - `GND` to `GND`
     - `VCC` to `3.3V`
     - `SDA` to `JB[4]`
     - `SCL` to `JB[3]`

3. **Firmware Deployment:**
   - Implement the driver and tasks in FreeRTOS for initializing the sensor, acquiring data, processing via PID, and handling user input.
   - Adjust the PID parameters to tune the system performance based on the gyroscopic data.

## Usage

Run the system by powering up the FPGA board and loading the application via Vitis. Use the UART interface to interact with the system, setting parameters and switching between modes. Detailed instructions and command options are available in the terminal interface.

## Deliverables

- Source code for the MPU6050 driver and FreeRTOS tasks.
- Design reports and diagrams illustrating the system architecture and data flow.
- A video demo showcasing the working system.
- Live demo sessions as scheduled.

## Troubleshooting

Refer to the `FAQ` section for common issues and troubleshooting steps related to hardware configuration and software functionality.

## References

- [MPU-6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU-6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [FreeRTOS API](https://www.freertos.org/a00106.html)

## Contact

For further information or support, please contact: rmuttha@pdx.edu

