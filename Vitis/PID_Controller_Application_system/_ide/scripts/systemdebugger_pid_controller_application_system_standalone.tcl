# Usage with Vitis IDE:
# In Vitis IDE create a Single Application Debug launch configuration,
# change the debug type to 'Attach to running target' and provide this 
# tcl script in 'Execute Script' option.
# Path of this script: D:\ECE544\PID_CONTROL_PROJ1\PID_Controller_Application_system\_ide\scripts\systemdebugger_pid_controller_application_system_standalone.tcl
# 
# 
# Usage with xsct:
# To debug using xsct, launch xsct and run below command
# source D:\ECE544\PID_CONTROL_PROJ1\PID_Controller_Application_system\_ide\scripts\systemdebugger_pid_controller_application_system_standalone.tcl
# 
connect -url tcp:127.0.0.1:3121
targets -set -filter {jtag_cable_name =~ "Digilent Nexys4DDR 210292A8AEAAA" && level==0 && jtag_device_ctx=="jsn-Nexys4DDR-210292A8AEAAA-13631093-0"}
fpga -file D:/ECE544/PID_CONTROL_PROJ1/PID_Controller_Application/_ide/bitstream/download.bit
targets -set -nocase -filter {name =~ "*microblaze*#0" && bscan=="USER2" }
loadhw -hw D:/ECE544/PID_CONTROL_PROJ1/PID_Controller_Platform/export/PID_Controller_Platform/hw/nexysa7fpga.xsa -regs
configparams mdm-detect-bscan-mask 2
targets -set -nocase -filter {name =~ "*microblaze*#0" && bscan=="USER2" }
rst -system
after 3000
targets -set -nocase -filter {name =~ "*microblaze*#0" && bscan=="USER2" }
dow D:/ECE544/PID_CONTROL_PROJ1/PID_Controller_Application/Debug/PID_Controller_Application.elf
targets -set -nocase -filter {name =~ "*microblaze*#0" && bscan=="USER2" }
con
