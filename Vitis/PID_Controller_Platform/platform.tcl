# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct D:\ECE544\PID_CONTROL_PROJ1\PID_Controller_Platform\platform.tcl
# 
# OR launch xsct and run below command.
# source D:\ECE544\PID_CONTROL_PROJ1\PID_Controller_Platform\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {PID_Controller_Platform}\
-hw {D:\ECE544\PID_CONTROL_PROJ1\TEST1_VIVADO\project_1\nexysa7fpga.xsa}\
-proc {microblaze_0} -os {freertos10_xilinx} -out {D:/ECE544/PID_CONTROL_PROJ1}

platform write
platform generate -domains 
platform active {PID_Controller_Platform}
bsp reload
bsp config support_static_allocation "false"
bsp config use_counting_semaphores "false"
bsp config use_freertos_asserts "true"
bsp config use_getmutex_holder "true"
bsp config use_mutexes "false"
bsp config use_recursive_mutexes "false"
bsp config use_stats_formatting_functions "true"
bsp config software_timers "false"
bsp write
bsp reload
catch {bsp regenerate}
bsp config use_getmutex_holder "true"
bsp config use_mutexes "false"
bsp config use_newlib_reent "false"
bsp config use_queue_sets "true"
bsp config use_stats_formatting_functions "true"
bsp config use_recursive_mutexes "false"
bsp write
platform generate
platform active {PID_Controller_Platform}
bsp reload
bsp config total_heap_size "65536"
bsp config total_heap_size "65536"
bsp config minimal_stack_size "200"
bsp config total_heap_size "8192"
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains freertos10_xilinx_domain 
bsp reload
bsp config total_heap_size "65536"
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains freertos10_xilinx_domain 
bsp reload
bsp config total_heap_size "8129"
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains freertos10_xilinx_domain 
bsp config total_heap_size "65536"
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains freertos10_xilinx_domain 
bsp config total_heap_size "8192"
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains freertos10_xilinx_domain 
bsp config max_priorities "5"
bsp config use_task_notifications "false"
bsp write
bsp reload
catch {bsp regenerate}
platform generate -domains freertos10_xilinx_domain 
platform generate -domains freertos10_xilinx_domain 
platform generate
