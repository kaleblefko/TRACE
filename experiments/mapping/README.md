# Package Setup:
if not performed already:
>sudo apt install ros-humble-rtabmap-ros

## Custom Drone Model Setup
>cd ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes

>cp 4002_gz_x500_depth 4022_gz_trace_drone

>nano 4022_gz_trace_drone

When nano opens change the following:
 
>@name Gazebo Trace Drone

PX4_SIM_MODEL=${PX4_SIM_MODEL:=trace_drone}
 
Then nano CMakeLists.txt, add into the arguments of px4_add_romfs_files 
>4022_gz_trace_drone 

Just scroll in the file and find 4021_gz_x500_flow and add it below.
 
After that,
 
>cd ~/PX4-Autopilot

>rm -rf build/

>make clean

>make px4_sitl
 
After px4 is done building run 
 
>cd ~/TRACE

>bash update_px4.sh
 
Then make px4_sitl gz_trace_drone should work.

# Running TRACE mapping experiment:
Running Mapping Pipeline with Teleop in VM

## Terminal 1
>cd ~/PX4-Autopilot

>PX4_GZ_WORLD=small_house make px4_sitl gz_trace_drone

if the drone is not ready for takeoff after these commands, set the following parameters:

>param set CBRK_SUPPLY_CHK 894248

>param set COM_ARM_WO_GPS 1

>param set NAV_RCL_ACT 0

>param set NAV_DLL_ACT 0

## Terminal 2
>MicroXRCEAgent udp4 -p 8888

## Terminal 3
>colcon build

>source install/setup.bash

>ros2 run trace_mapping mapping_node

## Terminal 4
>source install/setup.bash

>ros2 run trace_pipeline teleop

## Terminal 5
>rviz2

Then add topic -> occupancy grid map