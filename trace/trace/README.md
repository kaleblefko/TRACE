# Running TRACE Find-the-Blue-Ball Pipeline

## OLLAMA Setup:
On your computer outside the VM run<br>
export OLLAMA_HOST=0.0.0.0<br>
ollama serve<br>
To get the IP address do: ipconfig getifaddr en0 (MacOS)<br>
Must have a .env stored at ~/TRACE/.env<br>
See .env.example for an example .env file<br>
to stop ollama (address already bound error):
systemctl stop ollama

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
>cd ~/TRACE
>colcon build
>source install/setup.bash
>ros2 run trace mapping

## Terminal 4
>cd ~/TRACE
>source install/setup.bash
>ros2 run trace bbox

## Terminal 5
>cd ~/TRACE
>source install/setup.bash
>ros2 run trace visualizer

## Terminal 6
>cd ~/TRACE
>source install/setup.bash
>ros2 run trace nav

## Terminal 7
>rviz2
Then add topic -> occupancy grid map

## Terminal 8
>cd ~/TRACE
>source install/setup.bash
>ros2 run trace pipeline