## Setup:
On your computer outside the VM run<br>
export OLLAMA_HOST=0.0.0.0<br>
ollama serve<br>

To get the IP address do: ipconfig getifaddr en0 (MacOS)<br>

Must have a .env stored at ~/TRACE/.env<br>
See .env.example for an example .env file<br>

## Running

Terminal 1:

cd PX4-Autopilot
PX4_GZ_WORLD=colored_blocks_world make px4_sitl gz_x500_mono_cam_world

Terminal 2:
MicroXRCEAgent udp4 -p 8888

Terminal 3:
cd TRACE/experiments/px4_vlm_control
colcon build
source install/setup.bash
ros2 run vlm_control vlm_control