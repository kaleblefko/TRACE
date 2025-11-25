## Setup:
### On your computer outside the VM:<br>
export OLLAMA_HOST=0.0.0.0<br>
ollama serve<br>

To get the IP address do: ipconfig getifaddr en0 (MacOS)<br>

### On the VM:
Must have a .env stored at ~/TRACE/.env<br>
See .env.example for an example .env file<br>

run (the sudo apt installs will install the package for the whole VM):<br>
sudo apt install python3-opencv <br>
pip3 install opencv-python<br>
sudo apt install python3-dotenv<br>

## Running

Terminal 1:<br>

cd PX4-Autopilot<br>
PX4_GZ_WORLD=colored_blocks_world make px4_sitl gz_x500_mono_cam_world<br>

Terminal 2:<br>
MicroXRCEAgent udp4 -p 8888<br>

Terminal 3:<br>
cd TRACE/experiments/px4_vlm_control<br>
colcon build<br>
source install/setup.bash<br>
ros2 run vlm_control vlm_control<br>