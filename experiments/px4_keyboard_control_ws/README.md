## Before running:

cp worlds/colored_blocks_world.sdf ~/PX4_Autpilot/Tools/simulation/gz/worlds/

## To Run
# Terminal 1:
PX4_GZ_WORLD=colored_blocks_world make px4_sitl gz_x500_mono_cam_down
Once this is done starting up, in that terminal type
param set NAV_DLL_ACT 0 (It should then say in green Ready for takeoff!)

# Terminal 2:
MicroXRCEAgent udp4 -p 8888

# Terminal 3:
python3 /TRACE/experiments/px4_keyboard_control_ws/get_camera_image.py

# Terminal 4 (In Vscode):
sudo apt install python3-pynput (or pip, didn't work for me)
cd px4_keyboard_control_ws
colcon build
source install/setup.bash
ros2 run drone_controller offboard_ctrl
