How to build and run:

Build:

First need to install pynput for keyboard monitoring
sudo apt-get install python3-pip
pip3 install pynput --break-system-packages

colcon build --packages-select quadcopter_keyboard_control

Run:

Terminal 1 gz sim multicopter_velocity_control.sdf
Terminal 2 (while in /quadcopter_ws) ./start_x3_bridge.sh
Terminal 3 (In VSCode and while in /quadcopter_ws) ./start_keyboard_controller.sh 