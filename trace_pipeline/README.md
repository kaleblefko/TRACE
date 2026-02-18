# TRACE
Running Trace Pipeline:

# Terminal 1
cd ~/PX4-Autopilot
PX4_GZ_WORLD=small_house make px4_sitl gz_trace_drone

# Terminal 2
MicroXRCEAgent udp4 -p 8888

# Terminal 3
cd ~/TRACE/trace_pipeline
colcon build
source install/setup.bash
ros2 run trace_pipeline vlm_node

# Terminal 4
cd ~/TRACE/trace_pipeline
source install/setup.bash
ros2 run trace_pipeline pipeline_node



