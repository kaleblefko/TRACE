## Setup:
On your computer outside the VM run<br>
export OLLAMA_HOST=0.0.0.0<br>
ollama serve<br>

To get the IP address do: ipconfig getifaddr en0 (MacOS)<br>

Must have a .env stored at ~/TRACE/.env<br>
See .env.example for an example .env file<br>

# TRACE
Running Trace Pipeline in VM:

# Terminal 1
cd ~/PX4-Autopilot
PX4_GZ_WORLD=small_house make px4_sitl gz_trace_drone

if the drone is not ready for takeoff after these commands, set the following parameters:

param set CBRK_SUPPLY_CHK 894248
param set COM_ARM_WO_GPS 1
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0

# Terminal 2
MicroXRCEAgent udp4 -p 8888

# Terminal 3
colcon build
source install/setup.bash
ros2 run trace_pipeline vlm_node

# Terminal 4
source install/setup.bash
ros2 run trace_pipeline pipeline_node

# Terminal 5
if not performed already:
sudo apt install ros-humble-rtabmap-ros

source install/setup.bash
ros2 run trace_pipeline mapping_node

# Terminal 6
ros2 run rtabmap_slam rtabmap \
  --ros-args \
  -p subscribe_depth:=true \
  -p subscribe_rgb:=true \
  -p subscribe_odom:=true \
  -p frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p approx_sync:=true \
  -p "Grid/FromDepth:='true'" \
  -p "Grid/RayTracing:='true'" \
  -p "Grid/RangeMax:='8.0'" \
  -p "Grid/RangeMin:='0.2'" \
  -p "Grid/MaxObstacleHeight:='2.0'" \
  -p "Grid/MinGroundHeight:='0.1'" \
  -p "RGBD/AngularUpdate:='0.05'" \
  -p "RGBD/LinearUpdate:='0.05'" \
  --remap rgb/image:=/camera/rgb/image_raw \
  --remap rgb/camera_info:=/camera/rgb/camera_info \
  --remap depth/image:=/camera/depth/image_raw \
  --remap depth/camera_info:=/camera/depth/camera_info \
  --remap odom:=/rtabmap/odom \
  --remap map:=/slam/occupancy_grid \
  -- --delete_db_on_start

# Terminal 7
ros2 run rtabmap_viz rtabmap_viz \
  --ros-args \
  -p subscribe_depth:=true \
  -p subscribe_odom:=true \
  -p frame_id:=base_link \
  --remap rgb/image:=/camera/rgb/image_raw \
  --remap rgb/camera_info:=/camera/rgb/camera_info \
  --remap depth/image:=/camera/depth/image_raw \
  --remap depth/camera_info:=/camera/depth/camera_info \
  --remap odom:=/rtabmap/odom

# Terminal 8 (Optional, if not using VLM node)
source install/setup.bash
ros2 run trace_pipeline teleop

# How to Get Custom Drone Working
cd ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
 
cp 4002_gz_x500_depth 4022_gz_trace_drone
nano 4022_gz_trace_drone
When nano opens change the following
 
# @name Gazebo Trace Drone
PX4_SIM_MODEL=${PX4_SIM_MODEL:=trace_drone}
 
Then nano CMakeLists.txt, add into the arguments of px4_add_romfs_files, "4022_gz_trace_drone". Just scroll in the file and find 4021_gz_x500_flow and add it below.
 
Once done.
 
cd ~/PX4-Autopilot
rm -rf build/
make clean
make px4_sitl
 
After px4 is done building run 
 
cd ~/TRACE
bash update_px4.sh
 
Then make px4_sitl gz_trace_drone should work.
