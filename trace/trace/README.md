# Running TRACE Find-the-Blue-Ball Pipeline

## OLLAMA Setup
On your computer outside the VM run:
```
export OLLAMA_HOST=0.0.0.0
ollama serve
```
To get the IP address: `ipconfig getifaddr en0` (macOS)

Must have a `.env` stored at `~/TRACE/.env` — see `.env.example` for the required fields.

To stop ollama if you get an "address already bound" error:
```
systemctl stop ollama
```

## Terminal 1 — PX4 Simulator
```
cd ~/PX4-Autopilot
PX4_GZ_WORLD=small_house make px4_sitl gz_trace_drone
```

If the drone is not ready for takeoff, set the following parameters in the PX4 shell:
```
param set CBRK_SUPPLY_CHK 894248
param set COM_ARM_WO_GPS 1
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
```

## Terminal 2 — TRACE Stack
Ensure `xterm` is installed (one-time):
```
sudo apt install xterm
```

Build once (or after any code change), then launch everything with a single command:
```
cd ~/TRACE
colcon build
source install/setup.bash
ros2 launch trace trace.launch.py
```

Each node opens in its own labeled xterm window and starts sequentially so that dependencies are ready before the next node comes up:

| Delay | Window | Node |
|-------|--------|------|
| 0 s | `MicroXRCEAgent` | PX4 ↔ ROS2 bridge |
| 15 s | `mapping` | SLAM occupancy-grid mapper |
| 25 s | `bbox` | Object detector |
| 35 s | `nav` | SLAM-aware navigator (1.25 m altitude, 0.4 m obstacle clearance) |
| 45 s | `visualizer` | Detection visualizer |
| 60 s | `pipeline` | High-level state machine |
| 65 s | `text_input` | Operator text interface |

Once the drone finds and hovers above the target (`ARRIVED`) or completes a full 360° sweep without finding it (`SCAN_FAILED`), the `text_input` window will prompt:
```
[TRACE] Object found — drone is hovering above target.
[TRACE] Enter next target object: 
```
Type a new object label and press Enter. The drone will fly back to its takeoff origin and begin a fresh 360° scan searching for the new object. The object detector updates its target automatically — no restart needed.

To also open RViz2 at launch:
```
ros2 launch trace trace.launch.py use_rviz:=true
```
In RViz2 add the topic `/slam/occupancy_grid` (OccupancyGrid) to visualize the map.
