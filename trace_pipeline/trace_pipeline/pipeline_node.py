#!/usr/bin/env python3

import base64
import json
import math
import os

import cv2
import numpy as np
import requests
import rclpy
from dotenv import load_dotenv
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image as RosImage
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus


class TRACEPipeline(Node):
    def __init__(self):
        super().__init__('px4_roomba_vlm_controller')

        # =======================
        # PX4 QoS
        # =======================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # =======================
        # Publishers
        # =======================
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_profile
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos_profile
        )
        self.ts_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos_profile
        )

        # =======================
        # Vehicle status sub
        # =======================
        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile
        )

        # =======================
        # State tracking
        # =======================
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.armed = False
        self.offboard_mode = False
        self.commands_sent = False

        # Position in NED (what we send to PX4 as setpoints)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.yaw = 0.0  # heading in radians (PX4 local NED: 0 = North, +CCW)

        # Simple state machine phases
        self.phase = "INIT"
        self.flight_started = False
        self.nav_ready = False  # becomes True AFTER LAUNCH is complete

        # Roomba-style nav parameters
        self.target_altitude = -1.5   # NED: z = -1 m means 1 m above ground

        # === Safety thresholds (tuned down) ===
        self.depth_threshold = 1.5    # meters; start turning when corridor <= this
        self.turn_clear_margin = 0  # extra margin required to stop turning
        # So we stop going forward at <= 1.5m and only resume forward at >= 1.7m

        self.forward_step = 0.05      # meters per control loop (~1 m/s at 20 Hz)
        self.turn_step = 0.03         # radians per control loop (~1.7° per tick, gentler)
        self.turn_accum = 0.0         # accumulated yaw change in TURN_LEFT

        # =======================
        # Mapping
        # =======================
        # Depth camera params https://github.com/PX4/PX4-gazebo-models/blob/main/models/OakD-Lite/model.sdf
        self.cam_hfov     = 1.274          # radians
        self.cam_width    = 640
        self.cam_height   = 480
        self.cam_near     = 0.2            # metres
        self.cam_far      = 19.1           # metres

        # Map params
        self.map_resolution = 0.05         # cell resolution
        self.map_cols       = 500          # 25 m wide
        self.map_rows       = 500          # 25 m tall

        # origin @ 0,0 maps to (map_cols//2, map_rows//2)
        self.map_origin_col = self.map_cols // 2
        self.map_origin_row = self.map_rows // 2

        # -1 = unknown, 0 = free, 100 = occupied
        self.occ_grid  = np.full((self.map_rows, self.map_cols), -1, dtype=np.int8)

        # Seconds since node start when a cell was last observed
        self.map_stamp = np.zeros((self.map_rows, self.map_cols), dtype=np.float64)

        self.map_pub = self.create_publisher(OccupancyGrid, "/slam/occupancy_grid", 10)
        self.map_img_pub = self.create_publisher(RosImage, "/slam/map_image", 10)
        self.map_pub_counter = 0

        # =======================
        # Depth camera
        # =======================
        self.depth_node = GzNode()
        self.depth_topic = "/depth_camera"
        self.latest_depth_info = None
        self.depth_node.subscribe(GzImage, self.depth_topic, self.gz_depth_callback)

        # =======================
        # RGB camera for VLM
        # =======================
        self.cam_node = GzNode()
        # Use your existing camera topic from the original script
        # self.camera_topic = "/world/small_house/model/trace_drone_0/model/mono_cam/link/camera_link/sensor/camera/image"
        self.camera_topic = ""

        self.latest_image = None
        self.cam_node.subscribe(GzImage, self.camera_topic, self.gz_image_callback)

        # =======================
        # VLM big blue ball detection
        # =======================
        self.vlm_processing = False
        self.ball_hold = False  # True = override Roomba behavior and hold position
        self.vlm_check_interval = 20  # every 20 control loops (~1s at 20 Hz)
        self.vlm_check_counter = 0

        # =======================
        # Control loop
        # =======================
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # =======================
        # Logs
        # =======================
        self.get_logger().info('=' * 60)
        self.get_logger().info('PX4 Autonomous Roomba + VLM Ball Detection')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Flight Plan:')
        self.get_logger().info('  INIT: Engage offboard and arm')
        self.get_logger().info('  LAUNCH: Climb to target altitude')
        self.get_logger().info('  FORWARD: Fly forward along yaw while corridor (L/C/R) is clear')
        self.get_logger().info('  TURN_LEFT: Rotate left until corridor is clear again (with margin)')
        self.get_logger().info('  VLM OVERRIDE: If a big blue sphere is seen, hold position')
        self.get_logger().info(f'  depth_threshold = {self.depth_threshold} m, '
                               f'turn_clear = depth_threshold + {self.turn_clear_margin} m')
        self.get_logger().info('=' * 60)

    # =======================
    # Gazebo RGB camera callback
    # =======================
    def gz_image_callback(self, msg: GzImage):
        """Receive RGB camera frames for VLM."""
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 3))
            # Gazebo Image is typically in BGR, convert to RGB if needed
            img = img[:, :, ::-1]
            # Resize to something manageable for VLM
            self.latest_image = cv2.resize(img, (320, 240))
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")

    # =======================
    # Gazebo depth callback
    # =======================
    def gz_depth_callback(self, msg: GzImage):
        try:
            depth_data = np.frombuffer(msg.data, dtype=np.float32)
            depth_img = depth_data.reshape((msg.height, msg.width))

            h, w = depth_img.shape

            # Use a horizontal band around the center (middle third vertically)
            row_start = h // 3
            row_end = 2 * h // 3
            band = depth_img[row_start:row_end, :]

            # Split band into left / center / right thirds
            col_left_end = w // 3
            col_right_start = 2 * w // 3

            left_band = band[:, :col_left_end]
            center_band = band[:, col_left_end:col_right_start]
            right_band = band[:, col_right_start:]

            # Compute min depth in each region (ignoring NaNs if any)
            left_depth = float(np.nanmin(left_band))
            center_depth = float(np.nanmin(center_band))
            right_depth = float(np.nanmin(right_band))

            # Optional visualization (for debugging)
            depth_normalized = np.clip(depth_img, 0, 10.0) / 10.0 * 255
            depth_vis = depth_normalized.astype(np.uint8)
            depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

            self.latest_depth_info = {
                "img": depth_colored,
                "left": left_depth,
                "center": center_depth,
                "right": right_depth,
            }

            # ---- Mapping: update the occupancy map from the cropped depth image ----
            self._update_mapping(band)

        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    # =======================
    # PX4 status callback
    # =======================
    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg
        if not self.commands_sent:
            self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self.offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    # =======================
    # Depth helpers
    # =======================
    def get_front_depths(self):
        """Return (left, center, right) depths or (None, None, None) if unavailable."""
        if self.latest_depth_info is None:
            return None, None, None
        return (
            self.latest_depth_info["left"],
            self.latest_depth_info["center"],
            self.latest_depth_info["right"],
        )

    # =======================
    # VLM: big blue ball detection
    # =======================
    def check_for_blue_ball_vlm(self):
        """Use the VLM to check if a big blue sphere (yoga ball) is visible."""
        # Don't run detection until nav is ready (post-LAUNCH)
        if not self.nav_ready:
            return

        if self.latest_image is None:
            self.get_logger().debug("No RGB image available for VLM yet.")
            return

        self.vlm_processing = True
        try:
            success, buffer = cv2.imencode(".jpg", self.latest_image)
            if not success:
                self.get_logger().error("Failed to encode RGB image for VLM.")
                self.vlm_processing = False
                return

            img_b64 = base64.b64encode(buffer).decode("utf-8")

            uri = f"http://{os.getenv('OLLAMA_ENDPOINT')}/api/chat"
            system_prompt = """
            You are a visual inspector for a drone flying inside a house.

            Your task:
            Look at the image and decide if there is a clearly visible large blue sphere
            (like a blue yoga ball or exercise ball) anywhere in the frame.

            Output format:
            - Respond with exactly one word:
              - "yes"  -> if a large blue sphere / yoga ball is visible
              - "no"   -> if not

            Do not add any explanation, punctuation, or extra text.
            """

            data = {
                "model": os.getenv('OLLAMA_MODEL'),
                "messages": [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "images": [img_b64]},
                ],
            }

            self.get_logger().debug("Sending request to VLM for blue ball detection...")
            response = requests.post(uri, json=data, stream=True, timeout=10)

            full_reply = ""
            for line in response.iter_lines():
                if not line:
                    continue
                try:
                    json_data = json.loads(line)
                    msg = json_data.get("message", {}).get("content", "")
                    full_reply += msg
                    if json_data.get("done", False):
                        break
                except json.JSONDecodeError:
                    continue

            answer = full_reply.strip().lower()
            self.get_logger().info(f"VLM blue-ball reply: '{answer}'")

            if answer.startswith("yes"):
                if not self.ball_hold:
                    self.get_logger().info("Big blue sphere detected! Entering BALL_HOLD override (hover in place).")
                self.ball_hold = True
            elif answer.startswith("no"):
                if self.ball_hold:
                    self.get_logger().info("Big blue sphere no longer detected. Resuming Roomba navigation.")
                self.ball_hold = False
            else:
                self.get_logger().warn(f"Unexpected VLM reply for blue ball: '{answer}'. Keeping previous state.")

        except Exception as e:
            self.get_logger().error(f"VLM blue-ball detection error: {e}")
        finally:
            self.vlm_processing = False

    # =======================
    # Main control loop
    # =======================
    def control_loop(self):
        # Always publish offboard control + trajectory setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # Initial sequence: send some setpoints, then switch to offboard, then arm
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.get_logger().info("Engaging offboard mode...")

        if self.offboard_setpoint_counter == 20:
            self.arm_force()
            self.get_logger().info("Arming vehicle with force flag...")
            self.commands_sent = True

        # Once armed + in offboard, start flight
        if (self.offboard_setpoint_counter > 30 and
                self.armed and self.offboard_mode and
                not self.flight_started):
            self.flight_started = True
            self.phase = "LAUNCH"
            self.get_logger().info("Vehicle armed and in offboard mode. Starting LAUNCH phase...")

        # Periodic VLM check (blue yoga ball), BUT ONLY after nav_ready
        if self.nav_ready:
            self.vlm_check_counter += 1
            if (self.vlm_check_counter % self.vlm_check_interval == 0
                    and not self.vlm_processing):
                self.check_for_blue_ball_vlm()

        # Update flight phases (Roomba & LAUNCH)
        # Publish map at ~1 Hz (every 20 ticks)
        self.map_pub_counter += 1
        if self.map_pub_counter % 20 == 0:
            self._publish_map()

        if self.flight_started:
            self.update_flight_phase()

        # Optional logging every 2s
        if self.offboard_setpoint_counter % 40 == 0:
            if self.latest_depth_info is not None:
                l = self.latest_depth_info["left"]
                c = self.latest_depth_info["center"]
                r = self.latest_depth_info["right"]
                depth_str = f"L={l:.2f}, C={c:.2f}, R={r:.2f} m"
            else:
                depth_str = "no depth data"
            self.get_logger().info(
                f"Phase: {self.phase} | NavReady: {self.nav_ready} | Armed: {self.armed} | "
                f"Offboard: {self.offboard_mode} | Pos: [{self.pos_x:.2f}, {self.pos_y:.2f}, {self.pos_z:.2f}] | "
                f"Yaw: {self.yaw:.2f} rad | Depths: {depth_str} | "
                f"BallHold: {self.ball_hold}"
            )

        self.offboard_setpoint_counter += 1

    # =======================
    # Roomba-style state machine
    # =======================
    def update_flight_phase(self):
        # LAUNCH phase always runs, even before nav_ready
        left_depth, center_depth, right_depth = self.get_front_depths()
        clear_required = self.depth_threshold + self.turn_clear_margin

        # Phase: LAUNCH -> climb to target altitude
        if self.phase == "LAUNCH":
            if self.pos_z > self.target_altitude:
                # NED: decreasing z means going up
                self.pos_z -= 0.05
            else:
                self.pos_z = self.target_altitude
                self.nav_ready = True  # now Roomba + VLM are allowed
                self.get_logger().info(
                    f"LAUNCH complete at altitude {abs(self.target_altitude):.2f} m. "
                    f"Navigation is now READY. Switching to FORWARD phase..."
                )
                self.phase = "FORWARD"
            return  # Don't do any further logic in this call

        # From this point on, only run nav logic if nav_ready
        if not self.nav_ready:
            return

        # If VLM override is active, just hold current setpoint (no position updates)
        if self.ball_hold:
            return

        # Phase: FORWARD -> move forward along yaw while full corridor is clearly safe
        if self.phase == "FORWARD":
            if self.latest_depth_info is None:
                self.get_logger().warn("No depth data yet; holding position.")
                return

            if (left_depth is None or center_depth is None or right_depth is None):
                self.get_logger().warn("Incomplete depth data; holding position.")
                return

            # Move forward only if entire corridor is beyond depth_threshold
            if (left_depth > self.depth_threshold and
                    center_depth > self.depth_threshold and
                    right_depth > self.depth_threshold):
                # Move forward in *body frame* direction (aligned with yaw)
                dx = self.forward_step * math.cos(self.yaw)
                dy = self.forward_step * math.sin(self.yaw)

                self.pos_x += dx
                self.pos_y += dy

                self.get_logger().debug(
                    f"FORWARD: yaw={self.yaw:.2f} rad, step=({dx:.3f}, {dy:.3f}), "
                    f"pos=({self.pos_x:.2f}, {self.pos_y:.2f})"
                )
            else:
                self.get_logger().info(
                    f"Obstacle detected in corridor: "
                    f"L={left_depth:.2f}, C={center_depth:.2f}, R={right_depth:.2f} "
                    f"(threshold={self.depth_threshold}). Switching to TURN_LEFT..."
                )
                self.phase = "TURN_LEFT"
                self.turn_accum = 0.0

        # Phase: TURN_LEFT -> rotate in place until corridor is clear *plus margin*
        elif self.phase == "TURN_LEFT":
            if self.latest_depth_info is not None:
                if (left_depth is not None and center_depth is not None and right_depth is not None and
                        left_depth > clear_required and
                        center_depth > clear_required and
                        right_depth > clear_required):
                    self.get_logger().info(
                        f"Path cleared after turn with margin: "
                        f"L={left_depth:.2f}, C={center_depth:.2f}, R={right_depth:.2f} "
                        f"(required>{clear_required:.2f}). Switching back to FORWARD..."
                    )
                    self.phase = "FORWARD"
                    return
            else:
                self.get_logger().warn("No depth data during TURN_LEFT; continuing turn.")

            # Rotate left in place (body frame yaw)
            self.yaw += self.turn_step
            self.turn_accum += abs(self.turn_step)

            # Normalize yaw to [-pi, pi]
            if self.yaw > math.pi:
                self.yaw -= 2 * math.pi
            elif self.yaw < -math.pi:
                self.yaw += 2 * math.pi

            # Safety: if we've rotated more than one full circle and still no path, just log
            if self.turn_accum > 2 * math.pi:
                self.get_logger().warn(
                    "Completed >360° in TURN_LEFT without finding a clear path. "
                    "Consider adding a fallback behavior (e.g., climb or land)."
                )
                # For now, keep spinning; you can add a FAILSAFE or HOVER phase here.

    # =======================
    # Mapping helpers
    # =======================

    def _world_to_grid(self, wx, wy):
        """Convert a sim position to (col, row) grid cell. Returns None if out of bounds."""
        col = int(round(wy / self.map_resolution)) + self.map_origin_col
        row = int(round(wx / self.map_resolution)) + self.map_origin_row
        if 0 <= col < self.map_cols and 0 <= row < self.map_rows:
            return col, row
        return None

    def _update_mapping(self, cropped_depth: np.ndarray):
        """Core mapping update called on every depth frame."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # project minimum across columns, collapsing cropped image to 1d depth array
        col_min = np.nanmin(cropped_depth, axis=0)   # shape: (cam_width,)

        W = self.cam_width
        col_indices = np.arange(W, dtype=np.float64)
        bearings = self.yaw + (col_indices - W / 2.0) * (self.cam_hfov / W)

        drone_cell = self._world_to_grid(self.pos_x, self.pos_y)
        if drone_cell is None:
            return
        d_col, d_row = drone_cell

        for c in range(W):
            depth = float(col_min[c])

            if not np.isfinite(depth) or depth < self.cam_near or depth > self.cam_far:
                continue

            obs_x = self.pos_x + depth * math.cos(bearings[c])
            obs_y = self.pos_y + depth * math.sin(bearings[c])

            obs_cell = self._world_to_grid(obs_x, obs_y)
            if obs_cell is None:
                continue
            obs_col, obs_row = obs_cell

            # raycast cells as free up to the obstacle
            ray_len = max(abs(obs_col - d_col), abs(obs_row - d_row))
            if ray_len < 1:
                continue

            sample_cols = np.round(np.linspace(d_col, obs_col, ray_len, endpoint=False)).astype(int)
            sample_rows = np.round(np.linspace(d_row, obs_row, ray_len, endpoint=False)).astype(int)

            valid = (
                (sample_cols >= 0) & (sample_cols < self.map_cols) &
                (sample_rows >= 0) & (sample_rows < self.map_rows)
            )
            fc, fr = sample_cols[valid], sample_rows[valid]

            # mark free if not already confirmed occupied 
            free_mask = self.occ_grid[fr, fc] != 100
            self.occ_grid[fr[free_mask], fc[free_mask]] = 0
            self.map_stamp[fr[free_mask], fc[free_mask]] = now_sec

            # mark obstacle
            self.occ_grid[obs_row, obs_col] = 100
            self.map_stamp[obs_row, obs_col] = now_sec

    def _publish_map(self):
        """Publish map grid and rendered image."""
        now = self.get_clock().now().to_msg()

        # nav_msgs/OccupancyGrid -> /slam/occupancy_grid
        og = OccupancyGrid()
        og.header.stamp    = now
        og.header.frame_id = "map"
        og.info.resolution = self.map_resolution
        og.info.width      = self.map_cols
        og.info.height     = self.map_rows
        og.info.origin.position.x = -(self.map_origin_col * self.map_resolution)
        og.info.origin.position.y = -(self.map_origin_row * self.map_resolution)
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0
        og.data = self.occ_grid.flatten().tolist()
        self.map_pub.publish(og)

        # sensor_msgs/Image -> /slam/map_image
        render = self._render_map()
        img_msg = RosImage()
        img_msg.header.stamp    = now
        img_msg.header.frame_id = "map"
        img_msg.height   = render.shape[0]
        img_msg.width    = render.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.step     = render.shape[1] * 3
        img_msg.data     = render.tobytes()
        self.map_img_pub.publish(img_msg)

    def _render_map(self) -> np.ndarray:
        """
        Renders a colorized top-down map image as follows:

        unknown  - dark grey
        free     - grey, fades darker with time
        occupied - red, fades orange with time
        drone    - green
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        age_max = 30.0  

        img = np.zeros((self.map_rows, self.map_cols, 3), dtype=np.uint8)
        img[self.occ_grid == -1] = (40, 40, 40)

        free_mask = self.occ_grid == 0
        if free_mask.any():
            age = np.clip((now_sec - self.map_stamp[free_mask]) / age_max, 0, 1)
            brightness = (200 - age * 80).astype(np.uint8)
            img[free_mask] = np.stack([brightness, brightness, brightness], axis=-1)

        occ_mask = self.occ_grid == 100
        if occ_mask.any():
            age = np.clip((now_sec - self.map_stamp[occ_mask]) / age_max, 0, 1)
            r = np.zeros(occ_mask.sum(), dtype=np.uint8)
            g = (age * 120).astype(np.uint8)
            b = np.full(occ_mask.sum(), 200, dtype=np.uint8)
            img[occ_mask] = np.stack([r, g, b], axis=-1)

        dc = self._world_to_grid(self.pos_x, self.pos_y)
        if dc is not None:
            col, row = dc
            r0, r1 = max(0, row - 1), min(self.map_rows, row + 2)
            c0, c1 = max(0, col - 1), min(self.map_cols, col + 2)
            img[r0:r1, c0:c1] = (0, 255, 0)

        return img

    # =======================
    # PX4 helpers
    # =======================
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [self.pos_x, self.pos_y, self.pos_z]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = self.yaw
        msg.yawspeed = float('nan')
        self.ts_pub.publish(msg)

    def engage_offboard_mode(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)
        self.offboard_mode = True

    def arm_force(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0
        msg.param2 = 21196.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)
        self.armed = True

    def disarm(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 0.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def __del__(self):
        # You can add unsubscribe calls here if you store the handles.
        pass


def main(args=None):
    rclpy.init(args=args)
    env_path = os.path.expanduser('~/TRACE/.env')
    load_dotenv(dotenv_path=env_path)

    controller = TRACEPipeline()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Keyboard interrupt - disarming...")
        controller.disarm()
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
