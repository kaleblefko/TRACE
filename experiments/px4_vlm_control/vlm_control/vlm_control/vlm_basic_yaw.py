#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus
import numpy as np
import cv2
import requests
import json
import base64
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
import os
from dotenv import load_dotenv


class PX4VLMController(Node):
    def __init__(self):
        super().__init__('px4_vlm_controller')
        
        # Configure QoS profile for PX4 DDS topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
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
        
        # Subscriber to monitor vehicle status
        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile
        )
        
        # State tracking
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.armed = False
        self.offboard_mode = False
        self.commands_sent = False  # Track if we've sent the arm/offboard commands
        
        # Position tracking (NED frame)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.yaw = 1.787

        self.phase = "INIT"
        self.flight_started = False
        
        # VLM state
        self.move_command = None
        self.target_x = 0.0
        self.target_y = 0.0
        self.vlm_processing = False

        # Movement planning state
        self.movement_stage = None  # None / "TURNING" / "FORWARD"
        self.target_yaw = self.yaw
        self.move_distance = 2.0    # meters to move forward after turning
        self.start_x = self.pos_x
        self.start_y = self.pos_y
        
        # Gazebo camera setup
        self.gz_node = GzNode()
        self.camera_topic = "/world/arrows/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
        self.latest_image = None
        self.gz_node.subscribe(GzImage, self.camera_topic, self.gz_image_callback)
        
        # Control loop timer at 20Hz (always running for offboard mode)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('PX4 Autonomous Flight with VLM')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Flight Plan:')
        self.get_logger().info('  INIT: Engage offboard and arm')
        self.get_logger().info('  LAUNCH: Go up to -2.0 (2m altitude)')
        self.get_logger().info('  START: Move to (0, 2, -2)')
        self.get_logger().info('  VLM: Process camera and get command')
        self.get_logger().info('  MOVING: Execute VLM command')
        self.get_logger().info('  HOVER: Wait 10 seconds after movement')
        self.get_logger().info('=' * 60)

    def gz_image_callback(self, msg: GzImage):
        """Non-blocking callback for Gazebo camera images"""
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 3))
            img = img[:, :, ::-1]  # BGR to RGB
            self.latest_image = cv2.resize(img, (320, 240))
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def vehicle_status_callback(self, msg):
        """Track vehicle status - but don't overwrite if we've already set them"""
        self.vehicle_status = msg
        # Only update from callback if commands haven't been sent yet
        if not self.commands_sent:
            self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self.offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def control_loop(self):
        """Main control loop at 20Hz - ALWAYS runs to maintain offboard mode"""
        
        # Always publish offboard mode and setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        
        # Initial startup sequence
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.get_logger().info("Engaging offboard mode...")
        
        if self.offboard_setpoint_counter == 20:
            self.arm_force()
            self.get_logger().info("Arming vehicle with force flag...")
            self.commands_sent = True  # Mark that we've sent the commands
        
        # Wait for vehicle to be armed and in offboard before starting flight
        if self.offboard_setpoint_counter > 30 and self.armed and self.offboard_mode and not self.flight_started:
            self.flight_started = True
            self.phase = "LAUNCH"
            self.get_logger().info("Vehicle armed and in offboard mode. Starting LAUNCH phase...")
        
        # Only update flight phase after initialization is complete
        if self.flight_started:
            self.update_flight_phase()

        # Status logging every 2 seconds
        if self.offboard_setpoint_counter % 40 == 0:
            self.get_logger().info(
                f"Phase: {self.phase} | Armed: {self.armed} | Offboard: {self.offboard_mode} | "
                f"Pos: [{self.pos_x:.2f}, {self.pos_y:.2f}, {self.pos_z:.2f}]"
            )
        
        self.offboard_setpoint_counter += 1

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def update_flight_phase(self):
        """Update position setpoint based on flight phase"""
        if self.phase == "LAUNCH":
            if self.pos_z > -1.0:
                self.pos_z -= 0.05
            else:
                self.pos_z = -1.0
                self.prev_phase = "LAUNCH"
                self.phase = "HOVER"
                self.get_logger().info(f"LAUNCH complete at altitude 1m. Moving to START phase...")
        
        elif self.phase == "START":
            if self.pos_y < 2.0:
                self.pos_y += 0.05
            else:
                self.pos_y = 2.0
                self.prev_phase = "START"
                self.phase = "HOVER"
                self.get_logger().info(f"START complete at position [0, 2, -1]. Ready for VLM...")

        elif self.phase == "VLM":
            # Process VLM command once
            if not self.vlm_processing:
                self.get_logger().info("VLM phase - processing frame...")
                self.process_vlm_command()
        
        elif self.phase == "MOVING":
            # Two-stage behavior: 1) TURNING to target_yaw, 2) FORWARD along that heading
            if self.movement_stage == "TURNING":
                # Rotate towards target_yaw
                yaw_error = self.normalize_angle(self.target_yaw - self.yaw)
                max_yaw_step = 0.05  # rad per control loop (~1 rad/s at 20 Hz)

                if abs(yaw_error) < 0.02:
                    # Close enough - snap to target and start moving forward
                    self.yaw = self.target_yaw
                    self.movement_stage = "FORWARD"
                    self.get_logger().info(
                        f"Rotation complete (yaw={self.yaw:.2f} rad). Moving forward..."
                    )
                else:
                    yaw_step = np.clip(yaw_error, -max_yaw_step, max_yaw_step)
                    self.yaw += yaw_step

            elif self.movement_stage == "FORWARD":
                # Move forward in the direction of current yaw
                step = 0.05  # meters per control loop
                self.pos_x += step * np.cos(self.yaw)
                self.pos_y += step * np.sin(self.yaw)

                distance = np.sqrt(
                    (self.pos_x - self.start_x) ** 2 +
                    (self.pos_y - self.start_y) ** 2
                )

                if distance >= self.move_distance - 0.05:
                    self.get_logger().info(
                        f"Forward movement complete (moved ~{distance:.2f} m). "
                        "Entering HOVER phase..."
                    )
                    self.phase = "HOVER"
                    self.vlm_processing = False
                    self.movement_stage = None

        
        elif self.phase == "HOVER":
            # Hold position for 10 seconds before next VLM processing
            if not hasattr(self, 'hover_counter'):
                self.hover_counter = 0
                self.get_logger().info("HOVER: Waiting 10 seconds before next VLM processing...")
            
            self.hover_counter += 1
            
            # Log every 5 seconds (100 iterations at 20Hz)
            if self.hover_counter % 20 == 0:
                time_remaining = 10 - (self.hover_counter / 20)
                self.get_logger().info(f"HOVER: {time_remaining:.1f} seconds remaining...")
            
            if self.hover_counter >= 200:  # 15 seconds at 20Hz = 300 iterations
                self.get_logger().info("HOVER complete. Returning to VLM phase...")
                del self.hover_counter
                if self.prev_phase == "LAUNCH":
                    self.phase = "START"
                else:
                    self.phase = "VLM"

    def process_vlm_command(self):
        """Process VLM command using latest camera image"""
        if self.latest_image is None:
            self.get_logger().warn("No camera image available yet")
            return
        
        self.vlm_processing = True
        
        try:
            # Encode image
            success, buffer = cv2.imencode(".jpg", self.latest_image)
            if not success:
                self.get_logger().error("Failed to encode image")
                self.vlm_processing = False
                return
            
            img_b64 = base64.b64encode(buffer).decode("utf-8")
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()
            
            # Call VLM API
            uri = f"http://{os.getenv('OLLAMA_ENDPOINT')}/api/chat"
            system_prompt = """
            Arrow-Guided Drone Control System Prompt

            You are a drone-control agent. You receive an image and must choose exactly ONE action.

            ACTION SPACE (only these three words are allowed):

            forward

            left

            right

            ARROW → ACTION MAPPING (must ALWAYS follow this mapping):

            Arrow pointing up / forward → forward

            Arrow pointing left → left

            Arrow pointing right → right

            BEHAVIOR RULES:

            Identify all arrows visible in the image, including partial or incomplete arrows.

            If multiple arrows or ambiguous arrow shapes are present, select the arrow whose center is closest to the center of the image.

            Determine the direction that the selected arrow is pointing, even if the arrow is partially visible.

            Map the arrow direction to the corresponding action.

            Output ONLY the action word, nothing else.

            IMPORTANT CONSTRAINTS:

            Ignore all colors, shapes, and objects other than arrows.

            Do not explain your reasoning.

            Do not output anything except one valid action word.
            """
            
            data = {
                "model": os.getenv('OLLAMA_MODEL'),
                "messages": [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "images": [img_b64]}
                ],
            }
            
            self.get_logger().info("Sending request to VLM...")
            response = requests.post(uri, json=data, stream=True, timeout=10)
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()
            
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
            
            # Parse command and plan a "turn then forward" maneuver
            command = full_reply.strip().lower()
            self.get_logger().info(f"VLM Command: {command}")

            # Save starting position for forward distance tracking
            self.start_x = self.pos_x
            self.start_y = self.pos_y
            self.move_distance = 2.0  # meters

            if "forward" in command:
                self.move_command = "forward"
                # Keep current yaw, just move forward
                self.target_yaw = self.yaw

            elif "right" in command:
                self.move_command = "right"
                # Turn 90 degrees right (negative rotation in NED frame)
                self.target_yaw = self.yaw + (np.pi / 2.0)

            elif "left" in command:
                self.move_command = "left"
                # Turn 90 degrees left (positive rotation)
                self.target_yaw = self.yaw - (np.pi / 2.0)

            else:
                self.get_logger().warn(f"Unknown command: {command}, staying in VLM phase")
                self.vlm_processing = False
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint()
                return

            # Normalize target yaw for stability
            self.target_yaw = self.normalize_angle(self.target_yaw)

            # Start MOVING phase with a TURNING sub-stage
            self.movement_stage = "TURNING"
            self.phase = "MOVING"

            self.get_logger().info(
                f"Planned maneuver: {self.move_command} -> "
                f"turn to yaw={self.target_yaw:.2f} rad, then move forward {self.move_distance} m"
            )

            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()

                
        except Exception as e:
            self.get_logger().error(f"VLM processing error: {e}")
            self.vlm_processing = False

    def publish_offboard_control_mode(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        """Publish position setpoint in NED frame"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [self.pos_x, self.pos_y, self.pos_z]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = self.yaw
        msg.yawspeed = float('nan')
        self.ts_pub.publish(msg)

    def engage_offboard_mode(self):
        """Switch to offboard mode"""
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
        self.offboard_mode = True  # Set to true immediately

    def arm_force(self):
        """Arm the vehicle WITH FORCE FLAG to bypass preflight"""
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
        self.armed = True  # Set to true immediately
    
    def disarm(self):
        """Disarm the vehicle"""
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
        """Cleanup Gazebo node on destruction"""
        if hasattr(self, 'gz_node'):
            self.gz_node.unsubscribe(self.camera_topic)


def main(args=None):
    rclpy.init(args=args)
    env_path = os.path.expanduser('~/TRACE/.env')
    load_dotenv(dotenv_path=env_path)
    controller = PX4VLMController()
    
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


