#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus
from pynput import keyboard


class PX4KeyboardController(Node):
    def __init__(self):
        super().__init__('px4_keyboard_controller')
        
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
        self.motors_on = False
        self.active_keys = set()
        
        # Position tracking (NED frame)
        self.pos_x = 0.0  # North
        self.pos_y = 0.0  # East
        self.pos_z = -5.0  # Down (negative = up, start at 5m altitude)
        self.yaw = 0.0
        
        # Speed parameters
        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('angular_speed', 0.5)
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # Control loop timer at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        
        # Print instructions
        self.get_logger().info('=' * 60)
        self.get_logger().info('PX4 Keyboard Controller')
        self.get_logger().info('=' * 60)
        self.get_logger().info('FIRST - Run these in PX4 console (pxh>):')
        self.get_logger().info('  param set COM_ARM_WO_GPS 1')
        self.get_logger().info('  param set CBRK_SUPPLY_CHK 894248')
        self.get_logger().info('  param set NAV_RCL_ACT 0')
        self.get_logger().info('  param set NAV_DLL_ACT 0')
        self.get_logger().info('  param set COM_RCL_EXCEPT 4')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Controls:')
        self.get_logger().info('  SPACE - Toggle motors (arms/engages offboard)')
        self.get_logger().info('  W/S   - Forward/Backward (North/South)')
        self.get_logger().info('  A/D   - Left/Right (West/East)')
        self.get_logger().info('  Q/E   - Yaw left/right')
        self.get_logger().info('  R/F   - Up/Down (altitude)')
        self.get_logger().info('  ESC   - Emergency disarm and quit')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Make sure this terminal is in focus!')
        self.get_logger().info('=' * 60)

    def vehicle_status_callback(self, msg):
        """Track vehicle status"""
        self.vehicle_status = msg
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def on_press(self, key):
        """Handle key press events"""
        try:
            if hasattr(key, 'char') and key.char:
                char = key.char.lower()
                if char in ['w', 's', 'a', 'd', 'q', 'e', 'r', 'f']:
                    if char not in self.active_keys:
                        self.active_keys.add(char)
                        self.get_logger().info(f'Key pressed: {char}')
            elif key == keyboard.Key.esc:
                self.get_logger().info('ESC pressed - EMERGENCY DISARM')
                self.motors_on = False
                self.disarm()
                self.listener.stop()
                rclpy.shutdown()
            elif key == keyboard.Key.space:
                self.motors_on = not self.motors_on
                status = "ON (Arming...)" if self.motors_on else "OFF (Disarming...)"
                self.get_logger().info(f'Motors {status}')
                if not self.motors_on:
                    self.disarm()
        except AttributeError:
            pass
    
    def on_release(self, key):
        """Handle key release events"""
        try:
            if hasattr(key, 'char') and key.char:
                char = key.char.lower()
                self.active_keys.discard(char)
        except AttributeError:
            pass

    def control_loop(self):
        """Main control loop at 20Hz"""
        
        # Always publish offboard mode and setpoint when motors are on
        if self.motors_on or self.offboard_setpoint_counter < 30:
            self.publish_offboard_control_mode()
            self.update_position_setpoint()
            self.publish_trajectory_setpoint()
        
        # Initial startup sequence
        if self.offboard_setpoint_counter == 10 and self.motors_on:
            self.engage_offboard_mode()
            self.get_logger().info("Engaging offboard mode...")
        
        if self.offboard_setpoint_counter == 20 and self.motors_on and not self.armed:
            self.arm_force()
            self.get_logger().info("Arming vehicle with force flag...")
        
        # Status logging every 2 seconds
        if self.offboard_setpoint_counter % 40 == 0 and self.motors_on:
            self.get_logger().info(
                f"Armed: {self.armed} | Offboard: {self.offboard_mode} | "
                f"Pos: [{self.pos_x:.1f}, {self.pos_y:.1f}, {self.pos_z:.1f}] | "
                f"Yaw: {self.yaw:.2f}"
            )
        
        if self.motors_on:
            self.offboard_setpoint_counter += 1
        else:
            self.offboard_setpoint_counter = 0

    def update_position_setpoint(self):
        """Update target position based on keyboard input"""
        if not self.motors_on or not self.armed:
            return
        
        dt = 0.05  # 20Hz = 0.05s
        
        # Forward/Backward (X - North/South in NED)
        if 'w' in self.active_keys:
            self.pos_x += self.linear_speed * dt
        elif 's' in self.active_keys:
            self.pos_x -= self.linear_speed * dt
        
        # Left/Right (Y - East/West in NED)
        if 'a' in self.active_keys:
            self.pos_y -= self.linear_speed * dt  # West is negative
        elif 'd' in self.active_keys:
            self.pos_y += self.linear_speed * dt  # East is positive
        
        # Up/Down (Z - Down is positive in NED, so inverted)
        if 'r' in self.active_keys:
            self.pos_z -= self.linear_speed * dt  # Up is negative Z
        elif 'f' in self.active_keys:
            self.pos_z += self.linear_speed * dt  # Down is positive Z
        
        # Yaw rotation
        if 'q' in self.active_keys:
            self.yaw += self.angular_speed * dt
        elif 'e' in self.active_keys:
            self.yaw -= self.angular_speed * dt
        
        # Clamp altitude (don't go below ground or too high)
        self.pos_z = max(-50.0, min(self.pos_z, -0.5))  # -50m to -0.5m altitude
        
        # Log movement
        if self.active_keys and self.offboard_setpoint_counter % 10 == 0:
            self.get_logger().info(
                f"Target: [{self.pos_x:.1f}, {self.pos_y:.1f}, {self.pos_z:.1f}] "
                f"Yaw: {self.yaw:.2f}",
                throttle_duration_sec=0.5
            )

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
        msg.param1 = 1.0  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg.param2 = 6.0  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def arm_force(self):
        """Arm the vehicle WITH FORCE FLAG to bypass preflight"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0  # Arm
        msg.param2 = 21196.0  # Magic number: force arm
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)
    
    def disarm(self):
        """Disarm the vehicle"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 0.0  # Disarm
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = PX4KeyboardController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Keyboard interrupt - disarming...")
    finally:
        controller.motors_on = False
        controller.disarm()
        controller.listener.stop()
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

