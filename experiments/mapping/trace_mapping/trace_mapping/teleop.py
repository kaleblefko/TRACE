#!/usr/bin/env python3

import math
import threading
import pygame
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus

# Window size for the pygame HUD
WIN_W, WIN_H = 480, 320

# Colors
BLACK  = (0,   0,   0)
WHITE  = (255, 255, 255)
GREEN  = (0,   220, 80)
RED    = (220, 50,  50)
YELLOW = (240, 200, 0)
GRAY   = (80,  80,  80)
DGRAY  = (30,  30,  30)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_pub      = self.create_publisher(VehicleCommand,       "/fmu/in/vehicle_command",       qos_profile)
        self.offboard_pub = self.create_publisher(OffboardControlMode,  "/fmu/in/offboard_control_mode", qos_profile)
        self.ts_pub       = self.create_publisher(TrajectorySetpoint,   "/fmu/in/trajectory_setpoint",   qos_profile)
        self.status_sub   = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.vehicle_status_callback, qos_profile)

        # State
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.armed         = False
        self.offboard_mode = False
        self.motors_on     = False
        self.active_keys   = set()
        self._lock         = threading.Lock()
        self._running      = True   # set to False to break the main loop cleanly

        # Position (NED)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = -1.0
        self.yaw   = 1.787

        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('angular_speed', 0.5)
        self.linear_speed  = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # ROS timer at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)


        # pygame runs on the main thread; ROS spins in a background thread
        self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._ros_thread.start()

    # ------------------------------------------------------------------
    # ROS spin (background thread)
    # ------------------------------------------------------------------

    def _ros_spin(self):
        rclpy.spin(self)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.armed         = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard_mode = (msg.nav_state    == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _normalize_angle(self, angle):
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    # ------------------------------------------------------------------
    # pygame event processing (called from main thread each frame)
    # ------------------------------------------------------------------

    def process_pygame_events(self):
        """Call this every frame from the pygame loop. Returns False when it's time to quit."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._shutdown()

            elif event.type == pygame.KEYDOWN:
                k = event.key

                if k == pygame.K_ESCAPE:
                    self.get_logger().info('ESC - EMERGENCY DISARM')
                    self._shutdown()

                elif k == pygame.K_SPACE:
                    self.motors_on = not self.motors_on
                    self.get_logger().info(f'Motors {"ON" if self.motors_on else "OFF"}')
                    if not self.motors_on:
                        self.disarm()

                # Q/E = snap yaw 90° + auto-forward
                elif k == pygame.K_q:
                    with self._lock:
                        self.yaw = self._normalize_angle(self.yaw - math.pi / 2.0)
                        self.active_keys.add('q')
                    self.get_logger().info(f'Snap left 90° → yaw {math.degrees(self.yaw):.1f}°')

                elif k == pygame.K_e:
                    with self._lock:
                        self.yaw = self._normalize_angle(self.yaw + math.pi / 2.0)
                        self.active_keys.add('e')
                    self.get_logger().info(f'Snap right 90° → yaw {math.degrees(self.yaw):.1f}°')

                # W/S = forward/backward
                elif k == pygame.K_w: self.active_keys.add('w')
                elif k == pygame.K_s: self.active_keys.add('s')

                # A/D = continuous yaw
                elif k == pygame.K_a: self.active_keys.add('a')
                elif k == pygame.K_d: self.active_keys.add('d')

                # R/F = altitude
                elif k == pygame.K_r: self.active_keys.add('r')
                elif k == pygame.K_f: self.active_keys.add('f')

            elif event.type == pygame.KEYUP:
                k = event.key
                # Releasing Q/E stops the auto-forward that was injected on press
                if   k == pygame.K_q or k == pygame.K_e: self.active_keys.discard('w')
                elif k == pygame.K_w: self.active_keys.discard('w')
                elif k == pygame.K_s: self.active_keys.discard('s')
                elif k == pygame.K_a: self.active_keys.discard('a')
                elif k == pygame.K_d: self.active_keys.discard('d')
                elif k == pygame.K_r: self.active_keys.discard('r')
                elif k == pygame.K_f: self.active_keys.discard('f')

    def _shutdown(self):
        """Disarm, close pygame, and signal the main loop to exit."""
        self.motors_on = False
        self.disarm()
        self._running = False   # breaks the while loop in main()

    # ------------------------------------------------------------------
    # ROS control loop (20 Hz timer)
    # ------------------------------------------------------------------

    def control_loop(self):
        if self.motors_on or self.offboard_setpoint_counter < 30:
            self.publish_offboard_control_mode()
            self.update_position_setpoint()
            self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter == 10 and self.motors_on:
            self.engage_offboard_mode()
            self.get_logger().info("Engaging offboard mode...")

        if self.offboard_setpoint_counter == 20 and self.motors_on and not self.armed:
            self.arm_force()
            self.get_logger().info("Arming vehicle with force flag...")

        if self.offboard_setpoint_counter % 40 == 0 and self.motors_on:
            self.get_logger().info(
                f"Armed: {self.armed} | Offboard: {self.offboard_mode} | "
                f"Pos: [{self.pos_x:.1f}, {self.pos_y:.1f}, {self.pos_z:.1f}] | "
                f"Yaw: {math.degrees(self.yaw):.1f}°"
            )

        if self.motors_on:
            self.offboard_setpoint_counter += 1
        else:
            self.offboard_setpoint_counter = 0

    def update_position_setpoint(self):
        if not self.motors_on:
            return

        dt = 0.05

        with self._lock:
            keys = set(self.active_keys)

        if 'w' in keys:
            self.pos_x += math.cos(self.yaw) * self.linear_speed * dt
            self.pos_y += math.sin(self.yaw) * self.linear_speed * dt
        elif 's' in keys:
            self.pos_x -= math.cos(self.yaw) * self.linear_speed * dt
            self.pos_y -= math.sin(self.yaw) * self.linear_speed * dt

        if 'r' in keys:
            self.pos_z -= self.linear_speed * dt
        elif 'f' in keys:
            self.pos_z += self.linear_speed * dt

        if 'a' in keys:
            self.yaw = self._normalize_angle(self.yaw - self.angular_speed * dt)
        elif 'd' in keys:
            self.yaw = self._normalize_angle(self.yaw + self.angular_speed * dt)

        self.pos_z = max(-50.0, min(self.pos_z, -0.5))

    # ------------------------------------------------------------------
    # pygame HUD renderer
    # ------------------------------------------------------------------

    def draw_hud(self, screen, font_big, font_small):
        screen.fill(DGRAY)

        # Title bar
        pygame.draw.rect(screen, GRAY, (0, 0, WIN_W, 40))
        title = font_big.render("PX4 Teleop Controller", True, WHITE)
        screen.blit(title, (WIN_W // 2 - title.get_width() // 2, 8))

        # Status row
        arm_col = GREEN if self.armed         else RED
        off_col = GREEN if self.offboard_mode else RED
        mot_col = GREEN if self.motors_on     else RED
        screen.blit(font_small.render(f"ARMED: {'YES' if self.armed else 'NO'}",             True, arm_col), (10, 50))
        screen.blit(font_small.render(f"OFFBOARD: {'YES' if self.offboard_mode else 'NO'}", True, off_col), (10, 72))
        screen.blit(font_small.render(f"MOTORS: {'ON' if self.motors_on else 'OFF'}",        True, mot_col), (10, 94))

        # Position / yaw
        screen.blit(font_small.render(
            f"Pos  N:{self.pos_x:+.1f}  E:{self.pos_y:+.1f}  Alt:{-self.pos_z:.1f} m", True, WHITE),  (10, 125))
        screen.blit(font_small.render(
            f"Yaw  {math.degrees(self.yaw):+.1f}°", True, YELLOW), (10, 147))

        # Active keys
        key_str = "Active keys: " + " ".join(sorted(self.active_keys)) if self.active_keys else "Active keys: —"
        screen.blit(font_small.render(key_str, True, YELLOW), (10, 175))

        # Key reference
        pygame.draw.line(screen, GRAY, (0, 200), (WIN_W, 200), 1)
        for i, line in enumerate([
            "SPACE  Toggle motors",
            "W / S  Forward / Backward",
            "A / D  Continuous yaw left / right",
            "Q / E  Snap yaw ±90° + forward",
            "R / F  Up / Down",
            "ESC    Emergency disarm & quit",
        ]):
            screen.blit(font_small.render(line, True, RED), (10, 208 + i * 18))

        pygame.display.flip()

    # ------------------------------------------------------------------
    # PX4 command helpers
    # ------------------------------------------------------------------

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        msg.position     = [self.pos_x, self.pos_y, self.pos_z]
        msg.velocity     = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw          = self.yaw
        msg.yawspeed     = float('nan')
        self.ts_pub.publish(msg)

    def engage_offboard_mode(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1    = 1.0
        msg.param2    = 6.0
        msg.command   = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = msg.target_component = msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def arm_force(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1    = 1.0
        msg.param2    = 21196.0
        msg.command   = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = msg.target_component = msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def disarm(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1    = 0.0
        msg.command   = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = msg.target_component = msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    pygame.init()
    screen     = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("PX4 Teleop — click window to focus")
    font_big   = pygame.font.SysFont("monospace", 16, bold=True)
    font_small = pygame.font.SysFont("monospace", 13)
    clock      = pygame.time.Clock()

    controller = TeleopNode()

    try:
        while rclpy.ok() and controller._running:
            controller.process_pygame_events()
            controller.draw_hud(screen, font_big, font_small)
            clock.tick(30)

    except KeyboardInterrupt:
        pass

    finally:
        controller.motors_on = False
        controller.disarm()
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        pygame.quit()

if __name__ == "__main__":
    main()