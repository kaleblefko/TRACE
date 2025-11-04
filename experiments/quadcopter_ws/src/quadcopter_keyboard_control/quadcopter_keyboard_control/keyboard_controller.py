import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import sys

class QuadcopterKeyboardController(Node):
    def __init__(self):
        super().__init__('quadcopter_keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, '/X3/gazebo/command/twist', 10)

        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 1.0)
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        self.active_keys = set()
        self.motors_on = False

        self.timer = self.create_timer(0.02, self.publish_twist_command)
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        self.get_logger().info('=' * 60)
        self.get_logger().info('Quadcopter Keyboard Controller')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Controls:')
        self.get_logger().info('  SPACE - Toggle motors ON/OFF')
        self.get_logger().info('  W/S - Forward/Backward')
        self.get_logger().info('  A/D - Left/Right (roll)')
        self.get_logger().info('  Q/E - Yaw left/right')
        self.get_logger().info('  R/F - INcrease/decrease altitude')
        self.get_logger().info('  ESC- Quit')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Make sure this terminal is in focus for keyboard input')
        self.get_logger().info('=' * 60)

    def on_press(self, key):
        try: 
            if hasattr(key, 'char') and key.char:
                char = key.char.lower()
                if char in ['w', 's', 'a', 'd', 'q', 'e', 'r', 'f']:
                    self.active_keys.add(char)
                    self.get_logger().info(f'Key Pressed: {char}')
            elif key == keyboard.Key.esc:
                self.get_logger().info('ESC pressed, shutting down')
                self.motors_on = False
                self.listener.stop()
                rclpy.shutdown()
            elif key == keyboard.Key.space:
                self.motors_on = not self.motors_on
                status = "ON" if self.motors_on else "OFF"
                self.get_logger().info(f'Motors {status}')
        except AttributeError:
            pass
    
    def on_release(self, key):
        try: 
            if hasattr(key, 'char') and key.char:
                char = key.char.lower()
                self.active_keys.discard(char)
        except AttributeError:
            pass
    
    def make_twist_command(self):
        '''
        Calculate motor speeds based on input
        Motor layout (X configuration):

         Front
        2      0
         \\ x //
         // x \\
        1       3
          Back 
        Motor indicies: [0:FR, 1:BL, 2:FL, 3:BR]
        '''
        msg = Twist()
        if not self.motors_on:
            return msg

        if 'w' in self.active_keys:
            msg.linear.x = self.linear_speed
        elif 's' in self.active_keys:
            msg.linear.x = -self.linear_speed
        
        if 'a' in self.active_keys:
            msg.linear.y = self.linear_speed
        elif 'd' in self.active_keys:
            msg.linear.y = -self.linear_speed

        if 'r' in self.active_keys:
            msg.linear.z = self.linear_speed
        elif 'f' in self.active_keys:
            msg.linear.z = -self.linear_speed
        
        if 'q' in self.active_keys:
            msg.angular.z = self.angular_speed
        elif 'e' in self.active_keys:
            msg.angular.z = -self.angular_speed
        
        return msg

    def publish_twist_command(self):
        msg = self.make_twist_command()
        self.publisher_.publish(msg)

        if self.motors_on and any([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]):
            self.get_logger().info(
                f"Linear: [{msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}] "
                f"Angular: [{msg.angular.z:.2f}]",
                throttle_duration_sec=0.5
            )

def main(args=None):
    rclpy.init(args=args)
    controller = QuadcopterKeyboardController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.motors_on = False
        controller.publish_twist_command()
        controller.listener.stop()
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()