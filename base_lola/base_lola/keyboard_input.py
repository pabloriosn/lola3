import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.moveBindings = {
            'w': (1, 0),
            's': (-1, 0),
            'a': (0, 1),
            'd': (0, -1),
        }
        self.speed = 0.5
        self.turn = 0.5
        self.twist = Twist()

        self.running = True
        if sys.stdin.isatty():
            tty.setraw(sys.stdin.fileno())
        else:
            print("Unable to set terminal to raw mode, exiting.")
            self.running = False
        while self.running:
            self.check_keys()

        if sys.stdin.isatty():
            termios.tcsetattr(sys.stdin, termios.TCSAFLUSH, self.old_settings)

    def check_keys(self):
        if select.select([sys.stdin], [], [], 0.1)[0]:
            k = sys.stdin.read(1)
            if k in self.moveBindings.keys():
                x, z = self.moveBindings[k]
                self.twist.linear.x = x * self.speed
                self.twist.angular.z = z * self.turn
                self.publisher_.publish(self.twist)
            elif k == 'q':
                self.running = False
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    teleop_twist_keyboard = TeleopTwistKeyboard()

if __name__ == '__main__':
    main()
