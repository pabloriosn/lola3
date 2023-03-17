import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdometryHandler:
    def __init__(self):
        self._x = None
        self._y = None
        self._angle = None

    def update_data(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation

        # Convert quaternion to Euler angle (roll, pitch, yaw)
        _, _, yaw = self.quaternion_to_euler(orientation)
        self._angle = yaw

    def get_position(self):
        return self._x, self._y

    def get_angle(self):
        return self._angle

    def quaternion_to_euler(self, orientation):
        q_x = orientation.x
        q_y = orientation.y
        q_z = orientation.z
        q_w = orientation.w

        roll = math.atan2(2 * (q_w * q_x + q_y * q_z),
                          1 - 2 * (q_x**2 + q_y**2))
        pitch = math.asin(2 * (q_w * q_y - q_z * q_x))
        yaw = math.atan2(2 * (q_w * q_z + q_x * q_y),
                         1 - 2 * (q_y**2 + q_z**2))

        return roll, pitch, yaw

class OdometrySubscriber(Node):

    def __init__(self):
        super().__init__('odometry_subscriber')
        self.odometry_handler = OdometryHandler()
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        self.odometry_handler.update_data(msg)

def main(args=None):
    rclpy.init(args=args)

    odom_subscriber = OdometrySubscriber()

    rclpy.spin(odom_subscriber)

    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
