import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

class LidarTest(Node):
    def __init__(self):
        super().__init__("lidar_test")

        self._lidar_test_publisher = self.create_publisher(LaserScan, "LaserScan", 10)
        self._timer = self.create_timer(0.05, self._publish_callback)

    def _publish_callback(self):
        scann = LaserScan()
        header = Header()
        header.frame_id = 'laser_frame'
        scann.header = header


        scann.angle_min = -3.1415
        scann.angle_max = 3.1415
        scann.angle_increment = 0.00311202858575

        scann.time_increment = 4.99999987369e-05

        scann.range_min = 0.
        scann.range_max = 1.

        scann.ranges = [0.01, 0.44]
        scann.intensities = [0.14, 0.44]

        self._lidar_test_publisher.publish(scann)


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

    # Create the object for base_robot
    lidar_test = LidarTest()

    # Execute the node until CTRl+C or CTRl+Z
    rclpy.spin(lidar_test)

    # Delete the node before to finish the running
    # when press CTRl+C or CTRl+Z
    lidar_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()