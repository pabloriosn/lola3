import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import time
import math

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')

        self.odometry = None
        self.lock = threading.Lock()
        qos = QoSProfile(depth=10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos)
        self.odometry_subscription = self.create_subscription(
            Odometry, 'odom', self.odometry_callback, qos)
        self.subscription_thread = threading.Thread(target=self.spin_thread)
        self.subscription_thread.start()

        self.differential_error = 0  # Add differential error variable


    def spin_thread(self):
        rclpy.spin(self)

    def odometry_callback(self, msg):
        with self.lock:
            self.odometry = msg
            #self.get_logger().warning(f"Mensaje recibido: {msg}")

    def get_odometry(self):
        with self.lock:
            return self.odometry

    def get_position_yaw(self, odometry):
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        quaternion = (
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w,
        )
        _, _, yaw = self.quaternion_to_euler(*quaternion)
        #self.get_logger().warning(f"yaw en get position: {yaw}")

        return x, y, yaw

    def quaternion_to_euler(self, x, y, z, w):
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        #self.get_logger().warning(f"yaw en quaternion to euler: {yaw}")

        return roll, pitch, yaw

    def move_forward(self, distance, speed):
        self.get_logger().warning("Moving forward")
        if speed <= 0:
            raise ValueError("Speed must be positive")

        twist = Twist()
        twist.linear.x = speed
        self.move_distance(distance, twist)

    def move_backward(self, distance, speed):
        if speed <= 0:
            raise ValueError("Speed must be positive")
        self.get_logger().warning("Moving backward")
        twist = Twist()
        twist.linear.x = -speed
        self.move_distance(distance, twist)

    def turn_left(self, angle, angular_speed):
        if angular_speed <= 0:
            raise ValueError("Angular speed must be positive")

        self.get_logger().warning("Moving left")
        twist = Twist()
        twist.angular.z = angular_speed
        self.move_angle(angle, twist)

    def turn_right(self, angle, angular_speed):
        if angular_speed <= 0:
            raise ValueError("Angular speed must be positive")
        self.get_logger().warning("Moving right")
        twist = Twist()
        twist.angular.z = -angular_speed
        self.move_angle(angle, twist)

    def move_distance(self, distance, twist):
        while self.get_odometry() is None:
            self.get_logger().warning("Waiting for odometry data")
            time.sleep(0.1)

        start_odometry = self.get_odometry()
        start_position = start_odometry.pose.pose.position
        moved_distance = 0

        while moved_distance < distance:
            # Calculate error correction
            error_correction = self.differential_error * 0.1

            # Apply error correction to linear and angular velocities
            twist.linear.x += error_correction
            twist.angular.z += error_correction

            self.cmd_vel_publisher.publish(twist)
            current_odometry = self.get_odometry()
            current_position = current_odometry.pose.pose.position
            self.get_logger().warning(f"current: {current_position}")
            moved_distance = ((current_position.x - start_position.x) ** 2 +
                              (current_position.y - start_position.y) ** 2) ** 0.5

            # Update differential error
            self.differential_error = distance - moved_distance

        self.stop()

    def move_angle(self, angle, twist):
        start_odometry = self.get_odometry()
        while start_odometry is None:
            self.get_logger().warning("Odometry data is not available yet")
            time.sleep(0.1)

        _, _, start_yaw = self.get_position_yaw(start_odometry)
        self.get_logger().warning(f"start: {start_yaw}")
        moved_angle = 0

        while moved_angle < angle:
            # Calculate error correction
            error_correction = self.differential_error * 0.1

            # Apply error correction only to angular velocity
            twist.angular.z += error_correction

            self.cmd_vel_publisher.publish(twist)
            current_odometry = self.get_odometry()
            _, _, current_yaw = self.get_position_yaw(current_odometry)
            self.get_logger().warning(f"current: {current_yaw}")
            moved_angle = abs(current_yaw - start_yaw)

            if moved_angle > math.pi:
                moved_angle = 2 * math.pi - moved_angle

            # Update differential error
            self.differential_error = angle - moved_angle

        self.stop()

    def stop(self):
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)

    def destroy(self):
        self.destroy_publisher(self.cmd_vel_publisher)
        self.destroy_subscription(self.odometry_subscription)
