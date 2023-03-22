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
        """
        Initialize the MovementController class and set up ROS node, publishers, and subscribers.
        """
        # Call superclass constructor
        super().__init__('movement_controller')

        # Initialize variables for odometry data and thread locking
        self.odometry = None
        self._angle_target = 0
        self.lock = threading.Lock()

        # Set up QoS profile for communication
        qos = QoSProfile(depth=10)

        # Create publisher for Twist messages on 'cmd_vel' topic
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos)

        # Create subscription for Odometry messages on 'odom' topic and attach callback
        self.odometry_subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, qos)

        # Create thread for subscription processing
        self.subscription_thread = threading.Thread(target=self.spin_thread)
        self.subscription_thread.start()

        # Initialize differential error
        self.differential_error = 0

    def spin_thread(self):
        """
        Thread function to process incoming messages and callbacks.
        """
        rclpy.spin(self)

    def odometry_callback(self, msg):
        """
        Odometry callback function to update odometry data.
        :param msg: Odometry message received from the 'odom' topic.
        """
        with self.lock:
            self.odometry = msg

    def get_odometry(self):
        """
        Get the latest odometry data.
        :return: Latest Odometry data.
        """
        with self.lock:
            return self.odometry

    def get_position_yaw(self, odometry):
        """
        Extract the position and yaw from the odometry data.
        :param odometry: Odometry data.
        :return: tuple (x, y, yaw) of the robot's position and yaw.
        """
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        quaternion = (
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w,
        )
        _, _, yaw = self.quaternion_to_euler(*quaternion)

        return x, y, yaw

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to Euler angles.
        :param x: Quaternion x component.
        :param y: Quaternion y component.
        :param z: Quaternion z component.
        :param w: Quaternion w component.
        :return: tuple (roll, pitch, yaw) of the Euler angles.
        """
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

        return roll, pitch, yaw

    def move_forward(self, distance, speed):
        """
        Move the robot forward a specified distance at a specified speed.
        :param distance: Distance to move forward.
        :param speed: Speed at which to move forward.
        """
        # Log message to indicate that robot is moving forward
        self.get_logger().warning("Moving forward")

        # Check that speed is positive
        if speed <= 0:
            raise ValueError("Speed must be positive")

        # Initialize Twist message and set linear velocity
        twist = Twist()
        twist.linear.x = speed

        # Call move_distance function with input parameters
        self.move_distance(distance, twist)


    def move_backward(self, distance, speed):
        """
        Move the robot backward a specified distance at a specified speed.
        :param distance: Distance to move backward.
        :param speed: Speed at which to move backward.
        """
        # Check that speed is positive
        if speed <= 0:
            raise ValueError("Speed must be positive")

        # Log message to indicate that robot is moving backward
        self.get_logger().warning("Moving backward")

        # Initialize Twist message and set linear velocity
        twist = Twist()
        twist.linear.x = -speed

        # Call move_distance function with input parameters
        self.move_distance(distance, twist)


    def turn_left(self, angle, angular_speed):
        """
        Turn the robot left a specified angle at a specified angular speed.
        :param angle: Angle to turn left (in degrees).
        :param angular_speed: Angular speed at which to turn left.
        """
        # Check that angular speed is positive
        if angular_speed <= 0:
            raise ValueError("Angular speed must be positive")

        # Log message to indicate that robot is turning left
        self.get_logger().warning("Moving left")

        # Initialize Twist message and set angular velocity
        twist = Twist()
        twist.angular.z = angular_speed

        # Call move_angle function with input parameters
        angle_rad = math.radians(angle)
        self.move_angle(angle_rad, twist)


    def turn_right(self, angle, angular_speed):
        """
        Turn the robot right a specified angle at a specified angular speed.
        :param angle: Angle to turn right (in degrees).
        :param angular_speed: Angular speed at which to turn right.
        """
        # Check that angular speed is positive
        if angular_speed <= 0:
            raise ValueError("Angular speed must be positive")

        # Log message to indicate that robot is turning right
        self.get_logger().warning("Moving right")

        # Initialize Twist message and set angular velocity
        twist = Twist()
        twist.angular.z = -angular_speed

        # Call move_angle function with input parameters
        angle_rad = math.radians(angle)
        self.move_angle(angle_rad, twist)

    def move_distance(self, distance, twist):
        """
        Move the robot forward or backward a specified distance at a specified twist (linear and angular velocity).
        :param distance: Distance to move (in meters).
        :param twist: Twist message with linear and angular velocity set.
        """

        # Wait until odometry data is available
        while self.get_odometry() is None:
            self.get_logger().warning("Waiting for odometry data")
            time.sleep(0.1)

        # Get initial odometry data and robot position
        start_odometry = self.get_odometry()
        start_position = start_odometry.pose.pose.position
        epsilon = 0.005

        # Move the robot forward until the desired distance is covered
        while True:

            # Get current odometry data and robot position
            current_odometry = self.get_odometry()
            current_position = current_odometry.pose.pose.position

            # Calculate the distance the robot has moved since the start of the function
            moved_distance = ((current_position.x - start_position.x) ** 2 +
                              (current_position.y - start_position.y) ** 2) ** 0.5

            error = distance - moved_distance
            if error > epsilon:
                # Publish Twist message to command velocity publisher
                self.cmd_vel_publisher.publish(twist)
            else:
                break
        # Stop the robot
        self.stop()

    def move_angle(self, angle, twist):
        """
        Turn the robot left or right a specified angle at a specified twist (linear and angular velocity).
        :param angle: Angle to turn (in radians).
        :param twist: Twist message with linear and angular velocity set.
        """
        # Wait until odometry data is available
        start_odometry = self.get_odometry()
        while start_odometry is None:
            self.get_logger().warning("Odometry data is not available yet")
            time.sleep(0.1)

        # Get initial robot yaw angle
        _, _, start_yaw = self.get_position_yaw(start_odometry)
        self.get_logger().warning(f"start: {start_yaw}")

        epsilon = 0.08

        # Calculate target angle
        angle_deg = math.degrees(angle)
        self._angle_target = self._angle_target + angle_deg

        while True:
            # Get current robot yaw angle
            current_odometry = self.get_odometry()
            _, _, current_yaw = self.get_position_yaw(current_odometry)

            # Calculate the angle error
            error = self._angle_target - current_yaw
            error = (error + math.pi) % (2 * math.pi) - math.pi
            #self.get_logger().warning(f"error: {error}")
            if abs(error) > epsilon:
                self.cmd_vel_publisher.publish(twist)
            else:
                break
        """
        # Turn the robot until the desired angle is covered
        while moved_angle < adjusted_angle:
            # Publish Twist message to command velocity publisher
            self.cmd_vel_publisher.publish(twist)

            # Get current robot yaw angle
            current_odometry = self.get_odometry()
            _, _, current_yaw = self.get_position_yaw(current_odometry)

            # Calculate the angle the robot has turned since the start of the function
            moved_angle = abs(current_yaw - start_yaw)

            self.differential_error = angle - moved_angle
            # Normalize the angle
            if moved_angle > math.pi:
                moved_angle = 2 * math.pi - moved_angle
        """
        # Stop the robot
        self.stop()

    def _calculate_angle(self, angle_current: float) -> float:
        """
        Calculate the difference between the angle_current and the self._angle_target
        :param angle_current: float angle (radians)
        :return: difference in radians
        """
        dif = (self._angle_target * math.pi / 180 - angle_current) % (360 * math.pi / 180)
        if dif < math.pi:
            return dif
        else:
            return 2 * math.pi - dif

    def stop(self):
        """
        Stop the robot's motion by publishing a Twist message with zero velocity.
        """
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)

    def destroy(self):
        """
        Stops the robot's movement and releases all resources.
        """
        # Stop the robot's movement
        self.stop()

        # Destroy publisher and subscription to free resources
        self.destroy_publisher(self.cmd_vel_publisher)
        self.destroy_subscription(self.odometry_subscription)
