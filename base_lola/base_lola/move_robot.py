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
        self.x = 0.0
        self.y = 0.0
        self.current_yaw = 0.0
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
        self.get_logger().info("Starting subscription thread")
        rclpy.spin(self)

    def odometry_callback(self, msg):
        """
        Odometry callback function to update odometry data.
        :param msg: Odometry message received from the 'odom' topic.
        """
        with self.lock:
            self.odometry = msg
            self.update_position(self.odometry)


    def update_position (self, odometry):
        """
        Update the position and yaw from the odometry data.
        :param odometry: Odometry data.
        """
        self.x = odometry.pose.pose.position.x
        self.y = odometry.pose.pose.position.y
        quaternion = (
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w,
        )
        _, _, self.current_yaw = self.quaternion_to_euler(*quaternion)


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
        dir = 1
        # Check that angular speed is positive
        if angular_speed <= 0:
            raise ValueError("Angular speed must be positive")

        # Log message to indicate that robot is turning left
        self.get_logger().warning("Moving left")

        # Initialize Twist message and set angular velocity
        twist = Twist()
        twist.angular.z = angular_speed

        # Call move_angle function with input parameters
        self.move_angle(angle, twist, dir)


    def turn_right(self, angle, angular_speed):
        """
        Turn the robot right a specified angle at a specified angular speed.
        :param angle: Angle to turn right (in degrees).
        :param angular_speed: Angular speed at which to turn right.
        """
        dir = -1
        # Check that angular speed is positive
        if angular_speed <= 0:
            raise ValueError("Angular speed must be positive")

        # Log message to indicate that robot is turning right
        self.get_logger().warning("Moving right")

        # Initialize Twist message and set angular velocity
        twist = Twist()
        twist.angular.z = -angular_speed

        # Call move_angle function with input parameters
        self.move_angle(angle, twist, dir)

    def move_distance(self, distance, twist):
        """
        Move the robot forward or backward a specified distance at a specified twist (linear and angular velocity).
        :param distance: Distance to move (in meters).
        :param twist: Twist message with linear and angular velocity set.
        """

        # Wait until odometry data is available
        while self.odometry is None:
            self.get_logger().warning("Waiting for odometry data")
            time.sleep(0.1)


        # Get initial odometry data and robot position
        start_position = self.x, self.y
        epsilon = 0.005

        # Move the robot forward until the desired distance is covered
        while True:

            # Calculate the distance the robot has moved since the start of the function
            moved_distance = ((self.x - start_position[0]) ** 2 +
                              (self.y - start_position[1]) ** 2) ** 0.5

            error = distance - moved_distance
            if error > epsilon:
                # Publish Twist message to command velocity publisher
                self.cmd_vel_publisher.publish(twist)
            else:
                break
        # Stop the robot
        self.stop()

    def move_angle(self, turn_angle, twist, dire):
        """
        Set the angular velocity until final angle is reached
        :param dire: 1 -> Left or -1 -> Right
        :param turn_angle: int angle of rotation
        """
        # Wait until odometry data is available
        while self.odometry is None:
            self.get_logger().warning("Odometry data is not available yet")
            time.sleep(0.1)

        self._update_angle(dire * turn_angle)
        epsilon = .08


        # Turn left
        if dire == 1:
            while self._calculate_angle(self.current_yaw) > epsilon and rclpy.ok():

                self.cmd_vel_publisher.publish(twist)
                #time.sleep(0.1)

        # Turn right
        if dire == -1:
            while self._calculate_angle(self.current_yaw) > epsilon and rclpy.ok():
                self.get_logger().warning(f"Current yaw: {self.current_yaw}")
                self.cmd_vel_publisher.publish(twist)
                #time.sleep(0.1)

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

    def _update_angle(self, angle: int) -> None:
        """
        Calculate the final angle, by adding an angle in degrees
        :param angle: int angle (degrees)
        """
        self._angle_target = (self._angle_target + angle) % 360

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
