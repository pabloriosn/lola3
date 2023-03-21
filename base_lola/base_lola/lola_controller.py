import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion


class OdometryCalculator(Node):
    def __init__(self):
        """
        Initialize the OdometryCalculator node with necessary subscribers, publishers, and variables.
        """
        # Call superclass constructor and initialize node
        super().__init__('odometry_calculator')
        # Log message to indicate node is running
        self.get_logger().info(f"Odometry calculator is running")

        # Initialize JointState subscriber
        self.subscription = self.create_subscription(JointState, 'wheel_state', self.callback, 1)

        # Initialize JointState publisher
        self.publisher = self.create_publisher(JointState, 'cmd_wheel', 10)

        # Initialize timer for JointState publisher callback
        self.timer = self.create_timer(0.1, self.publisher_callback)

        # Initialize Twist subscriber
        self.subscription_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Initialize Odometry publisher
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)

        # Initialize linear and angular velocities, speed constant, and time variables
        self.velocidad_lineal = 0
        self.angulo = 0  # rad/s
        self._kdato = 36.28
        self.prev_time = None

        # Initialize position and orientation variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Initialize wheel distance constant
        self.L = 0.536

    def cmd_vel_callback(self, msg):
        """
        Update linear and angular velocities from the Twist message.
        :param msg: Twist message with linear and angular velocities.
        """
        # Update linear and angular velocities based on Twist message input
        self.velocidad_lineal = msg.linear.x
        self.angulo = msg.angular.z  # rad/s

    def calcular_velocidad_angular_ruedas(self):
        """
        Calculate angular velocities for each wheel based on linear and angular velocities.
        :return: tuple (velocidad_angular_rueda_izquierda, velocidad_angular_rueda_derecha) of wheel angular velocities.
        """
        # Calculate angular velocities for each wheel based on linear and angular velocities
        velocidad_angular_rueda_izquierda = (self.velocidad_lineal - (self.angulo * self.L / 2)) / (self.L / 2)
        velocidad_angular_rueda_derecha = (self.velocidad_lineal + (self.angulo * self.L / 2)) / (self.L / 2)

        # Return tuple containing angular velocities for each wheel
        return (velocidad_angular_rueda_izquierda, velocidad_angular_rueda_derecha)

    def publisher_callback(self):
        """
        Publish calculated wheel speeds as a JointState message to the 'cmd_wheel' topic.
        """
        # Get the angular velocity of the right and left wheels (in radians per second)

        vel_left, vel_right = self.calcular_velocidad_angular_ruedas()
        self.get_logger().info(f"The desire wheel speed L: {vel_left} y R:{vel_right} ")

        # Convert the angular velocities from rad/s to an integer data value in the range 0-127

        datod = int(round(vel_right * self._kdato))
        datoi = int(round(vel_left * self._kdato))

        # Apply limits to the data values

        if datod == 0:
            datod = 0
        elif datod > 0:
            datod = max(min(datod, 127), 10)
        else:
            datod = min(max(datod, -127), -10)

        if datoi == 0:
            datoi = 0
        elif datoi > 0:
            datoi = max(min(datoi, 127), 10)
        else:
            datoi = min(max(datoi, -127), -10)

        # Add an offset of 256 to the data value if it is negative

        datod = 256 + datod if datod < 0 else datod
        datoi = 256 + datoi if datoi < 0 else datoi

        # Create a new JointState message

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
        msg.name = ['RIGHT', 'LEFT']  # Set the joint names
        msg.velocity = [float(datod), float(datoi)]  # Set the wheel speeds

        # Publish the message on the 'wheel_state' topic

        self.publisher.publish(msg)

    def callback(self, msg):
        """
        Update odometry based on JointState message and publish as an Odometry message to the 'odom' topic.
        :param msg: JointState message with wheel position and angular velocity.
        """
        # Find the indices of the left and right wheels in the JointState message

        try:
            index_r = msg.name.index('RIGHT')
            index_l = msg.name.index('LEFT')
        except ValueError:
            self.get_logger().warn('Wheels not found in JointState message')
            return

        # Get the position and angular velocity of the wheels from the JointState message

        pos_r = msg.position[index_r]
        pos_l = msg.position[index_l]
        vel_r = msg.velocity[index_r]
        vel_l = msg.velocity[index_l]

        # Calculate de dt using the current time

        current_time = self.get_clock().now().to_msg()
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = (current_time.sec - self.prev_time.sec) + (current_time.nanosec - self.prev_time.nanosec) * 1e-9
        self.prev_time = current_time

        # Calculate the linear velocities of the two wheels (in meters per second)

        v_r = vel_r * 0.095  # Multiply by the radius
        v_l = vel_l * 0.095  # Multiply by the radius

        # Calculate the odometry

        delta_d = (v_r + v_l) / 2 * dt
        delta_theta = (v_r - v_l) / self.L * dt

        # Update the position and orientation of the robot

        self.x += delta_d * math.cos(self.theta)
        self.y += delta_d * math.sin(self.theta)
        self.theta += delta_theta

        # Convert the orientation from Euler angles to a quaternion

        quat = self.euler_to_quaternion(0, 0, self.theta)

        # Create an Odometry message

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set the position of the robot
        odom.pose.pose.position = Point()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Set the orientation of the robot
        odom.pose.pose.orientation = Quaternion()
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Publish the Odometry message
        self.publisher_odom.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion representation.
        :param roll: Roll angle in radians.
        :param pitch: Pitch angle in radians.
        :param yaw: Yaw angle in radians.
        :return: tuple (qx, qy, qz, qw) of quaternion components.
        """
        # Calculate the four quaternion values using the Euler angles
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(
            pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(
            pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(
            pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(
            pitch / 2) * math.sin(yaw / 2)

        # Return the quaternion values as a tuple
        return (qx, qy, qz, qw)


def main(args=None):
    """
    Initialize ROS2 node, create OdometryCalculator instance, and spin until shutdown.
    :param args: Optional command-line arguments.
    """
    # Initialize the ROS2 node
    rclpy.init(args=args)

    # Create an instance of the OdometryCalculator class
    odometry_calculator = OdometryCalculator()

    # Spin the node until it is shutdown
    rclpy.spin(odometry_calculator)

    # Destroy the node
    odometry_calculator.destroy_node()

    # Shutdown the ROS2 client library
    rclpy.shutdown()


# Run the main function if the script is executed
if __name__ == '__main__':
    main()
