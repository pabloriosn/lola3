import rclpy
from rclpy.node import Node
import sys
import serial
import time
import struct
from datetime import datetime
from math import copysign, pi

from sensor_msgs.msg import JointState


class HWClass(Node):
    """
    Initializes the HWClass node and its parameters. It creates a publisher for wheel state information and
    a subscriber for wheel velocity commands. It opens a serial connection with the Arduino board, reads the
    current encoder values and discards the first line of data from the Arduino board.
    """
    def __init__(self) -> None:

        super().__init__('hwinterface')

        self.declare_parameter('device_ard', 'NULL')
        self.declare_parameter('baudrate', 0)
        self.declare_parameter('timer_publisher', 0)
        self.declare_parameter('name_pub', 'NULL')
        self.declare_parameter('name_sub', 'NULL')


        self.device_ard = self.get_parameter('device_ard').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').value
        self._timer_publisher = 1/self.get_parameter('timer_publisher').value

        self._name_pub = 'wheel_state'
        self._name_sub = 'cmd_wheel'
        self.lastTwistTime = 0
        self._last_time = 0
        self._timeout = 5

        self._pasos_vuelta = 356.3 * 2 #Numero de pasos que da el encoder para dar una vuelta completa
        self._kdato = 36.28 #representa una constante que se utiliza para convertir la velocidad angular a entero

        # Create a ROS 2 publisher for wheel state information

        self._wheel_pub = self.create_publisher(JointState, self._name_pub, 10)
        self._timer = self.create_timer(self._timer_publisher, self._publisher_callback)

        self.get_logger().info(f"Publisher {self._name_pub} running")

        # Create a ROS 2 subscriber for wheel velocity commands

        self.create_subscription(JointState, self._name_sub, self._callback_vel, 1)
        self.get_logger().info(f"Subscriber {self._name_sub} running")

        # Open a serial connection to the Arduino device
        try:
            self.arduino = serial.Serial(self.device_ard, self.baudrate)
            time.sleep(0.01)
            if self.arduino.isOpen():
                self.get_logger().info(f"Device {self.device_ard} is connected")
            else:
                self.get_logger().error(f"Failed to connect to device {self.device_ard}")
                rclpy.shutdown()
                sys.exit(1)
        except serial.SerialException:
            self.get_logger().error(f"Failed to open serial port {self.device_ard}")
            rclpy.shutdown()
            sys.exit(1)

        # Read and discard the first line of data from the Arduino device
        data = self.arduino.readline()

        # Read the current encoder values
        self.steps_enc_left, self.time_enc_left, self.steps_enc_right, self.time_enc_right = self._read_encoder()


    def _read_encoder(self):
        """
        Sends a 'N' character to the Arduino board to request encoder data. Reads the data from the serial port
        and retrieves the encoder steps and time values for both wheels. Discards the first and last lines of data
        from the Arduino board.
        """
        # Send the 'N' character to the serial port to request encoder data
        self.arduino.write(str('N').encode())


        # Read the data from the serial port
        # Read the first line, but discard the data
        self.arduino.readline()

        # Read the left encoder steps and time values
        steps_enc_left = struct.unpack('i', self.arduino.read(4))[0]
        time_enc_left = struct.unpack('i', self.arduino.read(4))[0]

        # Read the right encoder steps and time values
        steps_enc_right = struct.unpack('i', self.arduino.read(4))[0]
        time_enc_right = struct.unpack('i', self.arduino.read(4))[0]

        # Read the final line, but discard the data
        self.arduino.readline()

        return steps_enc_left, time_enc_left, steps_enc_right, time_enc_right

    def _publisher_callback(self):
        """
        Reads the encoder data from the Arduino board and calculates the elapsed time and distance travelled
        since the last measurement for the left and right wheels. Updates the last measurement values for both wheels.
        Calculates the position and velocity for both wheels and creates a new JointState message object with
        these values. Publishes the JointState message to the 'wheel_state' topic and logs the velocity information.
        """

        # Check if it is still receiving data
        self._check_alive()

        # Read the encoder data
        steps_enc_left, time_enc_left, steps_enc_right, time_enc_right = self._read_encoder()

        # Calculate the elapsed time and distance travelled since the last measurement for the right and left wheel
        dt_right = float(time_enc_right - self.time_enc_right) * (10 ** -6)
        dsteps_right = float(steps_enc_right - self.steps_enc_right)

        dt_left = float((time_enc_left - self.time_enc_left) * (10 ** -6))
        dsteps_left = float(steps_enc_left - self.steps_enc_left)

        # Update the last measurement values for the right and left wheel
        self.time_enc_right = time_enc_right
        self.steps_enc_right = steps_enc_right

        self.time_enc_left = time_enc_left
        self.steps_enc_left = steps_enc_left

        # Calculate the position and velocity for the right and left wheel
        posD = (steps_enc_right / self._pasos_vuelta) * 2 * pi
        wd = ((dsteps_right / self._pasos_vuelta) * 2 * pi) / dt_right

        posI = (steps_enc_left / self._pasos_vuelta) * 2 * pi
        wi = ((dsteps_left / self._pasos_vuelta) * 2 * pi) / dt_left

        # Create a new JointState message object
        msg = JointState()

        # Set the values of the JointState message object
        msg.name = ['LEFT', 'RIGHT']  # Set the names of the joints (LEFT and RIGHT)
        msg.position = [posI, posD]  # Set the position values in radians for the left and right wheels
        msg.velocity = [wi, wd]  # Set the velocity values in radians per second for the left and right wheels
        msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp

        # Publish the JointState message to the 'wheel_state' topic
        self._wheel_pub.publish(msg)

        # Print velocity information
        self.get_logger().info(f"The real left velocity is {wi}, and right {wd}")


    def _check_alive(self):
        """
        Checks if the time since the last twist message exceeds the timeout value. If so,
        sends a '?' character to the Arduino board to request new data and logs a message indicating that
        the robot has stopped due to not receiving data.
        """

        # Check if the time since the last twist message exceeds the timeout value
        if (datetime.now().second - self.lastTwistTime) > self._timeout:
            # Send a "?" character to the Arduino device to request new data
            self.arduino.write(str('?').encode())

            # Log a message indicating that the robot has stopped due to not receiving data
            self.get_logger().info("Parada por no recibir datos")


    def _callback_vel(self, msg):
        """
        Extracts the desired angular velocities from the JointState message and converts them from rad/s to an integer
        data value in the range 0-127. Limits the data value to the range 10-127 and adjusts
        the sign if necessary. Adds an offset of 256 to the data value if it is negative. Sends the data
        values to the Arduino board in the format 'V###' using a serial connection. Updates the last twist time
        to the current time. Logs the data values (for debugging purposes).
        :param msg: Message Twits from ROS 2
        :return
        """

        datod = msg.velocity[0]  # formated
        datoi = msg.velocity[1]  # formated

        # Log the data values (for debugging purposes)
        self.get_logger().info(f"datod: {datod} y datoi: {datoi}")

        # Send the data values to the Arduino device in the format 'V###' using a serial connection
        self.arduino.write(('V' + format(int(datod), '03d') + format(int(datoi), '03d')).encode())

        # Update the last twist time to the current time
        self.lastTwistTime = datetime.now().second

def main(args=None):
    """
    Initializes the ROS2 system with the given arguments. Creates an instance of the HWClass node implementation,
    starts the ROS2 event loop and waits for it to stop. Cleans up the node resources when the event loop is stopped,
    and shuts down the ROS2 system.
    """
    # Initialize the ROS2 system with the given arguments
    rclpy.init(args=args)

    # Create an instance of the HwClass node implementation
    minimal_subscriber = HWClass()

    # Start the ROS2 event loop and wait for it to stop
    rclpy.spin(minimal_subscriber)

    # Clean up the node resources when the event loop is stopped
    minimal_subscriber.destroy_node()

    # Shut down the ROS2 system
    rclpy.shutdown()


if __name__ == '__main__':
    main()
