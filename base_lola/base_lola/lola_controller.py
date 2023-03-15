import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion


class OdometryCalculator(Node):
    def __init__(self):
        super().__init__('odometry_calculator')
        self.get_logger().info(f"Odometry calculator is running")
        self.subscription = self.create_subscription(JointState, 'wheel_state', self.callback, 1)


        self.publisher = self.create_publisher(JointState, 'cmd_wheel', 10)
        self.timer = self.create_timer(0.1, self.publisher_callback)

        self.subscription_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)


        self.velocidad_lineal = 0
        self.angulo = 0 #rad/s
        self._kdato = 36.28
        self.prev_time = None

        # Inicializar la posición y orientación del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Distancia entre las dos ruedas del robot (en metros)
        self.L = 0.536

    def cmd_vel_callback(self, msg):

        self.velocidad_lineal = msg.linear.x
        self.get_logger().info(f"La velocidad lineal recibida por cmd_vel: {self.velocidad_lineal}")
        self.angulo = msg.angular.z #rad/s

        self.get_logger().info(f"El angulo recibido por cmd_vel: {self.angulo}")

    def calcular_velocidad_angular_ruedas(self):

        #self.velocidad_lineal = 1
        #self.angulo = -90


        # Calcular la velocidad angular de cada rueda
        velocidad_angular_rueda_izquierda = (self.velocidad_lineal - (self.angulo * self.L / 2)) / (self.L / 2)
        velocidad_angular_rueda_derecha = (self.velocidad_lineal + (self.angulo * self.L / 2)) / (self.L / 2)

        # Devolver las velocidades angulares de las ruedas como una tupla
        return (velocidad_angular_rueda_izquierda, velocidad_angular_rueda_derecha)

    def publisher_callback(self):

        # Obtener la velocidad de las ruedas derecha e izquierda (en radianes por segundo)

        vel_left, vel_right = self.calcular_velocidad_angular_ruedas()
        self.get_logger().info(f"The desire wheel speed L: {vel_left} y R:{vel_right} ")


        # Extract the desired angular velocities from the JointState message


        # Convert the angular velocities from rad/s to an integer data value in the range 0-127
        datod = int(round(vel_right * self._kdato))
        datoi = int(round(vel_left * self._kdato))

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

        # Crear un nuevo mensaje JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()  # Establecer la marca de tiempo
        msg.name = ['RIGHT', 'LEFT']  # Establecer los nombres de las articulaciones
        msg.velocity = [float(datod), float(datoi)]  # Establecer las velocidades de las ruedas

        # Publicar el mensaje en el tópico 'wheel_state'
        self.publisher.publish(msg)

    def callback(self, msg):

        # Buscar los índices de las ruedas izquierda y derecha en el mensaje JointState
        try:
            index_r = msg.name.index('RIGHT')
            index_l = msg.name.index('LEFT')
        except ValueError:
            self.get_logger().warn('No se encontraron las ruedas en el mensaje JointState')
            return

        # Obtener las posiciones y velocidades angulares de las ruedas del mensaje JointState
        pos_r = msg.position[index_r]
        pos_l = msg.position[index_l]
        vel_r = msg.velocity[index_r]
        vel_l = msg.velocity[index_l]
        '''
        current_time = self.get_clock().now().to_msg()
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = (current_time.sec - self.prev_time.sec) + (current_time.nanosec - self.prev_time.nanosec) * 1e-9
        self.prev_time = current_time

        self.get_logger().info(f"DT:{dt}")
        '''
        dt = 1/6
        # Cálculo de las velocidades lineales de las dos ruedas (en metros por segundo)
        v_r = vel_r * 0.095 # Se multiplica por el radio
        v_l = vel_l * 0.095 # Se multiplica por el radio

        # Cálculo de la odometría
        delta_d = (v_r + v_l) / 2 * dt
        delta_theta = (v_r - v_l) / self.L * dt

        # Actualizar la posición y orientación del robot
        self.x += delta_d * math.cos(self.theta)
        self.y += delta_d * math.sin(self.theta)
        self.theta += delta_theta

        self.get_logger().info(f"x:{self.x}")
        self.get_logger().info(f"y:{self.y}")
        self.get_logger().info(f"theta:{self.theta}")

        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

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
        self.get_logger().info(f"PUBLICANDO ODOMETRia .....")
        self.publisher_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    odometry_calculator = OdometryCalculator()
    rclpy.spin(odometry_calculator)
    odometry_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
