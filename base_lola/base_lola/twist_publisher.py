import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MiPublicador(Node):
    def __init__(self):
        super().__init__('mi_publicador')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.publicar_velocidad)

    def publicar_velocidad(self):
        msg = Twist()
        msg.linear.x = 0.7  # velocidad lineal en metros por segundo
        msg.angular.z = 0.0  # ángulo de giro en radianes por segundo
        self.publisher_.publish(msg)
        self.get_logger().info('Publicando velocidad lineal {:.2f} m/s y ángulo de giro {:.2f} rad/s'.format(msg.linear.x, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    mi_publicador = MiPublicador()
    rclpy.spin(mi_publicador)
    mi_publicador.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
