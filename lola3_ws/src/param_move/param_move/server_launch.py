import rclpy
from .service.server import Server


def main(args=None) -> None:
    rclpy.init(args=args)

    service = Server()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
