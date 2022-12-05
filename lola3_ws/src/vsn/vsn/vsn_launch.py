import rclpy

from .service.client import Client
from .models.ia import Ia
from .image.image_sub import ImageSubscriber


def main(args=None):
    rclpy.init(args=args)

    client = Client()
    recv_image = ImageSubscriber()
    ia = Ia()

    while 1:
        # Cambiar tipo de clase
        image = recv_image.get_image_rgb()
        print(type(image))
        if image is not None:

            # Ia models
            action, distance = ia.random()

            server_response = client.send_action_request(action, distance)

            if not server_response:
                client.get_logger().info("An error has been detected from the server")

            if action == "stop":
                client.get_logger().info("Robot reached the goal satisfactorily")
                break

    client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
