import rclpy
from rclpy.node import Node

from custom_service.srv import ActionRobot


class Client(Node):

    def __init__(self, service_name: str = "Action_service") -> None:
        # Initialize the node
        super().__init__('Vsn_client')

        # Create the client
        self.cli = self.create_client(ActionRobot, service_name)

        # Check is the service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service down, waiting again...')
        self.req = ActionRobot.Request()

    def send_action_request(self, action: str, distance: str) -> bool:
        self.req.action = action
        self.req.distance = distance
        self.get_logger().info(f'Action: {self.req.action} and {self.req.distance}')

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
