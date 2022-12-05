from rclpy.node import Node

from custom_service.srv import ActionRobot
from ..action_robot.move_robot import MoveRobot


class Service(Node):

    def __init__(self, service_name: str = "Action_service") -> None:
        # Initialize the node
        super().__init__('Server_node')

        # Create service and configure
        self._srv = self.create_service(ActionRobot, service_name, self._launch_action_callback)
        self.get_logger().info("Service up, waiting action")

        # Define action that can performance the robot
        self.robot = MoveRobot()
        self._action_dict = {
            'forward': self.robot.move_forward,
            'backward': self.robot.move_backward,
            'right': self.robot.turn_right,
            'left': self.robot.turn_left,
            'stop': self.robot.stop_robot
        }

    def _launch_action_callback(self, request, response) -> bool:
        self._action = request.action
        self._distance = request.distance

        try:
            self._action_dict[self._action](self._distance)
            response.success = True
            return response

        except KeyError:
            self.get_logger().info("Client is sending wrong action, robot will stop")
            self._action_dict['stop']()
            response.success = False
            return response
