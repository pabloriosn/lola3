from rclpy.node import Node


class MoveRobot(Node):
    """
    Class to control the robot with parameterized movements
    """
    def __init__(self) -> None:
        super().__init__('MoveRobot_node')
        self._straight_distance = 1

    def move_forward(self, steps: float = 1.) -> None:
        """
        Move the robot forward
        :param steps: Number of steps of distances the robot moves forward
        """
        self.get_logger().info(f"Move forward {self._straight_distance * steps} meters")

    def move_backward(self, steps: float = 1.) -> None:
        """
        Move the robot backward
        :param steps: Number of steps of distances the robot moves backward
        """
        self.get_logger().info(f"Move backward {self._straight_distance * steps} meters")

    def turn_right(self, angle: int = 90) -> None:
        """
        Move the robot to the right
        :param angle: int angle of rotation to the right
        """
        self.get_logger().info(f"Move Right {angle}º")

    def turn_left(self, angle: int = 90) -> None:
        """
        Move the robot to the left
        :param angle: int angle of rotation to the left
        """
        self.get_logger().info(f"Move Left {angle}º")

    def stop_robot(self, num=None) -> None:
        """
        Publish the stop signal to robot: linear and angular velocity is 0
        """
        self.get_logger().info("Stop the Robot")
