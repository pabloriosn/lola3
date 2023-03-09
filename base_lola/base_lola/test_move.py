import rclpy
import time

from base_lola.move_robot import MoveRobot


def main(args=None):
    rclpy.init(args=args)



    node = rclpy.create_node("test_move_square")

    robot = MoveRobot(node)
    node.get_logger().info("test_move_square node started")

    while not robot.position_robot and rclpy.ok():
        time.sleep(0.1)

    node.get_logger().info("Avanti marinero")
    robot.move_forward()
    robot.turn_left()

    robot.move_forward()
    robot.turn_left()

    robot.move_forward()
    robot.turn_left()

    robot.move_forward()
    robot.turn_left()

    robot.stop_robot()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
