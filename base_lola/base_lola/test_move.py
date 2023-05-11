import rclpy
from base_lola.move_robot import MovementController
import time

def main(args=None):
    try:
        rclpy.init(args=args)

        # Create the MovementController node
        movement_controller = MovementController()

        # Move the robot in a 1-meter square
        for _ in range(2):

            movement_controller.turn_right(angle=360, angular_speed=0.8)
            time.sleep(1)
            movement_controller.turn_left(angle=359, angular_speed=0.8)
            time.sleep(1)
            movement_controller.move_forward(distance=0.5, speed=0.5)
            time.sleep(1)  # Pause to ensure the robot has stopped moving
            movement_controller.move_backward(distance=0.5, speed=0.5)
            time.sleep(1)  # Pause to ensure the robot has stopped moving

    except KeyboardInterrupt:
        movement_controller.get_logger().info("Keyboard interrupt, stopping robot")
    finally:
        # Stop the robot and shut down the node
        movement_controller.stop()
        movement_controller.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





