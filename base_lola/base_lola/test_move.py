import rclpy
from base_lola.move_robot import MovementController
import time

def main(args=None):
    rclpy.init(args=args)

    # Create the MovementController node
    movement_controller = MovementController()

    # Move the robot in a 1-meter square
    for _ in range(4):
        movement_controller.move_forward(distance=1, speed=0.2)
        time.sleep(1)  # Pause to ensure the robot has stopped moving
        movement_controller.turn_left(angle=90, angular_speed=0.5)
        time.sleep(1)

    # Stop the robot and shut down the node
    movement_controller.stop()
    movement_controller.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





