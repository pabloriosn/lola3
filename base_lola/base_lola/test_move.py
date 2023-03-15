import rclpy
import time
from threading import Thread
from base_lola.move_robot import MoveRobot

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("test_move_square")
    robot = MoveRobot(node)


    # Espera a que el nodo reciba la posición inicial del robot
    #while not robot.position_robot and rclpy.ok():
        #time.sleep(0.1)

    print("Avanti marinero")
    robot.move_forward()
    robot.turn_left()

    robot.move_forward()
    robot.turn_left()

    robot.move_forward()
    robot.turn_left()

    robot.move_forward()
    robot.turn_left()

    robot.stop_robot()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
