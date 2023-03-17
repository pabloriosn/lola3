import rclpy
from base_lola.move_robot import MovementController
import time

def main(args=None):
    rclpy.init(args=args)

    # Crear el nodo MovementController
    movement_controller = MovementController()

    # Mover el robot en un cuadrado de 1 metro
    for _ in range(4):
        movement_controller.move_forward(distance=1, speed=0.2)
        time.sleep(1)  # Pausa para asegurar que el robot haya detenido su movimiento
        movement_controller.turn_left(angle=1.5708, angular_speed=0.5)  # Aproximadamente 90 grados en radianes
        time.sleep(1)

    # Detener el robot y apagar el nodo
    movement_controller.stop()
    movement_controller.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
