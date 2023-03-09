from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sys_freq = 6
    device_arduino = '/dev/arduino'

    return LaunchDescription([
        Node(
            package='base_lola',
            executable='base_lola',
            name='hwinterface',
            parameters=[
                {'device_ard': device_arduino},  # Cambiar a la ruta correcta del dispositivo
                {'baudrate': 115200},
                {'timer_publisher': sys_freq},  # Hz
                {'name_pub': 'wheel_state'},
                {'name_sub': 'cmd_wheel'}
            ]
        ),
        Node(
            package='base_lola',
            executable='controller_lola',
            name='odometry_calculator'
        ),
        Node(
            package='base_lola',
            executable='move',
            name='move_robot'
        )
    ])