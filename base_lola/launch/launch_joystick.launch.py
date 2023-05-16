from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sys_freq = 6
    device_arduino = '/dev/arduino'
    joy_config = LaunchConfiguration('joy_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_config',
            default_value='logitech',
            description='Joy config for teleop_twist_joy'
        ),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/opt/ros/humble/share/teleop_twist_joy/launch/teleop-launch.py']),
            launch_arguments={
                'joy_config': joy_config
            }.items()
        )
    ])