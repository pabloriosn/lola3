from setuptools import setup
import os
from glob import glob

package_name = 'base_lola'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablo',
    maintainer_email='pablorios@edu.uah.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_lola = base_lola.lolaHW:main',
            'controller_lola = base_lola.lola_controller:main',
            'twist_publisher = base_lola.twist_publisher:main',
            'move = base_lola.test_move:main',
            'api_robot = base_lola.move_robot:MoveRobot',
            'server = base_lola.odom_server:main',
            'keyboard = base_lola.keyboard_input:main'
        ],
    },
)
