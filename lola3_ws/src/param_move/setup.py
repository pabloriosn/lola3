from setuptools import setup

package_name = 'param_move'
submodule_service = 'param_move/service'
submodule_robot = 'param_move/action_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, submodule_robot, submodule_service],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablo',
    maintainer_email='p.riosnavarro@gmail.com',
    description='Package to control the robot with parameterized movements',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = param_move.service_launch:main',
        ],
    },
)
