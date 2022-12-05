from setuptools import setup

package_name = 'vsn'
submodule_ia = 'vsn/models'
submodule_service = 'vsn/service'
submodule_image = 'vsn/image'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, submodule_ia, submodule_service],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablo',
    maintainer_email='p.riosnavarro@gmail.com',
    description='Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vsn_launch = vsn.vsn_launch:main',
        ],
    },
)
