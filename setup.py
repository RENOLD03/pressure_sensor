from setuptools import setup
import os
from glob import glob

package_name = 'pressure_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shoban Renold J',
    maintainer_email='shoban.r@aquaairx.com',
    description='MS5837 pressure sensor integration for ROS 2 with depth-to-odometry publishing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pressure_node = pressure_sensor.pressure_node:main',
            'depth_odom_node = pressure_sensor.depth_odom_node:main',
        ],
    },
)
