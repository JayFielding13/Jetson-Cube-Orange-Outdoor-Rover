from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetson_rover_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='JayFielding13@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'http_bridge = jetson_rover_bridge.http_bridge:main',
            'gps_bridge = jetson_rover_bridge.gps_bridge:main',
            'interactive_map = jetson_rover_bridge.interactive_map:main',
            'obstacle_detector_ultrasonic = jetson_rover_bridge.obstacle_detector_ultrasonic:main',
            'obstacle_detector_lidar = jetson_rover_bridge.obstacle_detector_lidar:main',
            'obstacle_fusion = jetson_rover_bridge.obstacle_fusion:main',
            'apriltag_detector = jetson_rover_bridge.apriltag_detector:main',
            'apriltag_follower = jetson_rover_bridge.apriltag_follower:main',
            'reactive_obstacle_avoidance = jetson_rover_bridge.reactive_obstacle_avoidance:main',
            'apriltag_follower_reactive = jetson_rover_bridge.apriltag_follower_reactive:main',
            'ultrasonic_gazebo_simulator = jetson_rover_bridge.ultrasonic_gazebo_simulator:main',
            'ultrasonic_bridge_sim = jetson_rover_bridge.ultrasonic_bridge_sim:main',
            'wander_behavior = jetson_rover_bridge.wander_behavior:main',
            'exploration_behavior = jetson_rover_bridge.exploration_behavior:main',
            'motion_smoother = jetson_rover_bridge.motion_smoother:main',
        ],
    },
)
