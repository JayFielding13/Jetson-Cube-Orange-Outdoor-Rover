#!/usr/bin/env python3
"""
MAVROS2 Launch File for Cube Orange Flight Controller with HERE 3+ RTK GPS
Jetson Orin Nano → Cube Orange via USB Serial

Connection: /dev/ttyACM0 @ 57600 baud
System ID: 1 (Cube Orange autopilot)
Component ID: 1 (autopilot component)
GPS: HERE 3+ with RTK capability
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:57600',
            description='FCU connection URL'
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='GCS connection URL (empty = no GCS)'
        ),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='Target system ID (Cube Orange)'
        ),
        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='Target component ID'
        ),
        DeclareLaunchArgument(
            'log_output',
            default_value='screen',
            description='Log output destination'
        ),
        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='FCU protocol version'
        ),
        DeclareLaunchArgument(
            'respawn_mavros',
            default_value='false',
            description='Respawn MAVROS node on crash'
        ),

        # MAVROS node
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output=LaunchConfiguration('log_output'),
            respawn=LaunchConfiguration('respawn_mavros'),
            respawn_delay=2.0,
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'target_system_id': LaunchConfiguration('tgt_system'),
                'target_component_id': LaunchConfiguration('tgt_component'),
                'fcu_protocol': LaunchConfiguration('fcu_protocol'),

                # System configuration
                'system_id': 255,  # GCS/companion computer ID
                'component_id': 240,  # Companion computer component

                # Connection settings
                'conn': {
                    'timeout': 30.0,  # Connection timeout (seconds)
                },

                # Time synchronization
                'time': {
                    'time_ref_source': 'fcu',  # Use FCU as time reference
                    'timesync_mode': 'MAVLINK',  # MAVLink timesync
                    'timesync_avg_alpha': 0.6,
                },

                # TF frames
                'local_frame': 'map',  # Local frame for odometry
                'tf': {
                    'send': True,  # Publish TF transforms
                    'frame_id': 'map',
                    'child_frame_id': 'base_link',
                    'rate_limit': 50.0,  # Max TF publish rate (Hz)
                },

                # Plugin configuration - ADDED RTK SUPPORT
                'plugin_allowlist': [
                    'sys_status',      # System status
                    'sys_time',        # Time synchronization
                    'gps',             # GPS data
                    'global_position', # Global position (GPS)
                    'imu',             # IMU data
                    'battery',         # Battery status
                    'command',         # MAVLink commands
                    'rc_io',           # RC input/output
                    'setpoint_attitude', # Attitude setpoint
                    'gps_status',      # GPS status (includes RTK)
                    'gps_rtk',         # RTK GPS plugin
                ],

                # GPS plugin settings
                'gps': {
                    'frame_id': 'gps',
                    'eph_fudge_factor': 1.1,
                    'epv_fudge_factor': 1.0,
                    'horiz_accuracy_fudge_factor': 1.0,
                    'vert_accuracy_fudge_factor': 1.0,
                    'speed_accuracy_fudge_factor': 1.0,
                },

                # RTK GPS settings
                'gps_rtk': {
                    'frame_id': 'gps',
                },

                # IMU plugin settings
                'imu': {
                    'frame_id': 'base_link',
                    'linear_acceleration_stdev': 0.0003,
                    'angular_velocity_stdev': 0.0003,
                    'orientation_stdev': 0.0,
                    'magnetic_stdev': 0.0,
                },

                # Battery plugin settings
                'battery': {
                    'voltage_fudge_factor': 1.0,
                    'current_fudge_factor': 1.0,
                },

                # System status plugin
                'sys': {
                    'min_voltage': 10.0,  # Minimum voltage warning
                    'disable_diag': False,
                },
            }]
        ),
    ])
