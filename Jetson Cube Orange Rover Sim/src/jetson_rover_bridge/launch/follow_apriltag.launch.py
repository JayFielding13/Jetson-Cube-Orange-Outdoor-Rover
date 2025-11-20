from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for AprilTag following behavior with state machine

    This uses the new follow_apriltag_behavior which has distinct states:
    - SEARCHING: Rotate to find tag
    - APPROACHING: Move toward tag when far
    - FOLLOWING: Maintain target distance
    - AVOIDING: Handle obstacles while tracking
    - LOST: Attempt to reacquire lost tag
    """

    # Launch arguments
    target_distance = LaunchConfiguration('target_distance', default='2.0')
    distance_tolerance = LaunchConfiguration('distance_tolerance', default='0.3')
    max_approach_speed = LaunchConfiguration('max_approach_speed', default='0.6')
    max_follow_speed = LaunchConfiguration('max_follow_speed', default='0.4')
    max_turn_speed = LaunchConfiguration('max_turn_speed', default='1.0')
    search_turn_speed = LaunchConfiguration('search_turn_speed', default='0.5')

    # Control gains
    distance_kp = LaunchConfiguration('distance_kp', default='0.5')
    angle_kp = LaunchConfiguration('angle_kp', default='1.2')

    # Safety
    safety_distance = LaunchConfiguration('safety_distance', default='1.0')
    danger_distance = LaunchConfiguration('danger_distance', default='0.5')

    # Timing
    lost_timeout = LaunchConfiguration('lost_timeout', default='2.0')
    search_timeout = LaunchConfiguration('search_timeout', default='15.0')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'target_distance',
            default_value='2.0',
            description='Target following distance (meters)'
        ),
        DeclareLaunchArgument(
            'distance_tolerance',
            default_value='0.3',
            description='Distance tolerance (meters)'
        ),
        DeclareLaunchArgument(
            'max_approach_speed',
            default_value='0.6',
            description='Maximum speed when approaching (m/s)'
        ),
        DeclareLaunchArgument(
            'max_follow_speed',
            default_value='0.4',
            description='Maximum speed when following (m/s)'
        ),
        DeclareLaunchArgument(
            'max_turn_speed',
            default_value='1.0',
            description='Maximum turn speed (rad/s)'
        ),
        DeclareLaunchArgument(
            'search_turn_speed',
            default_value='0.5',
            description='Turn speed when searching (rad/s)'
        ),
        DeclareLaunchArgument(
            'distance_kp',
            default_value='0.5',
            description='Proportional gain for distance control'
        ),
        DeclareLaunchArgument(
            'angle_kp',
            default_value='1.2',
            description='Proportional gain for angle control'
        ),
        DeclareLaunchArgument(
            'safety_distance',
            default_value='1.0',
            description='Safety distance for obstacles (meters)'
        ),
        DeclareLaunchArgument(
            'danger_distance',
            default_value='0.5',
            description='Danger distance triggering backup (meters)'
        ),
        DeclareLaunchArgument(
            'lost_timeout',
            default_value='2.0',
            description='Timeout before entering LOST state (seconds)'
        ),
        DeclareLaunchArgument(
            'search_timeout',
            default_value='15.0',
            description='Timeout for search before giving up (seconds)'
        ),

        # AprilTag detector
        Node(
            package='jetson_rover_bridge',
            executable='apriltag_detector',
            name='apriltag_detector',
            parameters=[{
                'target_tag_id': 0,
                'camera_fx': 600.0,
                'camera_fy': 600.0,
                'camera_cx': 960.0,
                'camera_cy': 540.0,
                'tag_size': 0.165,  # 6.5 inches
                'debug_image': True,
            }],
            output='screen'
        ),

        # Follow AprilTag Behavior (state machine)
        Node(
            package='jetson_rover_bridge',
            executable='follow_apriltag_behavior',
            name='follow_apriltag_behavior',
            parameters=[{
                'target_distance': target_distance,
                'distance_tolerance': distance_tolerance,
                'max_approach_speed': max_approach_speed,
                'max_follow_speed': max_follow_speed,
                'max_turn_speed': max_turn_speed,
                'search_turn_speed': search_turn_speed,
                'distance_kp': distance_kp,
                'angle_kp': angle_kp,
                'safety_distance': safety_distance,
                'danger_distance': danger_distance,
                'lost_timeout': lost_timeout,
                'search_timeout': search_timeout,
                'enabled': True,
            }],
            output='screen'
        ),
    ])
