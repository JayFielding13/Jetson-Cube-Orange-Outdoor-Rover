from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Complete system for following a moving AprilTag

    This simulates following a person walking around with an AprilTag.
    The apriltag_marker model moves through waypoints while the rover
    uses camera detection and obstacle avoidance to follow it safely.
    """

    # Launch arguments
    enable_ultrasonic = LaunchConfiguration('enable_ultrasonic', default='true')
    enable_lidar = LaunchConfiguration('enable_lidar', default='true')
    debug_images = LaunchConfiguration('debug_images', default='true')
    tag_speed = LaunchConfiguration('tag_speed', default='0.5')  # m/s walking speed
    tag_pause = LaunchConfiguration('tag_pause', default='2.0')  # seconds at waypoints

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'enable_ultrasonic',
            default_value='true',
            description='Enable ultrasonic obstacle detection'
        ),
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='true',
            description='Enable LiDAR obstacle detection'
        ),
        DeclareLaunchArgument(
            'debug_images',
            default_value='true',
            description='Publish debug images with tag detections'
        ),
        DeclareLaunchArgument(
            'tag_speed',
            default_value='0.5',
            description='AprilTag walking speed (m/s)'
        ),
        DeclareLaunchArgument(
            'tag_pause',
            default_value='2.0',
            description='Pause duration at each waypoint (seconds)'
        ),

        # ====================
        # AprilTag Movement
        # ====================
        Node(
            package='jetson_rover_bridge',
            executable='apriltag_mover',
            name='apriltag_mover',
            parameters=[{
                'model_name': 'apriltag_marker',
                'speed': tag_speed,
                'update_rate': 20.0,
                'loop_path': True,
                'pause_at_waypoints': tag_pause,
            }],
            output='screen'
        ),

        # ====================
        # Obstacle Detection
        # ====================

        # Ultrasonic obstacle detector
        Node(
            package='jetson_rover_bridge',
            executable='obstacle_detector_ultrasonic',
            name='ultrasonic_detector',
            parameters=[{
                'danger_distance': 0.5,
                'warning_distance': 1.0,
                'min_valid_range': 0.2,
                'max_valid_range': 6.0,
            }],
            output='screen',
            condition=IfCondition(enable_ultrasonic)
        ),

        # LiDAR obstacle detector
        Node(
            package='jetson_rover_bridge',
            executable='obstacle_detector_lidar',
            name='lidar_detector',
            parameters=[{
                'danger_distance': 0.8,
                'warning_distance': 2.0,
                'field_of_view': 120.0,  # degrees
                'min_valid_range': 0.1,
                'max_valid_range': 12.0,
            }],
            output='screen',
            condition=IfCondition(enable_lidar)
        ),

        # ====================
        # Reactive Obstacle Avoidance (Vector Field Histogram)
        # ====================
        Node(
            package='jetson_rover_bridge',
            executable='reactive_obstacle_avoidance',
            name='reactive_obstacle_avoidance',
            parameters=[{
                'critical_distance': 0.4,     # Absolute minimum (m)
                'influence_distance': 2.5,    # Avoidance influence range (m)
                'obstacle_weight': 2.0,       # Histogram weighting
                'sector_angle': 5.0,          # degrees per sector
                'num_sectors': 72,            # 360 / 5 = 72 sectors
            }],
            output='screen'
        ),

        # ====================
        # AprilTag Detection & Following
        # ====================

        # AprilTag detector
        Node(
            package='jetson_rover_bridge',
            executable='apriltag_detector',
            name='apriltag_detector',
            parameters=[{
                'target_tag_id': 0,
                'camera_fx': 600.0,  # Will auto-update from camera_info
                'camera_fy': 600.0,
                'camera_cx': 960.0,
                'camera_cy': 540.0,
                'tag_size': 0.165,  # 6.5 inches in meters
                'debug_image': debug_images,
            }],
            output='screen'
        ),

        # AprilTag follower (Reactive version with obstacle avoidance)
        Node(
            package='jetson_rover_bridge',
            executable='apriltag_follower_reactive',
            name='apriltag_follower_reactive',
            parameters=[{
                'target_distance': 2.0,         # Follow at 2m distance
                'distance_tolerance': 0.3,      # meters
                'max_linear_speed': 0.6,        # m/s (slightly faster than tag)
                'max_angular_speed': 1.0,       # rad/s
                'linear_kp': 0.5,               # P gain for distance
                'angular_kp': 1.2,              # P gain for angle
                'avoidance_blend': 0.7,         # Weight avoidance highly
                'min_forward_speed': 0.1,       # Never stop completely
                'search_angular_speed': 0.4,    # rad/s when searching
                'search_timeout': 15.0,         # seconds before giving up
                'lost_tag_timeout': 1.5,        # seconds before starting search
            }],
            output='screen'
        ),
    ])
