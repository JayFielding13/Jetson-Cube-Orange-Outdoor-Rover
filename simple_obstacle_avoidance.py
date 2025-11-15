#!/usr/bin/env python3
"""
Simple Obstacle Avoidance Controller for Jetson Rover
Uses LiDAR data to navigate and avoid obstacles

Author: Developed in simulation for safe testing
Date: November 11, 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Parameters
        self.declare_parameter('max_speed', 0.5)  # m/s
        self.declare_parameter('min_distance', 1.5)  # meters - stop distance
        self.declare_parameter('safe_distance', 2.5)  # meters - slow down distance
        self.declare_parameter('turn_speed', 0.3)  # rad/s

        self.max_speed = self.get_parameter('max_speed').value
        self.min_distance = self.get_parameter('min_distance').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.turn_speed = self.get_parameter('turn_speed').value

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self.latest_scan = None

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Obstacle Avoidance Controller Started!')
        self.get_logger().info(f'  Max Speed: {self.max_speed} m/s')
        self.get_logger().info(f'  Stop Distance: {self.min_distance} m')
        self.get_logger().info(f'  Safe Distance: {self.safe_distance} m')
        self.get_logger().info(f'  Turn Speed: {self.turn_speed} rad/s')

    def scan_callback(self, msg):
        """Store the latest LiDAR scan"""
        self.latest_scan = msg

    def get_sector_distances(self, scan):
        """
        Divide the LiDAR scan into sectors and find minimum distance in each

        Sectors (for 360° LiDAR):
        - Front: -30° to +30° (330° to 30°)
        - Front-Left: 30° to 90°
        - Left: 90° to 150°
        - Front-Right: 270° to 330°
        - Right: 210° to 270°

        Returns dict with min distance in each sector
        """
        ranges = np.array(scan.ranges)

        # Replace inf and invalid values with max range
        ranges = np.where(np.isinf(ranges), scan.range_max, ranges)
        ranges = np.where(ranges < scan.range_min, scan.range_max, ranges)

        num_readings = len(ranges)
        angle_increment = scan.angle_increment

        # Helper function to get min distance in angle range
        def min_in_range(start_deg, end_deg):
            # Convert degrees to indices
            start_idx = int((start_deg * np.pi / 180 - scan.angle_min) / angle_increment)
            end_idx = int((end_deg * np.pi / 180 - scan.angle_min) / angle_increment)

            # Handle wraparound for front sector
            if start_idx < 0 or end_idx >= num_readings:
                # Front sector wraps around
                if start_deg > 180:
                    indices = list(range(start_idx % num_readings, num_readings)) + list(range(0, end_idx))
                else:
                    indices = list(range(max(0, start_idx), min(num_readings, end_idx)))
            else:
                indices = list(range(start_idx, end_idx))

            if len(indices) > 0:
                sector_ranges = ranges[indices]
                return np.min(sector_ranges)
            else:
                return scan.range_max

        sectors = {
            'front': min_in_range(-30, 30),
            'front_left': min_in_range(30, 90),
            'left': min_in_range(90, 150),
            'front_right': min_in_range(-90, -30),
            'right': min_in_range(-150, -90),
        }

        return sectors

    def control_loop(self):
        """Main control loop - decides what to do based on LiDAR data"""

        if self.latest_scan is None:
            # No scan data yet
            return

        # Get distances in each sector
        sectors = self.get_sector_distances(self.latest_scan)

        # Create velocity command
        cmd = Twist()

        # Get the closest distance in front sectors
        front_dist = sectors['front']
        front_left_dist = sectors['front_left']
        front_right_dist = sectors['front_right']

        # Find closest obstacle in front 180°
        min_front_dist = min(front_dist, front_left_dist, front_right_dist)

        # Decision logic
        if min_front_dist < self.min_distance:
            # STOP! Obstacle too close
            cmd.linear.x = 0.0

            # Turn away from obstacle
            if front_left_dist < front_right_dist:
                # Obstacle on left, turn right
                cmd.angular.z = -self.turn_speed
                self.get_logger().info(f'⚠️  STOP & TURN RIGHT - Obstacle at {min_front_dist:.2f}m')
            else:
                # Obstacle on right (or centered), turn left
                cmd.angular.z = self.turn_speed
                self.get_logger().info(f'⚠️  STOP & TURN LEFT - Obstacle at {min_front_dist:.2f}m')

        elif min_front_dist < self.safe_distance:
            # Slow down and turn slightly
            cmd.linear.x = self.max_speed * 0.3  # 30% speed

            if front_left_dist < front_right_dist:
                # Obstacle on left, turn right gently
                cmd.angular.z = -self.turn_speed * 0.5
                self.get_logger().info(f'⚡ SLOW & TURN RIGHT - Obstacle at {min_front_dist:.2f}m',
                                     throttle_duration_sec=1.0)
            else:
                # Obstacle on right, turn left gently
                cmd.angular.z = self.turn_speed * 0.5
                self.get_logger().info(f'⚡ SLOW & TURN LEFT - Obstacle at {min_front_dist:.2f}m',
                                     throttle_duration_sec=1.0)

        else:
            # All clear - move forward at full speed
            cmd.linear.x = self.max_speed
            cmd.angular.z = 0.0
            self.get_logger().info(f'✓ CLEAR - Moving forward. Closest obstacle: {min_front_dist:.2f}m',
                                 throttle_duration_sec=2.0)

        # Publish command
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ObstacleAvoidance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        node.get_logger().info('Shutting down - sending stop command')
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
