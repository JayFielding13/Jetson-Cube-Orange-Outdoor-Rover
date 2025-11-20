#!/usr/bin/env python3
"""
Person Waypoint Driver - Drives the person_with_apriltag robot through waypoints

This node drives a simple mobile robot (representing a person with an AprilTag)
through predefined waypoints using smooth velocity commands, simulating someone
walking through the maze.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np


class PersonWaypointDriver(Node):
    def __init__(self):
        super().__init__('person_waypoint_driver')

        # Parameters
        self.declare_parameter('speed', 0.3)  # m/s walking speed (slow for following)
        self.declare_parameter('angular_speed', 0.8)  # rad/s turning speed
        self.declare_parameter('update_rate', 20.0)  # Hz
        self.declare_parameter('loop_path', True)
        self.declare_parameter('pause_at_waypoints', 1.0)  # seconds

        self.speed = self.get_parameter('speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.update_rate = self.get_parameter('update_rate').value
        self.loop_path = self.get_parameter('loop_path').value
        self.pause_duration = self.get_parameter('pause_at_waypoints').value

        # Define waypoints (x, y) - simple path through open areas
        self.waypoints = [
            (-5.0, 7.0),   # Start in dead-end corridor
            (-5.0, 5.0),   # Move toward junction
            (-3.0, 5.0),   # Turn into corridor
            (-1.0, 5.0),   # Continue down corridor
            (1.0, 5.0),    # Open area
            (1.0, 3.0),    # Turn south
            (1.0, 1.0),    # Continue south
            (1.0, -1.0),   # Continue south
            (-1.0, -1.0),  # Turn west
            (-3.0, -1.0),  # Continue west
            (-5.0, -1.0),  # Continue west
            (-5.0, 1.0),   # Turn north
            (-5.0, 3.0),   # Continue north
            (-5.0, 5.0),   # Back to start area
        ]

        # State variables
        self.current_waypoint_idx = 0
        self.current_position = np.array([0.0, 0.0])
        self.current_yaw = 0.0
        self.paused_time = 0.0
        self.is_paused = False
        self.position_initialized = False

        # Subscribe to odometry
        self.create_subscription(
            Odometry,
            '/person_odom',
            self.odom_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/person_cmd_vel', 10)

        # Control loop timer
        self.timer = self.create_timer(1.0/self.update_rate, self.control_loop)

        self.get_logger().info('Person waypoint driver started!')
        self.get_logger().info(f'Walking at {self.speed} m/s through {len(self.waypoints)} waypoints')

    def odom_callback(self, msg):
        """Update current position from odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.position_initialized = True

    def control_loop(self):
        """Main control loop"""
        if not self.position_initialized:
            return

        dt = 1.0 / self.update_rate
        cmd = Twist()

        # Handle pause at waypoint
        if self.is_paused:
            self.paused_time += dt
            if self.paused_time >= self.pause_duration:
                self.is_paused = False
                self.paused_time = 0.0
                # Move to next waypoint
                self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
                if not self.loop_path and self.current_waypoint_idx == 0:
                    self.get_logger().info('Completed path. Stopping.')
                    self.timer.cancel()
                    self.cmd_vel_pub.publish(cmd)  # Stop
                    return
            else:
                # Stay stopped while paused
                self.cmd_vel_pub.publish(cmd)
                return

        # Get target waypoint
        target = np.array(self.waypoints[self.current_waypoint_idx])

        # Calculate distance and angle to target
        delta = target - self.current_position
        distance = np.linalg.norm(delta)
        target_yaw = math.atan2(delta[1], delta[0])

        # Check if we've reached the waypoint
        if distance < 0.2:  # Within 20cm of waypoint
            self.is_paused = True
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} '
                f'at ({target[0]:.1f}, {target[1]:.1f})'
            )
            self.cmd_vel_pub.publish(cmd)  # Stop
            return

        # Calculate heading error
        heading_error = target_yaw - self.current_yaw
        # Normalize to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # If heading error is large, rotate in place
        if abs(heading_error) > 0.15:  # ~9 degrees
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed * np.sign(heading_error)
        else:
            # Drive toward target with smooth speed
            cmd.linear.x = min(self.speed, distance * 0.5)  # Slow down as we approach
            # Add proportional steering
            cmd.angular.z = 1.5 * heading_error

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PersonWaypointDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the person robot
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
