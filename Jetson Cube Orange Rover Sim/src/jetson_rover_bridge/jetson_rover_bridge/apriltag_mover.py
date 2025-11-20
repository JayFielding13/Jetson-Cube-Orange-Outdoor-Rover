#!/usr/bin/env python3
"""
AprilTag Mover Node - Simulates a person walking around with an AprilTag

Moves the apriltag_marker model in Gazebo along predefined waypoints,
simulating someone walking through the maze carrying an AprilTag.
The rover can then practice following this moving target.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
import math
import numpy as np


class AprilTagMover(Node):
    def __init__(self):
        super().__init__('apriltag_mover')

        # Parameters
        self.declare_parameter('model_name', 'apriltag_marker')
        self.declare_parameter('speed', 0.5)  # m/s walking speed
        self.declare_parameter('update_rate', 20.0)  # Hz
        self.declare_parameter('loop_path', True)  # Loop back to start
        self.declare_parameter('pause_at_waypoints', 2.0)  # seconds

        self.model_name = self.get_parameter('model_name').value
        self.speed = self.get_parameter('speed').value
        self.update_rate = self.get_parameter('update_rate').value
        self.loop_path = self.get_parameter('loop_path').value
        self.pause_duration = self.get_parameter('pause_at_waypoints').value

        # Define waypoints (x, y, z, yaw) - path through the maze
        # These waypoints create a walking path that the rover can follow
        self.waypoints = [
            (-5.0, 7.0, 0.5, 0.0),      # Start in dead-end corridor
            (-5.0, 5.0, 0.5, -math.pi/2),  # Move toward junction
            (-3.0, 5.0, 0.5, 0.0),      # Turn into main corridor
            (-1.0, 5.0, 0.5, 0.0),      # Continue down corridor
            (1.0, 5.0, 0.5, math.pi/2), # Approach center area
            (1.0, 3.0, 0.5, math.pi),   # Turn toward open area
            (-1.0, 3.0, 0.5, -math.pi/2), # Move to different section
            (-1.0, 1.0, 0.5, 0.0),      # Continue exploring
            (1.0, 1.0, 0.5, math.pi/2), # Back toward center
            (1.0, -1.0, 0.5, math.pi),  # South section
            (-2.0, -1.0, 0.5, 0.0),     # Return path
        ]

        # State variables
        self.current_waypoint_idx = 0
        self.current_position = np.array(self.waypoints[0][:3])
        self.current_yaw = self.waypoints[0][3]
        self.paused_time = 0.0
        self.is_paused = False

        # Create service client for setting model state
        self.set_state_client = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )

        self.get_logger().info('AprilTag mover started!')
        self.get_logger().info(f'Moving {self.model_name} at {self.speed} m/s through {len(self.waypoints)} waypoints')
        self.get_logger().info('Waiting for /gazebo/set_entity_state service to be available...')

        # Create timer for movement updates (will start calling service immediately)
        self.timer = self.create_timer(1.0/self.update_rate, self.update_position)

    def quaternion_from_yaw(self, yaw):
        """
        Convert yaw angle to quaternion with 90° pitch to keep tag upright
        Combines rotation around Y-axis (pitch=90°) and Z-axis (yaw)
        """
        # First rotate 90° around Y-axis (pitch) to stand the tag upright
        pitch = math.pi / 2.0  # 90 degrees

        # Calculate quaternion components for combined rotation
        # Roll-Pitch-Yaw to Quaternion conversion
        roll = 0.0

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy
        )

    def update_position(self):
        """Update the AprilTag position toward the next waypoint"""
        dt = 1.0 / self.update_rate

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
                    return
            else:
                # Stay at current position while paused
                self.send_model_state()
                return

        # Get target waypoint
        target = np.array(self.waypoints[self.current_waypoint_idx][:3])
        target_yaw = self.waypoints[self.current_waypoint_idx][3]

        # Calculate distance to target
        direction = target - self.current_position
        distance = np.linalg.norm(direction)

        # Check if we've reached the waypoint
        if distance < 0.1:  # Within 10cm of waypoint
            self.current_position = target
            self.current_yaw = target_yaw
            self.is_paused = True
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} '
                f'at ({target[0]:.1f}, {target[1]:.1f}). Pausing...'
            )
        else:
            # Move toward target
            direction = direction / distance  # Normalize
            move_distance = min(self.speed * dt, distance)
            self.current_position += direction * move_distance

            # Gradually rotate toward target yaw
            yaw_diff = target_yaw - self.current_yaw
            # Normalize to [-pi, pi]
            while yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi

            # Rotate at 1 rad/s
            max_yaw_change = 1.0 * dt
            yaw_change = np.clip(yaw_diff, -max_yaw_change, max_yaw_change)
            self.current_yaw += yaw_change

        # Send updated state to Gazebo
        self.send_model_state()

    def send_model_state(self):
        """Send the current model state to Gazebo"""
        request = SetEntityState.Request()

        state = EntityState()
        state.name = self.model_name
        state.reference_frame = 'world'

        # Set position and orientation
        state.pose = Pose()
        state.pose.position = Point(
            x=float(self.current_position[0]),
            y=float(self.current_position[1]),
            z=float(self.current_position[2])
        )
        state.pose.orientation = self.quaternion_from_yaw(self.current_yaw)

        # Set velocities to zero (teleporting)
        state.twist = Twist()

        request.state = state

        # Call service asynchronously
        future = self.set_state_client.call_async(request)
        # Don't wait for response to avoid blocking


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagMover()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
