#!/usr/bin/env python3
"""
Follow AprilTag Behavior with State Machine

This node implements a robust AprilTag following behavior with distinct states:
1. SEARCHING - Rotate in place looking for the tag
2. APPROACHING - Move toward tag if too far away
3. FOLLOWING - Maintain target distance while tracking tag
4. AVOIDING - Handle obstacles while trying to keep tag in view
5. LOST - Tag was lost, try to reacquire

Uses LiDAR for obstacle avoidance and AprilTag detection for visual tracking.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
import numpy as np
from enum import Enum
import time


class FollowState(Enum):
    """States for the follow behavior"""
    SEARCHING = 1      # Looking for AprilTag
    APPROACHING = 2    # Tag detected but too far
    FOLLOWING = 3      # Actively following at target distance
    AVOIDING = 4       # Obstacle detected, avoiding while tracking
    LOST = 5          # Tag lost, attempting to reacquire


class FollowAprilTagBehavior(Node):
    def __init__(self):
        super().__init__('follow_apriltag_behavior')

        # ==================== Parameters ====================
        self.declare_parameter('target_distance', 2.0)  # meters
        self.declare_parameter('distance_tolerance', 0.3)  # meters
        self.declare_parameter('max_approach_speed', 0.6)  # m/s
        self.declare_parameter('max_follow_speed', 0.4)  # m/s
        self.declare_parameter('max_turn_speed', 1.0)  # rad/s
        self.declare_parameter('search_turn_speed', 0.5)  # rad/s when searching

        # PID-like gains
        self.declare_parameter('distance_kp', 0.5)
        self.declare_parameter('angle_kp', 1.2)

        # Safety distances
        self.declare_parameter('safety_distance', 1.0)  # meters
        self.declare_parameter('danger_distance', 0.5)  # meters

        # State machine timing
        self.declare_parameter('lost_timeout', 2.0)  # seconds before entering LOST state
        self.declare_parameter('search_timeout', 15.0)  # seconds before giving up search

        self.declare_parameter('enabled', True)

        # Load parameters
        self.target_distance = self.get_parameter('target_distance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.max_approach_speed = self.get_parameter('max_approach_speed').value
        self.max_follow_speed = self.get_parameter('max_follow_speed').value
        self.max_turn_speed = self.get_parameter('max_turn_speed').value
        self.search_turn_speed = self.get_parameter('search_turn_speed').value
        self.distance_kp = self.get_parameter('distance_kp').value
        self.angle_kp = self.get_parameter('angle_kp').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.danger_distance = self.get_parameter('danger_distance').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.enabled = self.get_parameter('enabled').value

        # ==================== State Variables ====================
        self.state = FollowState.SEARCHING
        self.prev_state = None

        # AprilTag tracking
        self.tag_detected = False
        self.tag_distance = 0.0
        self.tag_angle = 0.0  # radians, positive = left, negative = right
        self.tag_position = None  # 3D position if available
        self.last_tag_time = time.time()

        # Obstacle avoidance
        self.latest_scan = None
        self.min_front_distance = float('inf')
        self.clearance_left = float('inf')
        self.clearance_right = float('inf')
        self.obstacle_detected = False

        # Timing
        self.state_enter_time = time.time()
        self.search_start_time = None

        # ==================== Subscribers ====================
        # AprilTag detection (from apriltag_detector)
        self.create_subscription(
            Bool,
            '/apriltag/detected',
            self.tag_detected_callback,
            10
        )

        self.create_subscription(
            Point,
            '/apriltag/position',
            self.tag_position_callback,
            10
        )

        # LiDAR for obstacle avoidance
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # ==================== Publishers ====================
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/follow_apriltag/state', 10)

        # ==================== Control Loop ====================
        # 20 Hz control loop
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Follow AprilTag Behavior started')
        self.get_logger().info(f'  Target distance: {self.target_distance}m (±{self.distance_tolerance}m)')
        self.get_logger().info(f'  Max speeds: approach={self.max_approach_speed}m/s, follow={self.max_follow_speed}m/s')
        self.get_logger().info(f'  Safety distances: safe={self.safety_distance}m, danger={self.danger_distance}m')
        self.get_logger().info(f'  Initial state: {self.state.name}')

    def tag_detected_callback(self, msg):
        """AprilTag detection status callback"""
        self.tag_detected = msg.data
        if self.tag_detected:
            self.last_tag_time = time.time()

    def tag_position_callback(self, msg):
        """AprilTag position callback (x=forward, y=left, z=up in camera frame)"""
        self.tag_position = msg

        # Calculate distance and angle from position
        # x is forward distance, y is lateral offset
        self.tag_distance = np.sqrt(msg.x**2 + msg.y**2)
        self.tag_angle = np.arctan2(msg.y, msg.x)  # Angle to tag

        self.tag_detected = True
        self.last_tag_time = time.time()

    def lidar_callback(self, msg):
        """Process LiDAR scan for obstacle avoidance"""
        self.latest_scan = msg

        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max

        num_beams = len(ranges)
        angles = msg.angle_min + np.arange(num_beams) * msg.angle_increment

        # Front sector (±30 degrees)
        front_mask = np.abs(angles) < np.radians(30)
        self.min_front_distance = np.min(ranges[front_mask]) if np.any(front_mask) else msg.range_max

        # Left sector
        left_mask = (angles > np.radians(30)) & (angles < np.radians(150))
        self.clearance_left = np.min(ranges[left_mask]) if np.any(left_mask) else msg.range_max

        # Right sector
        right_mask = (angles < np.radians(-30)) & (angles > np.radians(-150))
        self.clearance_right = np.min(ranges[right_mask]) if np.any(right_mask) else msg.range_max

        # Check for obstacles
        self.obstacle_detected = (self.min_front_distance < self.safety_distance)

    def change_state(self, new_state):
        """Transition to a new state"""
        if new_state != self.state:
            self.prev_state = self.state
            self.state = new_state
            self.state_enter_time = time.time()
            self.get_logger().info(f'State: {self.prev_state.name} -> {new_state.name}')

            # Publish state change
            state_msg = String()
            state_msg.data = new_state.name
            self.state_pub.publish(state_msg)

    def time_in_state(self):
        """Get time spent in current state"""
        return time.time() - self.state_enter_time

    def time_since_tag(self):
        """Get time since last tag detection"""
        return time.time() - self.last_tag_time

    def control_loop(self):
        """Main control loop - state machine"""
        if not self.enabled:
            return

        # Update state machine
        self.update_state_machine()

        # Execute state behavior
        cmd = Twist()

        if self.state == FollowState.SEARCHING:
            cmd = self.searching_behavior()
        elif self.state == FollowState.APPROACHING:
            cmd = self.approaching_behavior()
        elif self.state == FollowState.FOLLOWING:
            cmd = self.following_behavior()
        elif self.state == FollowState.AVOIDING:
            cmd = self.avoiding_behavior()
        elif self.state == FollowState.LOST:
            cmd = self.lost_behavior()

        # Publish command
        self.cmd_vel_pub.publish(cmd)

    def update_state_machine(self):
        """Update state transitions"""

        # Critical obstacle - always switch to AVOIDING
        if self.min_front_distance < self.danger_distance and self.state != FollowState.AVOIDING:
            self.change_state(FollowState.AVOIDING)
            return

        # State transitions based on current state
        if self.state == FollowState.SEARCHING:
            if self.tag_detected:
                # Found tag, decide if approaching or following
                if self.tag_distance > (self.target_distance + self.distance_tolerance):
                    self.change_state(FollowState.APPROACHING)
                else:
                    self.change_state(FollowState.FOLLOWING)
            elif self.search_start_time and (time.time() - self.search_start_time) > self.search_timeout:
                self.get_logger().warn('Search timeout - tag not found')
                # Could add IDLE state or keep searching
                self.search_start_time = time.time()  # Reset timer

        elif self.state == FollowState.APPROACHING:
            if not self.tag_detected:
                self.change_state(FollowState.LOST)
            elif self.tag_distance <= (self.target_distance + self.distance_tolerance):
                self.change_state(FollowState.FOLLOWING)
            elif self.obstacle_detected:
                self.change_state(FollowState.AVOIDING)

        elif self.state == FollowState.FOLLOWING:
            if not self.tag_detected:
                self.change_state(FollowState.LOST)
            elif self.tag_distance > (self.target_distance + self.distance_tolerance * 2):
                # Tag moved away significantly
                self.change_state(FollowState.APPROACHING)
            elif self.obstacle_detected:
                self.change_state(FollowState.AVOIDING)

        elif self.state == FollowState.AVOIDING:
            # Exit AVOIDING when obstacle cleared AND tag still visible
            if not self.obstacle_detected:
                if self.tag_detected:
                    if self.tag_distance > (self.target_distance + self.distance_tolerance):
                        self.change_state(FollowState.APPROACHING)
                    else:
                        self.change_state(FollowState.FOLLOWING)
                else:
                    self.change_state(FollowState.LOST)

        elif self.state == FollowState.LOST:
            if self.tag_detected:
                # Reacquired tag
                if self.tag_distance > (self.target_distance + self.distance_tolerance):
                    self.change_state(FollowState.APPROACHING)
                else:
                    self.change_state(FollowState.FOLLOWING)
            elif self.time_in_state() > self.lost_timeout:
                # Give up and search
                self.change_state(FollowState.SEARCHING)
                self.search_start_time = time.time()

    def searching_behavior(self):
        """Rotate in place to find AprilTag"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.search_turn_speed  # Slow rotation
        return cmd

    def approaching_behavior(self):
        """Move toward AprilTag when too far away"""
        cmd = Twist()

        # Distance error
        distance_error = self.tag_distance - self.target_distance

        # Linear velocity (proportional to distance error)
        cmd.linear.x = min(self.distance_kp * distance_error, self.max_approach_speed)
        cmd.linear.x = max(cmd.linear.x, 0.1)  # Minimum forward speed

        # Angular velocity (center the tag)
        cmd.angular.z = self.angle_kp * self.tag_angle
        cmd.angular.z = np.clip(cmd.angular.z, -self.max_turn_speed, self.max_turn_speed)

        return cmd

    def following_behavior(self):
        """Maintain target distance while tracking tag"""
        cmd = Twist()

        # Distance error
        distance_error = self.tag_distance - self.target_distance

        # Linear velocity (gentle corrections)
        if abs(distance_error) < self.distance_tolerance:
            # Within tolerance, just match tag motion
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.distance_kp * distance_error
            cmd.linear.x = np.clip(cmd.linear.x, -self.max_follow_speed, self.max_follow_speed)

        # Angular velocity (keep tag centered)
        cmd.angular.z = self.angle_kp * self.tag_angle
        cmd.angular.z = np.clip(cmd.angular.z, -self.max_turn_speed, self.max_turn_speed)

        return cmd

    def avoiding_behavior(self):
        """Avoid obstacles while trying to keep tag in view"""
        cmd = Twist()

        # Critical danger - back up
        if self.min_front_distance < self.danger_distance:
            cmd.linear.x = -0.3
            # Turn away from obstacle
            if self.clearance_left > self.clearance_right:
                cmd.angular.z = self.max_turn_speed
            else:
                cmd.angular.z = -self.max_turn_speed
        else:
            # Moderate obstacle - slow down and steer around
            cmd.linear.x = 0.2  # Slow forward speed

            # If tag is visible, try to bias toward it
            if self.tag_detected:
                # Blend obstacle avoidance with tag tracking
                avoid_turn = self.max_turn_speed if self.clearance_left > self.clearance_right else -self.max_turn_speed
                tag_turn = self.angle_kp * self.tag_angle
                cmd.angular.z = 0.7 * avoid_turn + 0.3 * tag_turn  # Favor avoidance
            else:
                # Just avoid
                cmd.angular.z = self.max_turn_speed if self.clearance_left > self.clearance_right else -self.max_turn_speed

        return cmd

    def lost_behavior(self):
        """Tag lost - try to reacquire by rotating slowly"""
        cmd = Twist()
        cmd.linear.x = 0.0
        # Rotate in same direction tag was last seen
        cmd.angular.z = self.search_turn_speed * np.sign(self.tag_angle) if self.tag_angle != 0 else self.search_turn_speed

        if self.time_in_state() > 1.0:
            self.get_logger().warn(f'Tag lost for {self.time_in_state():.1f}s', throttle_duration_sec=2.0)

        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = FollowAprilTagBehavior()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the rover
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
