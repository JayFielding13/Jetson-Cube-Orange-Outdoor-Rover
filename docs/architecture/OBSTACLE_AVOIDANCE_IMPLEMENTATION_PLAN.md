# Obstacle Avoidance & AprilTag Follow-Me - Implementation Plan
**Foundation for Autonomous Operation**

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│              Obstacle Avoidance Stack                │
└─────────────────────────────────────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
    ┌───▼───┐      ┌─────▼─────┐   ┌─────▼──────┐
    │LiDAR  │      │Ultrasonics│   │  Camera    │
    │Primary│      │Fallback   │   │ AprilTag   │
    └───┬───┘      └─────┬─────┘   └─────┬──────┘
        │                │                │
        └────────────────┼────────────────┘
                         │
                ┌────────▼─────────┐
                │  Sensor Fusion   │
                │  Obstacle Map    │
                └────────┬─────────┘
                         │
                ┌────────▼─────────┐
                │ Motion Planner   │
                │ Avoid + Follow   │
                └────────┬─────────┘
                         │
                    ┌────▼─────┐
                    │ cmd_vel  │
                    │ Commands │
                    └──────────┘
```

---

## Phase 1: Basic Obstacle Detection

### 1.1 Ultrasonic Obstacle Detector

**File:** `ros2_ws/src/jetson_rover_bridge/scripts/obstacle_detector_ultrasonic.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
import numpy as np

class UltrasonicObstacleDetector(Node):
    def __init__(self):
        super().__init__('ultrasonic_obstacle_detector')

        # Configurable thresholds
        self.declare_parameter('danger_distance', 0.5)  # meters
        self.declare_parameter('warning_distance', 1.0)  # meters
        self.declare_parameter('min_valid_range', 0.2)   # meters
        self.declare_parameter('max_valid_range', 6.0)   # meters

        self.danger_dist = self.get_parameter('danger_distance').value
        self.warning_dist = self.get_parameter('warning_distance').value
        self.min_range = self.get_parameter('min_valid_range').value
        self.max_range = self.get_parameter('max_valid_range').value

        # Sensor positions (angles in radians from front)
        self.sensors = {
            'front': 0.0,           # 0°
            'corner_left': 0.785,   # 45°
            'side_left': 1.571,     # 90°
            'rear': 3.142,          # 180°
            'side_right': -1.571,   # -90°
            'corner_right': -0.785  # -45°
        }

        # Store latest readings
        self.readings = {name: float('inf') for name in self.sensors.keys()}

        # Subscribers for each ultrasonic sensor
        for sensor_name in self.sensors.keys():
            self.create_subscription(
                Range,
                f'/ultrasonic/{sensor_name}',
                lambda msg, name=sensor_name: self.sensor_callback(msg, name),
                10
            )

        # Publishers
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacles/ultrasonic/detected', 10)
        self.danger_zone_pub = self.create_publisher(Bool, '/obstacles/ultrasonic/danger', 10)

        # Timer for processing
        self.create_timer(0.1, self.process_obstacles)  # 10 Hz

        self.get_logger().info('Ultrasonic Obstacle Detector started')

    def sensor_callback(self, msg, sensor_name):
        """Store sensor reading if valid"""
        if self.min_range <= msg.range <= self.max_range:
            self.readings[sensor_name] = msg.range
        else:
            self.readings[sensor_name] = float('inf')

    def process_obstacles(self):
        """Analyze sensor readings and publish obstacle status"""
        # Find closest obstacle
        min_distance = min(self.readings.values())

        # Determine which zone
        obstacle_detected = min_distance < self.warning_dist
        danger_zone = min_distance < self.danger_dist

        # Publish status
        self.obstacle_detected_pub.publish(Bool(data=obstacle_detected))
        self.danger_zone_pub.publish(Bool(data=danger_zone))

        if danger_zone:
            # Find which sensor detected it
            closest_sensor = min(self.readings, key=self.readings.get)
            self.get_logger().warn(
                f'DANGER: Obstacle at {min_distance:.2f}m on {closest_sensor}',
                throttle_duration_sec=1.0
            )
        elif obstacle_detected:
            self.get_logger().info(
                f'WARNING: Obstacle at {min_distance:.2f}m',
                throttle_duration_sec=1.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 1.2 LiDAR Obstacle Detector

**File:** `ros2_ws/src/jetson_rover_bridge/scripts/obstacle_detector_lidar.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
import numpy as np

class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        # Parameters
        self.declare_parameter('danger_distance', 0.8)   # meters
        self.declare_parameter('warning_distance', 2.0)  # meters
        self.declare_parameter('field_of_view', 120.0)   # degrees (front arc)

        self.danger_dist = self.get_parameter('danger_distance').value
        self.warning_dist = self.get_parameter('warning_distance').value
        self.fov = np.radians(self.get_parameter('field_of_view').value)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publishers
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacles/lidar/detected', 10)
        self.danger_zone_pub = self.create_publisher(Bool, '/obstacles/lidar/danger', 10)
        self.closest_obstacle_pub = self.create_publisher(Vector3, '/obstacles/lidar/closest', 10)

        self.get_logger().info('LiDAR Obstacle Detector started')

    def scan_callback(self, msg):
        """Process LiDAR scan for obstacles"""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter for front arc only (adjustable FOV)
        half_fov = self.fov / 2
        front_indices = np.where(np.abs(angles) <= half_fov)[0]

        if len(front_indices) == 0:
            return

        front_ranges = ranges[front_indices]
        front_angles = angles[front_indices]

        # Remove invalid readings
        valid_mask = (front_ranges >= msg.range_min) & (front_ranges <= msg.range_max)
        valid_ranges = front_ranges[valid_mask]
        valid_angles = front_angles[valid_mask]

        if len(valid_ranges) == 0:
            return

        # Find closest obstacle
        min_idx = np.argmin(valid_ranges)
        min_distance = valid_ranges[min_idx]
        min_angle = valid_angles[min_idx]

        # Convert to cartesian
        obstacle_x = min_distance * np.cos(min_angle)
        obstacle_y = min_distance * np.sin(min_angle)

        # Publish closest obstacle position
        obstacle_msg = Vector3()
        obstacle_msg.x = obstacle_x
        obstacle_msg.y = obstacle_y
        obstacle_msg.z = 0.0
        self.closest_obstacle_pub.publish(obstacle_msg)

        # Determine zones
        obstacle_detected = min_distance < self.warning_dist
        danger_zone = min_distance < self.danger_dist

        # Publish status
        self.obstacle_detected_pub.publish(Bool(data=obstacle_detected))
        self.danger_zone_pub.publish(Bool(data=danger_zone))

        if danger_zone:
            self.get_logger().warn(
                f'LiDAR DANGER: Obstacle at {min_distance:.2f}m, {np.degrees(min_angle):.1f}°',
                throttle_duration_sec=0.5
            )

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Phase 2: Sensor Fusion

### 2.1 Combined Obstacle Detector

**File:** `ros2_ws/src/jetson_rover_bridge/scripts/obstacle_fusion.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ObstacleFusion(Node):
    """
    Fuses LiDAR and Ultrasonic obstacle detection.
    - LiDAR is primary (long range, 360°)
    - Ultrasonics are fallback (short range, low objects)
    - If EITHER detects danger, stop the rover
    """
    def __init__(self):
        super().__init__('obstacle_fusion')

        # State tracking
        self.lidar_danger = False
        self.ultrasonic_danger = False
        self.lidar_warning = False
        self.ultrasonic_warning = False

        # Subscribers
        self.create_subscription(Bool, '/obstacles/lidar/danger', self.lidar_danger_cb, 10)
        self.create_subscription(Bool, '/obstacles/lidar/detected', self.lidar_warning_cb, 10)
        self.create_subscription(Bool, '/obstacles/ultrasonic/danger', self.ultrasonic_danger_cb, 10)
        self.create_subscription(Bool, '/obstacles/ultrasonic/detected', self.ultrasonic_warning_cb, 10)

        # Publishers
        self.danger_pub = self.create_publisher(Bool, '/obstacles/danger', 10)
        self.warning_pub = self.create_publisher(Bool, '/obstacles/warning', 10)
        self.emergency_stop_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        # Timer for fusion logic
        self.create_timer(0.1, self.fusion_callback)  # 10 Hz

        self.get_logger().info('Obstacle Fusion started')

    def lidar_danger_cb(self, msg):
        self.lidar_danger = msg.data

    def lidar_warning_cb(self, msg):
        self.lidar_warning = msg.data

    def ultrasonic_danger_cb(self, msg):
        self.ultrasonic_danger = msg.data

    def ultrasonic_warning_cb(self, msg):
        self.ultrasonic_warning = msg.data

    def fusion_callback(self):
        """Combine sensor inputs using OR logic"""
        # If EITHER sensor sees danger, it's danger
        danger = self.lidar_danger or self.ultrasonic_danger
        warning = self.lidar_warning or self.ultrasonic_warning

        # Publish fused status
        self.danger_pub.publish(Bool(data=danger))
        self.warning_pub.publish(Bool(data=warning))

        # Emergency stop if danger
        if danger:
            stop_cmd = Twist()
            self.emergency_stop_pub.publish(stop_cmd)

            # Log which sensor triggered
            source = []
            if self.lidar_danger:
                source.append('LiDAR')
            if self.ultrasonic_danger:
                source.append('Ultrasonic')

            self.get_logger().warn(
                f'EMERGENCY STOP: {", ".join(source)}',
                throttle_duration_sec=1.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Phase 3: AprilTag Detection

### 3.1 Install AprilTag Library

**On Jetson:**
```bash
ssh jay@100.91.191.47
sudo apt install -y ros-humble-apriltag-ros
sudo apt install -y python3-pip
pip3 install pupil-apriltags opencv-python
```

### 3.2 AprilTag Detector Node

**File:** `ros2_ws/src/jetson_rover_bridge/scripts/apriltag_detector.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        # Parameters
        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('camera_fx', 600.0)  # Focal length (calibrate!)
        self.declare_parameter('camera_fy', 600.0)
        self.declare_parameter('camera_cx', 320.0)  # Image center
        self.declare_parameter('camera_cy', 240.0)
        self.declare_parameter('tag_size', 0.165)   # meters (6.5 inches)

        self.target_id = self.get_parameter('target_tag_id').value
        fx = self.get_parameter('camera_fx').value
        fy = self.get_parameter('camera_fy').value
        cx = self.get_parameter('camera_cx').value
        cy = self.get_parameter('camera_cy').value
        self.tag_size = self.get_parameter('tag_size').value

        # Camera intrinsics
        self.camera_params = [fx, fy, cx, cy]

        # AprilTag detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.target_pose_pub = self.create_publisher(PoseStamped, '/apriltag/target_pose', 10)
        self.target_detected_pub = self.create_publisher(Bool, '/apriltag/detected', 10)
        self.debug_image_pub = self.create_publisher(Image, '/apriltag/debug_image', 10)

        self.get_logger().info(f'AprilTag Detector started (target ID: {self.target_id})')

    def image_callback(self, msg):
        """Detect AprilTags in camera image"""
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect tags
        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size
        )

        target_found = False

        for result in results:
            if result.tag_id == self.target_id:
                target_found = True

                # Extract pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera'

                # Translation (position)
                pose_msg.pose.position.x = result.pose_t[2][0]  # Forward (Z in camera frame)
                pose_msg.pose.position.y = -result.pose_t[0][0] # Left/right (X in camera frame)
                pose_msg.pose.position.z = -result.pose_t[1][0] # Up/down (Y in camera frame)

                # Rotation (orientation) - convert to quaternion if needed
                # For now, we mainly care about distance

                self.target_pose_pub.publish(pose_msg)

                # Draw detection on image
                corners = result.corners.astype(int)
                cv2.polylines(cv_image, [corners], True, (0, 255, 0), 2)
                cv2.putText(
                    cv_image,
                    f'ID {result.tag_id}: {pose_msg.pose.position.x:.2f}m',
                    (corners[0][0], corners[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

        # Publish detection status
        self.target_detected_pub.publish(Bool(data=target_found))

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.debug_image_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Phase 4: AprilTag Follow-Me

### 4.1 Follow-Me Controller

**File:** `ros2_ws/src/jetson_rover_bridge/scripts/apriltag_follower.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import numpy as np

class AprilTagFollower(Node):
    def __init__(self):
        super().__init__('apriltag_follower')

        # Parameters
        self.declare_parameter('target_distance', 2.0)    # meters
        self.declare_parameter('distance_tolerance', 0.3) # meters
        self.declare_parameter('max_linear_speed', 0.5)   # m/s
        self.declare_parameter('max_angular_speed', 0.8)  # rad/s
        self.declare_parameter('linear_kp', 0.5)          # P gain for distance
        self.declare_parameter('angular_kp', 1.0)         # P gain for angle

        self.target_dist = self.get_parameter('target_distance').value
        self.dist_tolerance = self.get_parameter('distance_tolerance').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value

        # State
        self.target_pose = None
        self.target_detected = False
        self.obstacle_danger = False
        self.enabled = False

        # Subscribers
        self.create_subscription(PoseStamped, '/apriltag/target_pose', self.pose_callback, 10)
        self.create_subscription(Bool, '/apriltag/detected', self.detected_callback, 10)
        self.create_subscription(Bool, '/obstacles/danger', self.obstacle_callback, 10)
        self.create_subscription(Bool, '/follow_me/enable', self.enable_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('AprilTag Follower started (disabled by default)')

    def pose_callback(self, msg):
        self.target_pose = msg

    def detected_callback(self, msg):
        self.target_detected = msg.data

    def obstacle_callback(self, msg):
        self.obstacle_danger = msg.data

    def enable_callback(self, msg):
        self.enabled = msg.data
        if self.enabled:
            self.get_logger().info('Follow-me ENABLED')
        else:
            self.get_logger().info('Follow-me DISABLED')
            # Stop rover
            self.cmd_vel_pub.publish(Twist())

    def control_loop(self):
        """Main control loop"""
        # Safety checks
        if not self.enabled:
            return

        if self.obstacle_danger:
            # STOP - obstacle in danger zone
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().warn('Follow-me: STOPPED due to obstacle', throttle_duration_sec=1.0)
            return

        if not self.target_detected or self.target_pose is None:
            # Can't see target - stop
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().warn('Follow-me: Target lost', throttle_duration_sec=1.0)
            return

        # Extract target position
        distance = self.target_pose.pose.position.x  # Forward distance
        lateral = self.target_pose.pose.position.y   # Left/right offset

        # Calculate errors
        distance_error = distance - self.target_dist
        angle_error = np.arctan2(lateral, distance)

        # Simple P controller
        linear_vel = self.linear_kp * distance_error
        angular_vel = self.angular_kp * angle_error

        # Clamp velocities
        linear_vel = np.clip(linear_vel, -self.max_linear, self.max_linear)
        angular_vel = np.clip(angular_vel, -self.max_angular, self.max_angular)

        # Stop if close enough
        if abs(distance_error) < self.dist_tolerance:
            linear_vel = 0.0

        # Publish command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

        self.get_logger().info(
            f'Following: dist={distance:.2f}m, lateral={lateral:.2f}m, '
            f'v={linear_vel:.2f}, ω={angular_vel:.2f}',
            throttle_duration_sec=1.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Implementation Timeline

### **Week 1: Obstacle Detection**
- [ ] Day 1-2: Implement ultrasonic detector
- [ ] Day 3-4: Implement LiDAR detector
- [ ] Day 5: Implement sensor fusion
- [ ] Day 6-7: Test in simulation

### **Week 2: AprilTag Follow-Me**
- [ ] Day 1-2: Install AprilTag libraries
- [ ] Day 3-4: Implement AprilTag detector
- [ ] Day 5-6: Implement follower controller
- [ ] Day 7: Integration testing

### **Week 3: Real-World Testing**
- [ ] Day 1-2: Deploy to Jetson
- [ ] Day 3-4: Calibrate sensors
- [ ] Day 5-6: Outdoor testing
- [ ] Day 7: Tuning and refinement

---

## Testing Strategy

### Simulation Testing (Desktop)

**1. Create test world with obstacles:**
```xml
<!-- In test_yard.world -->
<model name="obstacle_box">
  <static>true</static>
  <pose>2 0 0.5 0 0 0</pose>
  <!-- Box model -->
</model>
```

**2. Test each component:**
```bash
# Terminal 1: Launch simulation
ros2 launch jetson_rover_sim full_simulation.launch.py

# Terminal 2: Start obstacle detectors
ros2 run jetson_rover_bridge obstacle_detector_ultrasonic.py
ros2 run jetson_rover_bridge obstacle_detector_lidar.py
ros2 run jetson_rover_bridge obstacle_fusion.py

# Terminal 3: Monitor
ros2 topic echo /obstacles/danger
```

### Real Hardware Testing

**1. Static obstacle test:**
- Place box 1m in front
- Verify both sensors detect
- Verify fusion triggers danger

**2. Dynamic avoidance:**
- Drive toward obstacle
- Verify rover stops at safe distance

**3. AprilTag following:**
- Print AprilTag (tag36h11, ID 0)
- Mount on stick/backpack
- Enable follow-me
- Walk around, verify rover follows

---

## Configuration Files

### Launch File for Obstacle Avoidance

**File:** `ros2_ws/src/jetson_rover_bridge/launch/obstacle_avoidance.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Ultrasonic detector
        Node(
            package='jetson_rover_bridge',
            executable='obstacle_detector_ultrasonic.py',
            name='ultrasonic_detector',
            parameters=[{
                'danger_distance': 0.5,
                'warning_distance': 1.0,
            }]
        ),

        # LiDAR detector
        Node(
            package='jetson_rover_bridge',
            executable='obstacle_detector_lidar.py',
            name='lidar_detector',
            parameters=[{
                'danger_distance': 0.8,
                'warning_distance': 2.0,
                'field_of_view': 120.0,
            }]
        ),

        # Sensor fusion
        Node(
            package='jetson_rover_bridge',
            executable='obstacle_fusion.py',
            name='obstacle_fusion'
        ),

        # AprilTag detector
        Node(
            package='jetson_rover_bridge',
            executable='apriltag_detector.py',
            name='apriltag_detector',
            parameters=[{
                'target_tag_id': 0,
                'tag_size': 0.165,  # 6.5 inch tag
            }]
        ),

        # AprilTag follower
        Node(
            package='jetson_rover_bridge',
            executable='apriltag_follower.py',
            name='apriltag_follower',
            parameters=[{
                'target_distance': 2.0,
                'max_linear_speed': 0.5,
                'max_angular_speed': 0.8,
            }]
        ),
    ])
```

---

## Next Steps

1. **Create the Python files** in `ros2_ws/src/jetson_rover_bridge/scripts/`
2. **Make them executable:** `chmod +x *.py`
3. **Test in simulation first** on your desktop
4. **Deploy to Jetson** when ready
5. **Print AprilTag** - I can generate one for you
6. **Outdoor testing** with safety protocols

**Want me to help you create these files and test them in simulation first?**
