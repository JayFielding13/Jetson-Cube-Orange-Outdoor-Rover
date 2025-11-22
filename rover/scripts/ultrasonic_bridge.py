#!/usr/bin/env python3
"""
Jetson Ultrasonic Sensor Bridge - ROS2 Node

This node reads raw ultrasonic sensor data from the ESP32 via USB serial,
performs all filtering and processing, and publishes to ROS2 topics.

Published Topics:
  - /ultrasonic/pointcloud (sensor_msgs/PointCloud2) - For RViz visualization
  - /ultrasonic/front (sensor_msgs/Range)
  - /ultrasonic/corner_left (sensor_msgs/Range)
  - /ultrasonic/corner_right (sensor_msgs/Range)
  - /ultrasonic/side_left (sensor_msgs/Range)
  - /ultrasonic/side_right (sensor_msgs/Range)
  - /ultrasonic/rear (sensor_msgs/Range)

Configuration:
  - Serial port: /dev/ttyUSB0 (or set via ROS parameter)
  - Baud rate: 115200
  - Update rate: 10 Hz (matches ESP32 output)
"""

import json
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial

from sensor_msgs.msg import Range, PointCloud2, PointField
from std_msgs.msg import Header
import struct


class UltrasonicBridge(Node):
    def __init__(self):
        super().__init__('ultrasonic_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('filter_window', 5)  # Moving average window
        self.declare_parameter('outlier_threshold', 0.5)  # Reject readings > 0.5m difference
        self.declare_parameter('timeout', 2.0)  # Serial timeout in seconds

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.filter_window = self.get_parameter('filter_window').value
        self.outlier_threshold = self.get_parameter('outlier_threshold').value
        self.timeout = self.get_parameter('timeout').value

        # Sensor configuration (name, angle in degrees)
        self.sensors = {
            'front': 0,
            'corner_left': -45,
            'corner_right': 45,
            'side_left': -90,
            'side_right': 90,
            'rear': 180
        }

        # Moving average filters for each sensor
        self.filters = {name: deque(maxlen=self.filter_window) for name in self.sensors.keys()}

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create Range publishers for each sensor
        self.range_publishers = {}
        for name in self.sensors.keys():
            topic = f'/ultrasonic/{name}'
            self.range_publishers[name] = self.create_publisher(Range, topic, qos_profile)

        # Create PointCloud2 publisher for visualization
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/ultrasonic/pointcloud',
            qos_profile
        )

        # Initialize serial connection
        self.serial = None
        self.connect_serial()

        # Statistics
        self.message_count = 0
        self.error_count = 0
        self.last_stats_time = time.time()

        # Create timer for reading serial data (10 Hz)
        self.timer = self.create_timer(0.1, self.read_and_publish)

        self.get_logger().info('Ultrasonic bridge node started')
        self.get_logger().info(f'Serial port: {self.serial_port} @ {self.baud_rate} baud')
        self.get_logger().info(f'Filter window: {self.filter_window} samples')

    def connect_serial(self):
        """Connect to ESP32 via serial port"""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to {self.serial_port}')

            # Read and log startup message
            if self.serial.in_waiting:
                startup_msg = self.serial.readline().decode('utf-8', errors='ignore').strip()
                self.get_logger().info(f'ESP32: {startup_msg}')

        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
            self.get_logger().error('Retrying in 5 seconds...')
            time.sleep(5)
            self.connect_serial()

    def read_and_publish(self):
        """Read sensor data from serial and publish to ROS2 topics"""
        if not self.serial or not self.serial.is_open:
            self.get_logger().warn('Serial connection lost, reconnecting...')
            self.connect_serial()
            return

        try:
            # Read line from serial
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()

                # Parse JSON
                try:
                    data = json.loads(line)

                    # Validate message format
                    if 'timestamp' not in data or 'sensors' not in data:
                        return

                    # Process sensor readings
                    sensor_data = {}
                    for sensor in data['sensors']:
                        name = sensor['name']
                        distance = sensor['distance']
                        valid = sensor['valid']
                        angle = sensor['angle']

                        # Apply filtering
                        filtered_distance = self.filter_reading(name, distance, valid)

                        sensor_data[name] = {
                            'distance': filtered_distance,
                            'angle': angle,
                            'valid': filtered_distance is not None
                        }

                    # Publish Range messages
                    stamp = self.get_clock().now().to_msg()
                    for name, reading in sensor_data.items():
                        self.publish_range(name, reading, stamp)

                    # Publish PointCloud2
                    self.publish_pointcloud(sensor_data, stamp)

                    # Update statistics
                    self.message_count += 1
                    self.print_stats()

                except json.JSONDecodeError as e:
                    self.error_count += 1
                    if self.error_count % 10 == 0:
                        self.get_logger().warn(f'JSON decode error: {e}')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            self.serial.close()
            self.serial = None

    def filter_reading(self, sensor_name, distance, valid):
        """
        Apply moving average filter with outlier rejection

        Args:
            sensor_name: Name of the sensor
            distance: Raw distance reading
            valid: Whether the reading is valid

        Returns:
            Filtered distance or None if invalid
        """
        if not valid:
            return None

        filter_queue = self.filters[sensor_name]

        # Outlier rejection: compare to previous filtered value
        if len(filter_queue) > 0:
            avg = sum(filter_queue) / len(filter_queue)
            if abs(distance - avg) > self.outlier_threshold:
                # Outlier detected, skip this reading
                return avg  # Return previous filtered value

        # Add to filter queue
        filter_queue.append(distance)

        # Return moving average
        return sum(filter_queue) / len(filter_queue)

    def publish_range(self, sensor_name, reading, stamp):
        """Publish Range message for individual sensor"""
        msg = Range()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = f'ultrasonic_{sensor_name}'

        # Range message parameters for AJ-SR04M
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = math.radians(15)  # ~15° beam width
        msg.min_range = 0.02  # 2cm minimum
        msg.max_range = 6.0   # 6m maximum

        if reading['valid']:
            msg.range = reading['distance']
        else:
            msg.range = msg.max_range  # Invalid = max range

        self.range_publishers[sensor_name].publish(msg)

    def publish_pointcloud(self, sensor_data, stamp):
        """
        Publish PointCloud2 for visualization in RViz

        Convert polar coordinates (angle, distance) to Cartesian (x, y, z)
        with the rover as the origin.
        """
        points = []

        for name, reading in sensor_data.items():
            if not reading['valid']:
                continue

            angle_deg = reading['angle']
            distance = reading['distance']

            # Convert to radians
            angle_rad = math.radians(angle_deg)

            # Cartesian coordinates (front = +X, left = +Y, up = +Z)
            x = distance * math.cos(angle_rad)
            y = distance * math.sin(angle_rad)
            z = 0.0  # Sensors mounted at same height

            # Add point with intensity (0.0 - 1.0 based on distance)
            intensity = 1.0 - (distance / 6.0)  # Closer = brighter
            points.append([x, y, z, intensity])

        if not points:
            return  # No valid points

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'

        # Define point fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 16  # 4 floats * 4 bytes
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.width = len(points)
        msg.height = 1

        # Pack point data
        msg.data = b''.join([struct.pack('ffff', *point) for point in points])

        self.pointcloud_pub.publish(msg)

    def print_stats(self):
        """Print statistics every 10 seconds"""
        current_time = time.time()
        if current_time - self.last_stats_time >= 10.0:
            hz = self.message_count / 10.0
            self.get_logger().info(
                f'Stats: {self.message_count} messages ({hz:.1f} Hz), '
                f'{self.error_count} errors'
            )
            self.message_count = 0
            self.error_count = 0
            self.last_stats_time = current_time

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = UltrasonicBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
