#!/usr/bin/env python3
"""
ROS 2 Sensor Bridge - HTTP Server
Subscribes to ROS 2 sensor topics and exposes data via REST API
For Mobile RTK Control Module integration
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, CompressedImage, NavSatFix, BatteryState
from mavros_msgs.msg import State
from flask import Flask, jsonify, Response
from flask_cors import CORS
import threading
import base64
import json
from datetime import datetime

app = Flask(__name__)
CORS(app)

class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')
        
        # Latest sensor data storage
        self.latest_scan = None
        self.latest_image = None
        self.scan_lock = threading.Lock()
        self.image_lock = threading.Lock()

        # MAVROS data storage
        self.latest_gps = None
        self.latest_battery = None
        self.latest_state = None
        self.gps_lock = threading.Lock()
        self.battery_lock = threading.Lock()
        self.state_lock = threading.Lock()

        # QoS profile for MAVROS topics (BEST_EFFORT reliability)
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to LiDAR topic
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Subscribe to compressed camera topic (more efficient)
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

        # Subscribe to MAVROS topics (with BEST_EFFORT QoS)
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            mavros_qos
        )

        self.battery_subscriber = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            mavros_qos
        )

        self.state_subscriber = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            mavros_qos
        )

        self.get_logger().info('Sensor Bridge Node initialized')
        self.get_logger().info('Subscribed to /scan and /image_raw/compressed')
        self.get_logger().info('Subscribed to MAVROS: /global_position/global, /battery, /state')
        
    def scan_callback(self, msg):
        """Process incoming LiDAR scan data"""
        with self.scan_lock:
            # Convert to simplified JSON-friendly format
            ranges = list(msg.ranges)
            
            # Filter out invalid readings (inf, nan)
            valid_ranges = []
            for r in ranges:
                if msg.range_min < r < msg.range_max:
                    valid_ranges.append(round(r, 3))  # Round to 3 decimals
                else:
                    valid_ranges.append(None)
            
            # Calculate statistics
            valid_values = [r for r in valid_ranges if r is not None]
            min_distance = min(valid_values) if valid_values else None
            
            self.latest_scan = {
                'timestamp': datetime.now().isoformat(),
                'angle_min': round(msg.angle_min, 4),
                'angle_max': round(msg.angle_max, 4),
                'angle_increment': round(msg.angle_increment, 6),
                'range_min': round(msg.range_min, 2),
                'range_max': round(msg.range_max, 2),
                'ranges': valid_ranges,
                'min_distance': round(min_distance, 3) if min_distance else None,
                'num_points': len(valid_values)
            }
    
    def image_callback(self, msg):
        """Process incoming camera image data"""
        with self.image_lock:
            # Store compressed JPEG data as base64
            self.latest_image = {
                'timestamp': datetime.now().isoformat(),
                'format': msg.format,
                'data': base64.b64encode(bytes(msg.data)).decode('utf-8')
            }

    def gps_callback(self, msg):
        """Process incoming GPS data from MAVROS"""
        with self.gps_lock:
            self.latest_gps = {
                'timestamp': datetime.now().isoformat(),
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude,
                'status': {
                    'status': msg.status.status,  # -1=NO_FIX, 0=FIX, 1=SBAS_FIX, 2=GBAS_FIX
                    'service': msg.status.service  # GPS service type
                },
                'position_covariance': list(msg.position_covariance),
                'position_covariance_type': msg.position_covariance_type
            }

    def battery_callback(self, msg):
        """Process incoming battery data from MAVROS"""
        with self.battery_lock:
            self.latest_battery = {
                'timestamp': datetime.now().isoformat(),
                'voltage': msg.voltage,
                'current': msg.current,
                'charge': msg.charge,
                'capacity': msg.capacity,
                'design_capacity': msg.design_capacity,
                'percentage': msg.percentage,
                'power_supply_status': msg.power_supply_status,
                'power_supply_health': msg.power_supply_health,
                'power_supply_technology': msg.power_supply_technology,
                'present': msg.present,
                'cell_voltage': list(msg.cell_voltage) if msg.cell_voltage else [],
                'cell_temperature': list(msg.cell_temperature) if msg.cell_temperature else [],
                'location': msg.location,
                'serial_number': msg.serial_number
            }

    def state_callback(self, msg):
        """Process incoming state data from MAVROS"""
        with self.state_lock:
            self.latest_state = {
                'timestamp': datetime.now().isoformat(),
                'connected': msg.connected,
                'armed': msg.armed,
                'guided': msg.guided,
                'manual_input': msg.manual_input,
                'mode': msg.mode,
                'system_status': msg.system_status
            }

# Global bridge node
bridge_node = None

@app.route('/api/lidar', methods=['GET'])
def get_lidar_data():
    """Return latest LiDAR scan data"""
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    
    with bridge_node.scan_lock:
        if bridge_node.latest_scan is None:
            return jsonify({'error': 'No scan data available'}), 503
        return jsonify(bridge_node.latest_scan)

@app.route('/api/camera', methods=['GET'])
def get_camera_data():
    """Return latest camera image (base64 encoded JPEG)"""
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    
    with bridge_node.image_lock:
        if bridge_node.latest_image is None:
            return jsonify({'error': 'No camera data available'}), 503
        return jsonify(bridge_node.latest_image)

@app.route('/api/mavros/gps', methods=['GET'])
def get_mavros_gps():
    """Return latest GPS data from MAVROS"""
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503

    with bridge_node.gps_lock:
        if bridge_node.latest_gps is None:
            return jsonify({'error': 'No GPS data available'}), 503
        return jsonify(bridge_node.latest_gps)

@app.route('/api/mavros/battery', methods=['GET'])
def get_mavros_battery():
    """Return latest battery data from MAVROS"""
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503

    with bridge_node.battery_lock:
        if bridge_node.latest_battery is None:
            return jsonify({'error': 'No battery data available'}), 503
        return jsonify(bridge_node.latest_battery)

@app.route('/api/mavros/state', methods=['GET'])
def get_mavros_state():
    """Return latest state data from MAVROS"""
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503

    with bridge_node.state_lock:
        if bridge_node.latest_state is None:
            return jsonify({'error': 'No state data available'}), 503
        return jsonify(bridge_node.latest_state)

@app.route('/api/mavros/status', methods=['GET'])
def get_mavros_status():
    """Return status of all MAVROS data sources"""
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503

    status = {
        'gps': bridge_node.latest_gps is not None,
        'battery': bridge_node.latest_battery is not None,
        'state': bridge_node.latest_state is not None,
        'timestamp': datetime.now().isoformat()
    }

    # Add summary data if available
    if bridge_node.latest_gps:
        with bridge_node.gps_lock:
            status['gps_summary'] = {
                'latitude': bridge_node.latest_gps['latitude'],
                'longitude': bridge_node.latest_gps['longitude'],
                'altitude': bridge_node.latest_gps['altitude']
            }

    if bridge_node.latest_state:
        with bridge_node.state_lock:
            status['state_summary'] = {
                'connected': bridge_node.latest_state['connected'],
                'armed': bridge_node.latest_state['armed'],
                'mode': bridge_node.latest_state['mode']
            }

    if bridge_node.latest_battery:
        with bridge_node.battery_lock:
            status['battery_summary'] = {
                'voltage': bridge_node.latest_battery['voltage'],
                'percentage': bridge_node.latest_battery['percentage']
            }

    return jsonify(status)

@app.route('/api/sensors/status', methods=['GET'])
def get_sensors_status():
    """Return status of all sensors"""
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503

    status = {
        'lidar': bridge_node.latest_scan is not None,
        'camera': bridge_node.latest_image is not None,
        'mavros_gps': bridge_node.latest_gps is not None,
        'mavros_battery': bridge_node.latest_battery is not None,
        'mavros_state': bridge_node.latest_state is not None,
        'timestamp': datetime.now().isoformat()
    }

    if bridge_node.latest_scan:
        status['lidar_details'] = {
            'num_points': bridge_node.latest_scan['num_points'],
            'min_distance': bridge_node.latest_scan['min_distance']
        }

    if bridge_node.latest_gps:
        with bridge_node.gps_lock:
            status['mavros_gps_summary'] = {
                'latitude': bridge_node.latest_gps['latitude'],
                'longitude': bridge_node.latest_gps['longitude'],
                'altitude': bridge_node.latest_gps['altitude']
            }

    if bridge_node.latest_state:
        with bridge_node.state_lock:
            status['mavros_state_summary'] = {
                'connected': bridge_node.latest_state['connected'],
                'armed': bridge_node.latest_state['armed'],
                'mode': bridge_node.latest_state['mode']
            }

    return jsonify(status)

def run_ros2():
    """Run ROS 2 node in background thread"""
    global bridge_node
    rclpy.init()
    bridge_node = SensorBridge()
    rclpy.spin(bridge_node)

if __name__ == '__main__':
    # Start ROS 2 in background thread
    ros2_thread = threading.Thread(target=run_ros2, daemon=True)
    ros2_thread.start()
    
    # Wait for node to initialize
    import time
    time.sleep(2)
    
    print("=" * 60)
    print("ROS 2 Sensor Bridge - HTTP Server")
    print("=" * 60)
    print("Sensor Endpoints:")
    print("  GET /api/lidar              - Latest LiDAR scan data")
    print("  GET /api/camera             - Latest camera image (base64 JPEG)")
    print("  GET /api/sensors/status     - All sensor availability status")
    print("")
    print("MAVROS Endpoints:")
    print("  GET /api/mavros/gps         - GPS position from flight controller")
    print("  GET /api/mavros/battery     - Battery telemetry")
    print("  GET /api/mavros/state       - Flight controller state")
    print("  GET /api/mavros/status      - MAVROS data availability status")
    print("")
    print("Listening on: http://0.0.0.0:5001")
    print("=" * 60)
    
    # Run Flask server
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
