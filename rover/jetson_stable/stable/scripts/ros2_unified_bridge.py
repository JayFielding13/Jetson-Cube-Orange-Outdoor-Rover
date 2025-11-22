#!/usr/bin/env python3
"""
ROS 2 Unified Bridge - HTTP Server with MAVROS2 Control
Subscribes to ROS 2 sensor topics and provides control via MAVROS2 services
For Mobile RTK Control Module integration

EXPERIMENTAL VERSION - Manual Joystick Control + RTK Status Tracking
Changes:
- Added /api/velocity endpoint for direct RC override control
- Added safety timeout (500ms) for manual control
- Added manual_mode tracking
- Added RTK corrections status tracking via /mavros/gps_rtk/send_rtcm subscription
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, CompressedImage, BatteryState
from mavros_msgs.msg import State, Waypoint, GPSRAW, OverrideRCIn, RTCM
from mavros_msgs.srv import CommandLong, SetMode, WaypointPush, WaypointClear, WaypointSetCurrent
from flask import Flask, jsonify, Response, request
from flask_cors import CORS
import threading
import base64
import json
import time
from datetime import datetime

app = Flask(__name__)
CORS(app)

class UnifiedBridge(Node):
    def __init__(self):
        super().__init__('unified_bridge')

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

        # Manual control state
        self.manual_mode = False
        self.last_velocity_time = time.time()
        self.velocity_timeout = 0.5  # 500ms timeout
        self.velocity_lock = threading.Lock()

        # === RTK Corrections Tracking ===
        self.rtk_corrections_count = 0
        self.rtk_bytes_received = 0
        self.last_rtk_time = None
        self.rtk_connected = False
        self.rtk_timeout = 5.0  # Consider disconnected after 5s of no data
        self.rtk_lock = threading.Lock()

        # QoS profile for MAVROS topics (BEST_EFFORT reliability)
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for RTCM topic (RELIABLE to match the forwarder)
        rtcm_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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

        # Subscribe to compressed camera topic
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

        # Subscribe to MAVROS topics (with BEST_EFFORT QoS)
        self.gps_subscriber = self.create_subscription(
            GPSRAW,
            '/mavros/mavros/gps1/raw',
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

        # === Subscribe to RTCM corrections topic ===
        self.rtcm_subscriber = self.create_subscription(
            RTCM,
            '/mavros/gps_rtk/send_rtcm',
            self.rtcm_callback,
            rtcm_qos
        )

        # RC Override publisher for manual control
        self.rc_override_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )

        # MAVROS service clients
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.mission_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.mission_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.mission_set_current_client = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')

        # Safety timeout timer (check every 100ms)
        self.safety_timer = self.create_timer(0.1, self.check_velocity_timeout)
        
        # RTK status check timer (check every 1s)
        self.rtk_status_timer = self.create_timer(1.0, self.check_rtk_status)

        self.get_logger().info('Unified Bridge Node initialized (with Manual Control + RTK Tracking)')
        self.get_logger().info('Subscribed to /scan and /image_raw/compressed')
        self.get_logger().info('Subscribed to MAVROS: /mavros/gps1/raw, /battery, /state')
        self.get_logger().info('Subscribed to RTCM: /mavros/gps_rtk/send_rtcm')
        self.get_logger().info('RC Override publisher ready on /mavros/rc/override')
        self.get_logger().info('MAVROS service clients ready')

    # === RTK Corrections Callback ===
    def rtcm_callback(self, msg):
        """Track incoming RTCM correction messages"""
        with self.rtk_lock:
            self.rtk_corrections_count += 1
            self.rtk_bytes_received += len(msg.data)
            self.last_rtk_time = time.time()
            self.rtk_connected = True

    # === RTK Status Check (runs every 1s) ===
    def check_rtk_status(self):
        """Check if RTK corrections are still being received"""
        with self.rtk_lock:
            if self.last_rtk_time is not None:
                elapsed = time.time() - self.last_rtk_time
                if elapsed > self.rtk_timeout:
                    if self.rtk_connected:
                        self.get_logger().warn(f'RTK corrections timeout ({elapsed:.1f}s since last)')
                    self.rtk_connected = False

    # === Get RTK Status ===
    def get_rtk_status(self):
        """Return current RTK corrections status"""
        with self.rtk_lock:
            if self.last_rtk_time is not None:
                age_ms = int((time.time() - self.last_rtk_time) * 1000)
            else:
                age_ms = None
            
            return {
                'connected': self.rtk_connected,
                'source': 'MQTT' if self.rtk_connected else 'NONE',
                'corrections_received': self.rtk_corrections_count,
                'bytes_received': self.rtk_bytes_received,
                'last_correction_age_ms': age_ms
            }

    # Safety timeout callback
    def check_velocity_timeout(self):
        """Stop if no velocity command received for timeout period"""
        with self.velocity_lock:
            if self.manual_mode:
                elapsed = time.time() - self.last_velocity_time
                if elapsed > self.velocity_timeout:
                    self.get_logger().warn(f'Velocity timeout ({elapsed:.2f}s) - stopping')
                    self.send_rc_override(0.0, 0.0)  # Stop
                    self.manual_mode = False

    # RC Override for velocity control
    def send_rc_override(self, linear: float, angular: float) -> bool:
        """
        Send RC Override command for velocity control
        
        Args:
            linear: -1.0 (reverse) to +1.0 (forward)
            angular: -1.0 (right) to +1.0 (left)
        
        Returns:
            True if command was sent
        """
        # ArduRover RC channel mapping (typical):
        # Channel 1: Steering (1000-2000, 1500=center)
        # Channel 3: Throttle (1000-2000, 1500=neutral)
        
        # Convert -1.0 to +1.0 range to PWM (1000-2000)
        # Neutral/center is 1500
        throttle_pwm = int(1500 + (linear * 500))  # 1000-2000
        steering_pwm = int(1500 + (angular * 500))  # 1000-2000
        
        # Clamp to valid PWM range
        throttle_pwm = max(1000, min(2000, throttle_pwm))
        steering_pwm = max(1000, min(2000, steering_pwm))
        
        # Create RC Override message
        msg = OverrideRCIn()
        # channels array: [ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ...]
        # Set to 0 or 65535 to release channel (let RC take over)
        # We only override channels 1 (steering) and 3 (throttle)
        msg.channels = [
            steering_pwm,  # Channel 1: Steering
            65535,         # Channel 2: Not used (release)
            throttle_pwm,  # Channel 3: Throttle
            65535,         # Channel 4: Not used (release)
            65535,         # Channel 5: Not used (release)
            65535,         # Channel 6: Not used (release)
            65535,         # Channel 7: Not used (release)
            65535,         # Channel 8: Not used (release)
            65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535  # 9-18
        ]
        
        self.rc_override_pub.publish(msg)
        
        # Update manual mode state
        with self.velocity_lock:
            if linear != 0.0 or angular != 0.0:
                self.manual_mode = True
                self.last_velocity_time = time.time()
        
        return True

    # Release RC Override (return control to RC/autopilot)
    def release_rc_override(self):
        """Release all RC override channels"""
        msg = OverrideRCIn()
        # Set all channels to 0 to release (some systems) or 65535
        msg.channels = [0] * 18
        self.rc_override_pub.publish(msg)
        
        with self.velocity_lock:
            self.manual_mode = False

    def scan_callback(self, msg):
        """Process incoming LiDAR scan data"""
        with self.scan_lock:
            ranges = list(msg.ranges)
            valid_ranges = []
            for r in ranges:
                if msg.range_min < r < msg.range_max:
                    valid_ranges.append(round(r, 3))
                else:
                    valid_ranges.append(None)

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
            self.latest_image = {
                'timestamp': datetime.now().isoformat(),
                'format': msg.format,
                'data': base64.b64encode(bytes(msg.data)).decode('utf-8')
            }

    def gps_callback(self, msg):
        """Process incoming GPS RAW data from MAVROS (HERE 3+ GPS)"""
        with self.gps_lock:
            latitude = msg.lat / 1e7
            longitude = msg.lon / 1e7
            altitude = msg.alt / 1000.0
            hdop = (msg.eph / 1000.0) if msg.eph > 0 else 1.0
            fix_type_map = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6}
            fix_type = fix_type_map.get(msg.fix_type, 1)

            self.latest_gps = {
                'timestamp': datetime.now().isoformat(),
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude,
                'satellites': msg.satellites_visible,
                'fix_type': fix_type,
                'hdop': hdop,
                'eph': msg.eph,
                'epv': msg.epv,
                'h_acc': msg.h_acc,
                'v_acc': msg.v_acc,
                'vel': msg.vel,
                'cog': msg.cog
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

    def call_command_service(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        """Call MAVROS CommandLong service"""
        if not self.command_client.service_is_ready():
            for _ in range(10):
                time.sleep(0.1)
                if self.command_client.service_is_ready():
                    break
            else:
                return {'success': False, 'error': 'Command service not available'}

        request = CommandLong.Request()
        request.command = command
        request.param1 = param1
        request.param2 = param2
        request.param3 = param3
        request.param4 = param4
        request.param5 = param5
        request.param6 = param6
        request.param7 = param7

        future = self.command_client.call_async(request)
        timeout = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if future.done():
            try:
                result = future.result()
                return {'success': result.success, 'result': result.result}
            except Exception as e:
                return {'success': False, 'error': f'Service call exception: {str(e)}'}
        else:
            return {'success': False, 'error': 'Service call timeout'}

    def call_set_mode_service(self, base_mode=0, custom_mode=''):
        """Call MAVROS SetMode service"""
        if not self.set_mode_client.service_is_ready():
            for _ in range(10):
                time.sleep(0.1)
                if self.set_mode_client.service_is_ready():
                    break
            else:
                return {'success': False, 'error': 'SetMode service not available'}

        request = SetMode.Request()
        request.base_mode = base_mode
        request.custom_mode = custom_mode

        future = self.set_mode_client.call_async(request)
        timeout = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if future.done():
            try:
                result = future.result()
                return {'success': result.mode_sent}
            except Exception as e:
                return {'success': False, 'error': f'Service call exception: {str(e)}'}
        else:
            return {'success': False, 'error': 'Service call timeout'}

    def call_mission_push_service(self, waypoints_data):
        """Call MAVROS WaypointPush service"""
        if not self.mission_push_client.service_is_ready():
            for _ in range(10):
                time.sleep(0.1)
                if self.mission_push_client.service_is_ready():
                    break
            else:
                return {'success': False, 'error': 'Mission push service not available'}

        request = WaypointPush.Request()
        request.waypoints = []

        for wp_data in waypoints_data:
            wp = Waypoint()
            wp.frame = wp_data.get('frame', 3)
            wp.command = wp_data.get('command', 16)
            wp.is_current = wp_data.get('is_current', False)
            wp.autocontinue = wp_data.get('autocontinue', True)
            wp.param1 = wp_data.get('param1', 0.0)
            wp.param2 = wp_data.get('param2', 0.0)
            wp.param3 = wp_data.get('param3', 0.0)
            wp.param4 = wp_data.get('param4', 0.0)
            wp.x_lat = wp_data['latitude']
            wp.y_long = wp_data['longitude']
            wp.z_alt = wp_data.get('altitude', 0.0)
            request.waypoints.append(wp)

        future = self.mission_push_client.call_async(request)
        timeout = 10.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if future.done():
            try:
                result = future.result()
                return {'success': result.success, 'wp_transfered': result.wp_transfered}
            except Exception as e:
                return {'success': False, 'error': f'Service call exception: {str(e)}'}
        else:
            return {'success': False, 'error': 'Service call timeout'}

# Global bridge node
bridge_node = None

#========== SENSOR DATA ENDPOINTS ==========

@app.route('/api/lidar', methods=['GET'])
def get_lidar_data():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    with bridge_node.scan_lock:
        if bridge_node.latest_scan is None:
            return jsonify({'error': 'No scan data available'}), 503
        return jsonify(bridge_node.latest_scan)

@app.route('/api/camera', methods=['GET'])
def get_camera_data():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    with bridge_node.image_lock:
        if bridge_node.latest_image is None:
            return jsonify({'error': 'No camera data available'}), 503
        return jsonify(bridge_node.latest_image)

@app.route('/api/gps', methods=['GET'])
def get_gps_data():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    with bridge_node.gps_lock:
        if bridge_node.latest_gps is None:
            return jsonify({'error': 'No GPS data available'}), 503
        return jsonify(bridge_node.latest_gps)

@app.route('/api/battery', methods=['GET'])
def get_battery_data():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    with bridge_node.battery_lock:
        if bridge_node.latest_battery is None:
            return jsonify({'error': 'No battery data available'}), 503
        return jsonify(bridge_node.latest_battery)

@app.route('/api/state', methods=['GET'])
def get_state_data():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    with bridge_node.state_lock:
        if bridge_node.latest_state is None:
            return jsonify({'error': 'No state data available'}), 503
        return jsonify(bridge_node.latest_state)

@app.route('/api/health', methods=['GET'])
def health_check():
    if bridge_node is None:
        return jsonify({'success': False, 'error': 'Bridge not initialized'}), 503
    return jsonify({
        'success': True,
        'status': 'online',
        'timestamp': datetime.now().isoformat(),
        'manual_control': True,
        'rtk_tracking': True  # Indicate RTK tracking is available
    })

@app.route('/api/status', methods=['GET'])
def get_status():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503

    status = {
        'success': True,
        'lidar': bridge_node.latest_scan is not None,
        'camera': bridge_node.latest_image is not None,
        'gps': bridge_node.latest_gps is not None,
        'battery': bridge_node.latest_battery is not None,
        'state': bridge_node.latest_state is not None,
        'timestamp': datetime.now().isoformat(),
        'manual_mode': bridge_node.manual_mode
    }

    # Include RTK corrections status
    status['rtk'] = bridge_node.get_rtk_status()

    if bridge_node.latest_gps:
        with bridge_node.gps_lock:
            status['gps_summary'] = {
                'latitude': bridge_node.latest_gps['latitude'],
                'longitude': bridge_node.latest_gps['longitude'],
                'altitude': bridge_node.latest_gps['altitude']
            }
            status['gps'] = bridge_node.latest_gps.copy()

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

    if bridge_node.latest_scan:
        status['lidar_summary'] = {
            'num_points': bridge_node.latest_scan['num_points'],
            'min_distance': bridge_node.latest_scan['min_distance']
        }

    return jsonify(status)

#========== RTK STATUS ENDPOINT ==========

@app.route('/api/rtk', methods=['GET'])
def get_rtk_status():
    """Get RTK corrections status"""
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    
    rtk_status = bridge_node.get_rtk_status()
    rtk_status['success'] = True
    return jsonify(rtk_status)

#========== CONTROL ENDPOINTS ==========

@app.route('/api/arm', methods=['POST'])
def arm_vehicle():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    result = bridge_node.call_command_service(command=400, param1=1.0)
    return jsonify(result)

@app.route('/api/disarm', methods=['POST'])
def disarm_vehicle():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    # Release RC override before disarming
    bridge_node.release_rc_override()
    result = bridge_node.call_command_service(command=400, param1=0.0)
    return jsonify(result)

@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    data = request.get_json()
    if not data or 'mode' not in data:
        return jsonify({'error': 'Missing mode parameter'}), 400
    mode = data['mode']
    base_mode = data.get('base_mode', 0)
    result = bridge_node.call_set_mode_service(base_mode=base_mode, custom_mode=mode)
    return jsonify(result)

@app.route('/api/upload_mission', methods=['POST'])
def upload_mission():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    data = request.get_json()
    if not data or 'waypoints' not in data:
        return jsonify({'error': 'Missing waypoints parameter'}), 400
    waypoints = data['waypoints']
    result = bridge_node.call_mission_push_service(waypoints)
    return jsonify(result)

@app.route('/api/stop', methods=['POST'])
def emergency_stop():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    # Release RC override first
    bridge_node.release_rc_override()
    result = bridge_node.call_command_service(command=400, param1=0.0)
    if result.get('success'):
        return jsonify({'success': True, 'message': 'Emergency stop executed'})
    return jsonify(result)

@app.route('/api/pause', methods=['POST'])
def pause_vehicle():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    # Release RC override and set to HOLD mode
    bridge_node.release_rc_override()
    result = bridge_node.call_set_mode_service(custom_mode='HOLD')
    if result.get('success'):
        return jsonify({'success': True, 'message': 'Vehicle paused (HOLD mode)'})
    return jsonify(result)

@app.route('/api/cancel', methods=['POST'])
def cancel_mission():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    bridge_node.release_rc_override()
    result = bridge_node.call_mission_push_service([])
    if result.get('success'):
        mode_result = bridge_node.call_set_mode_service(custom_mode='MANUAL')
        return jsonify({'success': True, 'message': 'Mission cancelled, switched to MANUAL'})
    return jsonify(result)

@app.route('/api/target', methods=['POST'])
def send_target():
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503
    data = request.get_json()
    if not data or 'latitude' not in data or 'longitude' not in data:
        return jsonify({'error': 'Missing latitude or longitude parameter'}), 400

    waypoint = {
        'latitude': data['latitude'],
        'longitude': data['longitude'],
        'altitude': data.get('altitude', 10.0),
        'frame': 3,
        'command': 16,
        'is_current': True,
        'autocontinue': True
    }

    result = bridge_node.call_mission_push_service([waypoint])
    if result.get('success'):
        mode_result = bridge_node.call_set_mode_service(custom_mode='AUTO')
        return jsonify({'success': True, 'message': 'Target waypoint uploaded and AUTO mode set'})
    return jsonify(result)

#========== MANUAL CONTROL ENDPOINT ==========

@app.route('/api/velocity', methods=['POST'])
def send_velocity():
    """
    Direct velocity control for joystick/manual mode
    Sends RC Override commands to Cube Orange

    Request JSON:
    {
        "linear": 0.5,    # -1.0 to +1.0 (backward/forward)
        "angular": -0.3   # -1.0 to +1.0 (right/left turn)
    }
    """
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503

    data = request.json
    if not data:
        return jsonify({
            'success': False,
            'message': 'No data provided'
        }), 400

    # Check if vehicle is armed
    with bridge_node.state_lock:
        if bridge_node.latest_state and not bridge_node.latest_state.get('armed', False):
            return jsonify({
                'success': False,
                'message': 'Vehicle not armed - ARM first'
            }), 400

    # Get values with defaults
    linear = float(data.get('linear', 0.0))
    angular = float(data.get('angular', 0.0))

    # Clamp to valid range
    linear = max(-1.0, min(1.0, linear))
    angular = max(-1.0, min(1.0, angular))

    # Send RC Override
    success = bridge_node.send_rc_override(linear, angular)

    return jsonify({
        'success': success,
        'linear': linear,
        'angular': angular,
        'message': 'Velocity command sent' if success else 'Failed to send command'
    })

@app.route('/api/velocity/release', methods=['POST'])
def release_velocity():
    """Release manual control (return control to RC/autopilot)"""
    if bridge_node is None:
        return jsonify({'error': 'Bridge not initialized'}), 503

    bridge_node.release_rc_override()
    return jsonify({
        'success': True,
        'message': 'RC override released'
    })

def run_ros2():
    """Run ROS 2 node in background thread"""
    global bridge_node
    rclpy.init()
    bridge_node = UnifiedBridge()
    rclpy.spin(bridge_node)

if __name__ == '__main__':
    ros2_thread = threading.Thread(target=run_ros2, daemon=True)
    ros2_thread.start()
    time.sleep(2)

    print("=" * 70)
    print("ROS 2 Unified Bridge - MAVROS2 Integration")
    print("EXPERIMENTAL: Manual Control + RTK Status Tracking")
    print("=" * 70)
    print("Sensor Endpoints:")
    print("  GET  /api/lidar         - Latest LiDAR scan data")
    print("  GET  /api/camera        - Latest camera image (base64 JPEG)")
    print("  GET  /api/gps           - GPS position from flight controller")
    print("  GET  /api/battery       - Battery telemetry")
    print("  GET  /api/state         - Flight controller state")
    print("  GET  /api/status        - Comprehensive system status (includes RTK)")
    print("  GET  /api/rtk           - RTK corrections status")
    print("")
    print("Control Endpoints:")
    print("  POST /api/arm           - ARM the vehicle")
    print("  POST /api/disarm        - DISARM the vehicle")
    print("  POST /api/set_mode      - Set flight mode")
    print("  POST /api/upload_mission - Upload waypoint mission")
    print("")
    print("Manual Control Endpoints:")
    print("  POST /api/velocity      - Send velocity command (linear, angular)")
    print("  POST /api/velocity/release - Release RC override")
    print("")
    print("RTK Status Tracking:")
    print("  Subscribed to: /mavros/gps_rtk/send_rtcm")
    print("  Timeout: 5 seconds (marks as disconnected)")
    print("")
    print("Listening on: http://0.0.0.0:5001")
    print("Safety timeout: 500ms (stops if no velocity command received)")
    print("=" * 70)

    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
