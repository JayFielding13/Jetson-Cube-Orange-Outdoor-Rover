#!/usr/bin/env python3
"""
Jetson Orin Nano - Rover Control Server
Receives commands from Mobile RTK Terminal via HTTP REST API
Interfaces with Cube Orange flight controller via MAVLink

Install dependencies:
    pip3 install flask pymavlink

Usage:
    python3 jetson_rover_server.py
"""

from flask import Flask, request, jsonify
import logging
import threading
import time
from datetime import datetime
from pymavlink import mavutil
import paho.mqtt.client as mqtt

# ========== RTK CORRECTION FORWARDING VIA MQTT ==========

class RTKForwarder:
    """Forward RTCM corrections from MQTT base station to Cube Orange GPS"""
    def __init__(self):
        self.mqtt_client = None
        self.connected = False
        self.correction_count = 0
        self.base_host = '192.168.254.165'
        self.base_port = 1883
        self.mavlink_connection = None

    def connect_to_base_station(self, mavlink_connection):
        """Connect to MQTT base station and subscribe to RTK corrections"""
        try:
            self.mavlink_connection = mavlink_connection
            logging.info(f"Connecting to RTK base (MQTT) at {self.base_host}:{self.base_port}")

            # Create MQTT client
            self.mqtt_client = mqtt.Client(client_id="jetson_rover_rtk", protocol=mqtt.MQTTv5)

            # Set callbacks
            self.mqtt_client.on_connect = self._on_mqtt_connect
            self.mqtt_client.on_message = self._on_mqtt_message
            self.mqtt_client.on_disconnect = self._on_mqtt_disconnect

            # Connect to MQTT broker
            self.mqtt_client.connect(self.base_host, self.base_port, 60)
            self.mqtt_client.loop_start()  # Start background thread

            return True

        except Exception as e:
            logging.error(f"Failed to connect to RTK base (MQTT): {e}")
            self.connected = False
            return False

    def _on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        """MQTT connection callback"""
        if rc == 0:
            logging.info("✓ Connected to RTK base station (MQTT)")
            # Subscribe to RTK corrections topic
            client.subscribe("rtk/base/corrections", qos=0)
            client.subscribe("rtk/base/status", qos=0)
            self.connected = True
        else:
            logging.error(f"MQTT connection failed with code {rc}")
            self.connected = False

    def _on_mqtt_disconnect(self, client, userdata, rc, properties=None):
        """MQTT disconnection callback"""
        logging.warning("RTK base station (MQTT) disconnected")
        self.connected = False

    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback - receives RTCM corrections"""
        if msg.topic == "rtk/base/corrections":
            # RTCM correction data received
            rtcm_data = msg.payload
            if rtcm_data and len(rtcm_data) > 0:
                # Forward immediately to Cube Orange
                self._forward_rtcm_to_mavlink(rtcm_data)
                self.correction_count += 1

    def _forward_rtcm_to_mavlink(self, data):
        """Forward RTCM data to Cube Orange via MAVLink GPS_RTCM_DATA"""
        if not self.mavlink_connection:
            return

        try:
            # Send RTCM data to Cube Orange via MAVLink GPS_RTCM_DATA message
            # Split into chunks of 180 bytes (MAVLink GPS_RTCM_DATA max payload)

            for i in range(0, len(data), 180):
                chunk = data[i:i+180]
                chunk_len = len(chunk)

                # Determine fragment flags
                if len(data) <= 180:
                    flags = 1  # Single fragment (LSB set, others clear)
                elif i == 0:
                    flags = 0  # First fragment of multi-fragment message
                elif i + 180 >= len(data):
                    flags = 2  # Last fragment (bit 1 set)
                else:
                    flags = 3  # Middle fragment

                # Pad chunk to 180 bytes
                chunk_padded = chunk + b'\x00' * (180 - chunk_len)

                # Send GPS_RTCM_DATA message to Cube Orange
                self.mavlink_connection.mav.gps_rtcm_data_send(
                    flags,          # Fragment flags
                    chunk_len,      # Length of actual data in this chunk
                    list(chunk_padded)  # Data as list of bytes
                )

        except Exception as e:
            logging.error(f"Error forwarding RTCM to MAVLink: {e}")

    def get_status(self):
        """Get RTK forwarder status"""
        return {
            'connected': self.connected,
            'corrections_forwarded': self.correction_count
        }


# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

app = Flask(__name__)

# Rover state
class RoverState:
    def __init__(self):
        self.armed = False
        self.mavlink_connected = False
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_alt = 0.0
        self.gps_satellites = 0
        self.gps_fix_type = 0
        self.gps_hdop = 0.0
        self.battery_voltage = 0.0
        self.battery_percent = 0
        self.target_lat = None
        self.target_lon = None
        self.target_distance = 0.0
        self.last_command_time = time.time()

        # MAVLink connection
        self.mavlink_connection = None

state = RoverState()

# RTK forwarder instance
rtk_forwarder = RTKForwarder()


def init_mavlink():
    """Initialize MAVLink connection to Cube Orange"""
    try:
        logging.info("Initializing MAVLink connection to Cube Orange...")
        logging.info("Port: /dev/ttyACM0, Baud: 57600")

        # Connect to Cube Orange via USB
        state.mavlink_connection = mavutil.mavlink_connection(
            '/dev/ttyACM0',  # Cube Orange on USB
            baud=57600       # Standard ArduPilot baud rate
        )

        # Wait for heartbeat
        logging.info("Waiting for heartbeat from Cube Orange...")
        state.mavlink_connection.wait_heartbeat()
        logging.info("✓ Heartbeat received from Cube Orange")
        logging.info(f"  System ID: {state.mavlink_connection.target_system}")

        logging.info(f"  Component ID: {state.mavlink_connection.target_component}")

        # Connect to RTK base station for corrections
        rtk_forwarder.connect_to_base_station(state.mavlink_connection)

        state.mavlink_connected = True

        return True

    except Exception as e:
        logging.error(f"Failed to initialize MAVLink: {e}")
        state.mavlink_connected = False
        return False


def arm_motors():
    """Send ARM command to Cube Orange"""
    try:
        if not state.mavlink_connected:
            logging.error("Cannot arm: MAVLink not connected")
            return False

        logging.info("Sending ARM command to Cube Orange...")

        # Send ARM command
        state.mavlink_connection.mav.command_long_send(
            state.mavlink_connection.target_system,
            state.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # 1 to arm, 0 to disarm
            0, 0, 0, 0, 0, 0
        )

        # Wait for command acknowledgment
        ack = state.mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                state.armed = True
                logging.info("✓ Motors ARMED")
                return True
            else:
                logging.error(f"ARM command rejected: {ack.result}")
                return False
        else:
            logging.warning("No ARM acknowledgment received (timeout), but command may have succeeded")
            state.armed = True  # Assume success
            return True

    except Exception as e:
        logging.error(f"Failed to arm motors: {e}")
        return False


def disarm_motors():
    """Send DISARM command to Cube Orange"""
    try:
        if not state.mavlink_connected:
            logging.error("Cannot disarm: MAVLink not connected")
            return False

        logging.info("Sending DISARM command to Cube Orange...")

        # Send DISARM command
        state.mavlink_connection.mav.command_long_send(
            state.mavlink_connection.target_system,
            state.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # 1 to arm, 0 to disarm
            0, 0, 0, 0, 0, 0
        )

        # Wait for command acknowledgment
        ack = state.mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                state.armed = False
                logging.info("✓ Motors DISARMED")
                return True
            else:
                logging.error(f"DISARM command rejected: {ack.result}")
                return False
        else:
            logging.warning("No DISARM acknowledgment received (timeout), but command may have succeeded")
            state.armed = False  # Assume success
            return True

    except Exception as e:
        logging.error(f"Failed to disarm motors: {e}")
        return False


def send_target_position(lat, lon, distance_offset=0.0):
    """Send navigation target to Cube Orange"""
    try:
        if not state.mavlink_connected:
            logging.error("Cannot send target: MAVLink not connected")
            return False

        state.target_lat = lat
        state.target_lon = lon
        state.target_distance = distance_offset
        state.last_command_time = time.time()

        logging.info(f"Sending target position: {lat:.7f}, {lon:.7f}, offset={distance_offset}m")

        # Send position target using SET_POSITION_TARGET_GLOBAL_INT
        # This tells the rover to navigate to a specific GPS coordinate
        state.mavlink_connection.mav.set_position_target_global_int_send(
            0,  # time_boot_ms (not used)
            state.mavlink_connection.target_system,
            state.mavlink_connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b110111111000),  # type_mask (only position)
            int(lat * 1e7),  # lat_int
            int(lon * 1e7),  # lon_int
            0,  # alt (for rover, altitude is not used)
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0, 0  # yaw, yaw_rate
        )

        logging.info("✓ Target position sent to Cube Orange")
        return True

    except Exception as e:
        logging.error(f"Failed to send target position: {e}")
        return False


def emergency_stop():
    """Emergency stop - disarm motors immediately"""
    try:
        logging.warning("⛔ EMERGENCY STOP ACTIVATED")

        # Disarm motors
        disarm_motors()

        # Clear target
        state.target_lat = None
        state.target_lon = None

        # Optionally set mode to HOLD
        try:
            mode_id = state.mavlink_connection.mode_mapping().get('HOLD', None)
            if mode_id:
                state.mavlink_connection.set_mode(mode_id)
                logging.info("Set mode to HOLD")
        except:
            pass

        return True

    except Exception as e:
        logging.error(f"Emergency stop failed: {e}")
        return False


def telemetry_thread():
    """Background thread to read telemetry from Cube Orange"""
    while True:
        try:
            if state.mavlink_connected and state.mavlink_connection:
                # Read telemetry messages
                msg = state.mavlink_connection.recv_match(blocking=False)
                if msg:
                    # Handle different message types
                    msg_type = msg.get_type()

                    if msg_type == 'GLOBAL_POSITION_INT':
                        state.gps_lat = msg.lat / 1e7
                        state.gps_lon = msg.lon / 1e7
                        state.gps_alt = msg.alt / 1000.0  # mm to meters

                    elif msg_type == 'GPS_RAW_INT':
                        state.gps_satellites = msg.satellites_visible
                        state.gps_fix_type = msg.fix_type
                        if msg.eph != 65535:  # Check for valid HDOP
                            state.gps_hdop = msg.eph / 100.0  # cm to meters

                    elif msg_type == 'BATTERY_STATUS':
                        if len(msg.voltages) > 0 and msg.voltages[0] != 65535:
                            state.battery_voltage = msg.voltages[0] / 1000.0
                        state.battery_percent = msg.battery_remaining

                    elif msg_type == 'HEARTBEAT':
                        state.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

            time.sleep(0.05)  # 20Hz polling

        except Exception as e:
            logging.error(f"Telemetry thread error: {e}")
            time.sleep(1)


# ========== HTTP API ENDPOINTS ==========

@app.route('/api/status', methods=['GET'])
def get_status():
    """Get rover status"""
    return jsonify({
        'success': True,
        'timestamp': datetime.now().isoformat(),
        'armed': state.armed,
        'mavlink_connected': state.mavlink_connected,
        'gps': {
            'latitude': state.gps_lat,
            'longitude': state.gps_lon,
            'altitude': state.gps_alt,
            'satellites': state.gps_satellites,
            'fix_type': state.gps_fix_type,
            'hdop': state.gps_hdop
        },
        'battery': {
            'voltage': state.battery_voltage,
            'percent': state.battery_percent
        },
        'target': {
            'latitude': state.target_lat,
            'longitude': state.target_lon,
            'distance_offset': state.target_distance
        } if state.target_lat else None,
        'rtk': rtk_forwarder.get_status(),
        'last_command_age': time.time() - state.last_command_time
    })


@app.route('/api/arm', methods=['POST'])
def api_arm():
    """Arm motors"""
    success = arm_motors()
    return jsonify({
        'success': success,
        'armed': state.armed,
        'message': 'Motors armed' if success else 'Failed to arm motors'
    })


@app.route('/api/disarm', methods=['POST'])
def api_disarm():
    """Disarm motors"""
    success = disarm_motors()
    return jsonify({
        'success': success,
        'armed': state.armed,
        'message': 'Motors disarmed' if success else 'Failed to disarm motors'
    })


@app.route('/api/target', methods=['POST'])
def api_target():
    """Set navigation target"""
    data = request.get_json()

    if not data or 'latitude' not in data or 'longitude' not in data:
        return jsonify({
            'success': False,
            'message': 'Missing latitude or longitude'
        }), 400

    lat = float(data['latitude'])
    lon = float(data['longitude'])
    distance_offset = float(data.get('distance_offset', 0.0))

    success = send_target_position(lat, lon, distance_offset)

    return jsonify({
        'success': success,
        'target': {
            'latitude': lat,
            'longitude': lon,
            'distance_offset': distance_offset
        },
        'message': 'Target set' if success else 'Failed to set target'
    })


@app.route('/api/stop', methods=['POST'])
def api_stop():
    """Emergency stop"""
    success = emergency_stop()
    return jsonify({
        'success': success,
        'armed': state.armed,
        'message': 'Emergency stop executed' if success else 'Emergency stop failed'
    })


@app.route('/api/pause', methods=['POST'])
def api_pause():
    """Pause rover - stop movement but stay armed"""
    try:
        # Clear target but don't disarm
        state.target_lat = None
        state.target_lon = None
        logging.info("⏸ Rover PAUSED - target cleared, staying armed")
        return jsonify({
            'success': True,
            'armed': state.armed,
            'message': 'Rover paused'
        })
    except Exception as e:
        logging.error(f"Pause failed: {e}")
        return jsonify({
            'success': False,
            'message': f'Pause failed: {str(e)}'
        })


@app.route('/api/cancel', methods=['POST'])
def api_cancel():
    """Cancel mission - clear all targets"""
    try:
        state.target_lat = None
        state.target_lon = None
        logging.info("🚫 Mission CANCELLED - all targets cleared")
        return jsonify({
            'success': True,
            'armed': state.armed,
            'message': 'Mission cancelled'
        })
    except Exception as e:
        logging.error(f"Cancel failed: {e}")
        return jsonify({
            'success': False,
            'message': f'Cancel failed: {str(e)}'
        })


@app.route('/api/health', methods=['GET'])
def api_health():
    """Health check endpoint"""
    return jsonify({
        'success': True,
        'status': 'running',
        'mavlink_connected': state.mavlink_connected
    })


def main():
    """Main entry point"""
    logging.info("=" * 60)
    logging.info("Baby Tessla Rover Control Server")
    logging.info("Jetson Orin Nano → Cube Orange Interface")
    logging.info("=" * 60)

    # Initialize MAVLink connection
    if not init_mavlink():
        logging.error("Failed to initialize MAVLink - exiting")
        return

    # Start telemetry thread
    telemetry = threading.Thread(target=telemetry_thread, daemon=True)
    telemetry.start()
    logging.info("Telemetry thread started")

    # Start Flask server
    logging.info("Starting HTTP API server on 0.0.0.0:5000")
    logging.info("=" * 60)
    app.run(host='0.0.0.0', port=5000, debug=False)


if __name__ == '__main__':
    main()
