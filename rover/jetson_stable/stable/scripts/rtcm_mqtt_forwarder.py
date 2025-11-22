#!/usr/bin/env python3
"""
RTCM MQTT-to-MAVROS2 Forwarder
Subscribes to RTCM corrections from RTK base station via MQTT
and forwards them to MAVROS2 for injection into Cube Orange + HERE 3+
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import RTCM
import paho.mqtt.client as mqtt
import logging
from datetime import datetime

# Configuration
MQTT_BROKER = '100.66.67.11'  # RTKPi base station
MQTT_PORT = 1883
MQTT_TOPIC_CORRECTIONS = 'rtk/base/corrections'
MQTT_TOPIC_STATUS = 'rtk/base/status'

class RTCMForwarder(Node):
    def __init__(self):
        super().__init__('rtcm_mqtt_forwarder')

        # Setup logging
        self.get_logger().info('Initializing RTCM MQTT Forwarder...')

        # MAVROS QoS profile (RELIABLE to match MAVROS gps_rtk subscriber)
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher to MAVROS2 RTK topic
        self.rtcm_publisher = self.create_publisher(
            RTCM,
            '/mavros/gps_rtk/send_rtcm',
            mavros_qos
        )

        # Statistics
        self.bytes_received = 0
        self.messages_sent = 0
        self.last_correction_time = None

        # Status reporting timer (every 10 seconds)
        self.status_timer = self.create_timer(10.0, self.report_status)

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "jetson-rtcm-forwarder")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

        # Connect to MQTT broker
        try:
            self.get_logger().info(f'Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}...')
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')
            raise

        self.get_logger().info('RTCM Forwarder initialized')

    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties):
        """Called when MQTT connection is established"""
        if reason_code == 0:
            self.get_logger().info(f'Connected to MQTT broker successfully')
            # Subscribe to corrections topic
            client.subscribe(MQTT_TOPIC_CORRECTIONS)
            client.subscribe(MQTT_TOPIC_STATUS)
            self.get_logger().info(f'Subscribed to {MQTT_TOPIC_CORRECTIONS}')
        else:
            self.get_logger().error(f'Failed to connect to MQTT broker: {reason_code}')

    def on_mqtt_disconnect(self, client, userdata, flags, reason_code, properties):
        """Called when MQTT connection is lost"""
        self.get_logger().warn(f'Disconnected from MQTT broker: {reason_code}')
        if reason_code != 0:
            self.get_logger().info('Attempting to reconnect...')

    def on_mqtt_message(self, client, userdata, msg):
        """Called when MQTT message is received"""
        if msg.topic == MQTT_TOPIC_CORRECTIONS:
            # Received RTCM correction data
            self.forward_rtcm(msg.payload)
        elif msg.topic == MQTT_TOPIC_STATUS:
            # Log base station status
            self.get_logger().info(f'Base station status: {msg.payload.decode()}', throttle_duration_sec=60.0)

    def forward_rtcm(self, rtcm_data):
        """Forward RTCM data to MAVROS2"""
        try:
            # Create RTCM message
            rtcm_msg = RTCM()
            rtcm_msg.header.stamp = self.get_clock().now().to_msg()
            rtcm_msg.header.frame_id = 'gps'

            # Convert bytes to list of integers (ROS message format)
            rtcm_msg.data = list(rtcm_data)

            # Publish to MAVROS2
            self.rtcm_publisher.publish(rtcm_msg)

            # Update statistics
            self.bytes_received += len(rtcm_data)
            self.messages_sent += 1
            self.last_correction_time = datetime.now()

        except Exception as e:
            self.get_logger().error(f'Error forwarding RTCM data: {e}')

    def report_status(self):
        """Report forwarding statistics"""
        if self.messages_sent > 0:
            self.get_logger().info(
                f'RTCM Stats: {self.bytes_received} bytes, '
                f'{self.messages_sent} messages, '
                f'last: {self.last_correction_time.strftime("%H:%M:%S") if self.last_correction_time else "never"}'
            )
            # Reset counters
            self.bytes_received = 0
            self.messages_sent = 0
        else:
            self.get_logger().warn('No RTCM corrections received in last 10 seconds')

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down RTCM forwarder...')
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        forwarder = RTCMForwarder()
        rclpy.spin(forwarder)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fatal error: {e}')
    finally:
        if rclpy.ok():
            forwarder.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
