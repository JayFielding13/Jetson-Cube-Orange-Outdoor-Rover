#!/usr/bin/env python3
"""
Pi-to-Pi Communication Module
================================================================================
Handles communication between Navigation Pi and Visualization Pi for the 
dual-Pi rover setup.

This module provides:
- NavigationDataPublisher: Sends telemetry from Navigation Pi to Visualization Pi
- VisualizationDataReceiver: Receives telemetry on Visualization Pi
- Thread-safe data handling with automatic reconnection
- JSON-based message protocol for reliability

Part of the Mini Rover Development Project - Dashboard V3 Dual Pi
Author: Developed with Jay Fielding
Version: 1.0
Date: 2025-01-22
================================================================================
"""

import socket
import json
import threading
import time
import queue
from datetime import datetime
from typing import Dict, Any, Optional, Callable
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class NavigationDataPublisher:
    """
    Publisher for sending navigation telemetry from Navigation Pi to Visualization Pi
    
    Features:
    - Automatic connection management with reconnection
    - Thread-safe data queuing
    - JSON message protocol
    - Connection status monitoring
    """
    
    def __init__(self, viz_pi_ip: str = "192.168.1.11", port: int = 8888):
        self.viz_pi_ip = viz_pi_ip
        self.port = port
        self.socket = None
        self.connected = False
        self.running = False
        
        # Thread-safe data queue
        self.data_queue = queue.Queue(maxsize=100)
        self.send_thread = None
        
        # Statistics
        self.stats = {
            'messages_sent': 0,
            'connection_attempts': 0,
            'last_successful_send': None,
            'total_disconnections': 0
        }
        
        logger.info(f"NavigationDataPublisher initialized for {viz_pi_ip}:{port}")
    
    def start(self):
        """Start the publisher with automatic connection management"""
        if self.running:
            return
            
        self.running = True
        self.send_thread = threading.Thread(target=self._send_worker, daemon=True)
        self.send_thread.start()
        logger.info("NavigationDataPublisher started")
    
    def stop(self):
        """Stop the publisher and clean up resources"""
        self.running = False
        if self.send_thread:
            self.send_thread.join(timeout=2.0)
        self._disconnect()
        logger.info("NavigationDataPublisher stopped")
    
    def _connect(self) -> bool:
        """Establish connection to Visualization Pi"""
        try:
            if self.socket:
                self.socket.close()
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)  # 5 second timeout
            self.socket.connect((self.viz_pi_ip, self.port))
            self.connected = True
            self.stats['connection_attempts'] += 1
            logger.info(f"✅ Connected to Visualization Pi at {self.viz_pi_ip}:{self.port}")
            return True
            
        except Exception as e:
            self.connected = False
            self.stats['connection_attempts'] += 1
            logger.warning(f"❌ Failed to connect to Visualization Pi: {e}")
            return False
    
    def _disconnect(self):
        """Disconnect from Visualization Pi"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        
        if self.connected:
            self.stats['total_disconnections'] += 1
            logger.info("Disconnected from Visualization Pi")
        
        self.connected = False
    
    def send_navigation_data(self, nav_data: Dict[str, Any]) -> bool:
        """
        Queue navigation data for sending to Visualization Pi
        
        Args:
            nav_data: Dictionary containing navigation telemetry
            
        Returns:
            bool: True if queued successfully, False if queue full
        """
        if not self.running:
            return False
        
        # Add timestamp if not present
        if 'timestamp' not in nav_data:
            nav_data['timestamp'] = time.time()
        
        try:
            # Try to queue data (non-blocking)
            self.data_queue.put_nowait(nav_data)
            return True
        except queue.Full:
            logger.warning("Data queue full - dropping navigation data")
            return False
    
    def _send_worker(self):
        """Background thread worker for sending data"""
        reconnect_delay = 1.0
        
        while self.running:
            # Ensure connection
            if not self.connected:
                if self._connect():
                    reconnect_delay = 1.0  # Reset delay on successful connection
                else:
                    time.sleep(reconnect_delay)
                    reconnect_delay = min(reconnect_delay * 1.5, 30.0)  # Exponential backoff
                    continue
            
            try:
                # Get data with timeout
                try:
                    nav_data = self.data_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                
                # Send data
                if self._send_data(nav_data):
                    self.stats['messages_sent'] += 1
                    self.stats['last_successful_send'] = time.time()
                    self.data_queue.task_done()
                else:
                    # Re-queue data if send failed
                    try:
                        self.data_queue.put_nowait(nav_data)
                    except queue.Full:
                        pass  # Drop data if queue full
                    
                    self._disconnect()
                    
            except Exception as e:
                logger.error(f"Send worker error: {e}")
                self._disconnect()
                time.sleep(1.0)
    
    def _send_data(self, nav_data: Dict[str, Any]) -> bool:
        """Send single data packet"""
        try:
            if not self.connected or not self.socket:
                return False
            
            # Serialize and send
            data_json = json.dumps(nav_data) + '\n'
            self.socket.send(data_json.encode('utf-8'))
            return True
            
        except Exception as e:
            logger.warning(f"Send failed: {e}")
            return False
    
    def get_connection_status(self) -> Dict[str, Any]:
        """Get current connection status and statistics"""
        return {
            'connected': self.connected,
            'running': self.running,
            'target': f"{self.viz_pi_ip}:{self.port}",
            'queue_size': self.data_queue.qsize(),
            'stats': self.stats.copy()
        }


class VisualizationDataReceiver:
    """
    Receiver for navigation telemetry on Visualization Pi
    
    Features:
    - Multi-client server support
    - Thread-safe callback system
    - Automatic message parsing
    - Connection monitoring
    """
    
    def __init__(self, port: int = 8888):
        self.port = port
        self.server_socket = None
        self.running = False
        self.data_callback = None
        self.connection_callback = None
        
        # Active connections
        self.connections = {}
        self.connection_counter = 0
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'total_connections': 0,
            'active_connections': 0,
            'last_message_time': None
        }
        
        logger.info(f"VisualizationDataReceiver initialized on port {port}")
    
    def start_server(self, data_callback: Callable[[Dict[str, Any]], None], 
                    connection_callback: Optional[Callable[[str], None]] = None):
        """
        Start the receiver server
        
        Args:
            data_callback: Function called when navigation data received
            connection_callback: Optional function called on connection events
        """
        if self.running:
            return
        
        self.data_callback = data_callback
        self.connection_callback = connection_callback
        
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.port))
            self.server_socket.listen(5)
            self.running = True
            
            logger.info(f"📡 Visualization server listening on port {self.port}")
            
            # Start accept thread
            accept_thread = threading.Thread(target=self._accept_connections, daemon=True)
            accept_thread.start()
            
        except Exception as e:
            logger.error(f"Failed to start server: {e}")
            raise
    
    def stop_server(self):
        """Stop the receiver server and clean up"""
        self.running = False
        
        # Close all client connections
        for conn_id, conn_info in list(self.connections.items()):
            try:
                conn_info['socket'].close()
            except:
                pass
        self.connections.clear()
        
        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        logger.info("Visualization server stopped")
    
    def _accept_connections(self):
        """Accept incoming connections from Navigation Pi"""
        while self.running:
            try:
                if not self.server_socket:
                    break
                
                client_socket, addr = self.server_socket.accept()
                self.connection_counter += 1
                conn_id = f"nav_pi_{self.connection_counter}"
                
                # Store connection info
                self.connections[conn_id] = {
                    'socket': client_socket,
                    'address': addr,
                    'connected_time': time.time(),
                    'messages_received': 0
                }
                
                self.stats['total_connections'] += 1
                self.stats['active_connections'] = len(self.connections)
                
                logger.info(f"✅ Navigation Pi connected: {addr} (ID: {conn_id})")
                
                if self.connection_callback:
                    self.connection_callback(f"Connected: {addr}")
                
                # Start data handler for this connection
                data_thread = threading.Thread(
                    target=self._handle_client_data, 
                    args=(conn_id,), 
                    daemon=True
                )
                data_thread.start()
                
            except Exception as e:
                if self.running:
                    logger.error(f"Accept connection error: {e}")
                break
    
    def _handle_client_data(self, conn_id: str):
        """Handle data from a specific client connection"""
        conn_info = self.connections.get(conn_id)
        if not conn_info:
            return
        
        client_socket = conn_info['socket']
        buffer = ""
        
        try:
            while self.running and conn_id in self.connections:
                # Receive data
                data = client_socket.recv(1024)
                if not data:
                    break
                
                buffer += data.decode('utf-8')
                
                # Process complete messages (newline-delimited)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            nav_data = json.loads(line)
                            
                            # Update statistics
                            self.stats['messages_received'] += 1
                            self.stats['last_message_time'] = time.time()
                            conn_info['messages_received'] += 1
                            
                            # Call data callback
                            if self.data_callback:
                                self.data_callback(nav_data)
                                
                        except json.JSONDecodeError as e:
                            logger.warning(f"Invalid JSON from {conn_id}: {e}")
                        except Exception as e:
                            logger.error(f"Data callback error: {e}")
                            
        except Exception as e:
            logger.warning(f"Client {conn_id} error: {e}")
        
        finally:
            # Cleanup connection
            self._cleanup_connection(conn_id)
    
    def _cleanup_connection(self, conn_id: str):
        """Clean up a disconnected client"""
        if conn_id in self.connections:
            conn_info = self.connections[conn_id]
            try:
                conn_info['socket'].close()
            except:
                pass
            
            del self.connections[conn_id]
            self.stats['active_connections'] = len(self.connections)
            
            logger.info(f"🔌 Navigation Pi disconnected: {conn_id}")
            
            if self.connection_callback:
                self.connection_callback(f"Disconnected: {conn_id}")
    
    def get_server_status(self) -> Dict[str, Any]:
        """Get current server status and statistics"""
        return {
            'running': self.running,
            'port': self.port,
            'active_connections': list(self.connections.keys()),
            'stats': self.stats.copy()
        }
    
    def broadcast_to_navigation_pis(self, message: Dict[str, Any]) -> int:
        """
        Send a message to all connected Navigation Pis
        
        Args:
            message: Dictionary to send
            
        Returns:
            int: Number of successful sends
        """
        if not self.connections:
            return 0
        
        message_json = json.dumps(message) + '\n'
        successful_sends = 0
        
        for conn_id, conn_info in list(self.connections.items()):
            try:
                conn_info['socket'].send(message_json.encode('utf-8'))
                successful_sends += 1
            except Exception as e:
                logger.warning(f"Failed to send to {conn_id}: {e}")
                self._cleanup_connection(conn_id)
        
        return successful_sends


# Utility functions for easy integration
def create_navigation_publisher(viz_pi_ip: str = "192.168.1.11") -> NavigationDataPublisher:
    """Create and start a navigation data publisher"""
    publisher = NavigationDataPublisher(viz_pi_ip)
    publisher.start()
    return publisher

def create_visualization_receiver(data_callback: Callable[[Dict[str, Any]], None]) -> VisualizationDataReceiver:
    """Create and start a visualization data receiver"""
    receiver = VisualizationDataReceiver()
    receiver.start_server(data_callback)
    return receiver