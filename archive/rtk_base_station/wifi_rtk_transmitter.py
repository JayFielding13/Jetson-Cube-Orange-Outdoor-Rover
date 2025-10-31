#!/usr/bin/env python3
"""
WiFi RTK Transmitter
Connects to RTCM server and broadcasts corrections via WiFi
Much more reliable and higher bandwidth than SiK radio
"""

import socket
import time
import threading
from datetime import datetime

class WiFiRTKTransmitter:
    def __init__(self):
        # RTCM server connection
        self.rtcm_host = 'localhost'
        self.rtcm_port = 2101
        self.rtcm_socket = None

        # WiFi server settings
        self.wifi_port = 8101  # Port for RTK corrections over WiFi
        self.server_socket = None
        self.client_connections = []

        # Statistics
        self.bytes_received = 0
        self.bytes_transmitted = 0
        self.packets_sent = 0
        self.connected_clients = 0

    def connect_rtcm_server(self):
        """Connect to RTCM correction server"""
        try:
            print(f"🔌 Connecting to RTCM server at {self.rtcm_host}:{self.rtcm_port}...")
            self.rtcm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.rtcm_socket.settimeout(10)
            self.rtcm_socket.connect((self.rtcm_host, self.rtcm_port))

            # Test receiving data
            self.rtcm_socket.settimeout(3)
            test_data = self.rtcm_socket.recv(32)
            if test_data:
                print(f"✅ Connected to RTCM server - received {len(test_data)} bytes")
                # Reconnect to reset stream position
                self.rtcm_socket.close()
                self.rtcm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.rtcm_socket.settimeout(1)
                self.rtcm_socket.connect((self.rtcm_host, self.rtcm_port))
                return True
            else:
                print("⚠️  Connected but no data available")
                return False
        except Exception as e:
            print(f"❌ Failed to connect to RTCM server: {e}")
            if self.rtcm_socket:
                self.rtcm_socket.close()
                self.rtcm_socket = None
            return False

    def start_wifi_server(self):
        """Start WiFi server to accept mobile RTK connections"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.wifi_port))
            self.server_socket.listen(5)
            self.server_socket.settimeout(1.0)  # Non-blocking accept

            print(f"📡 WiFi RTK server listening on port {self.wifi_port}")
            print("🔗 Waiting for mobile RTK connections...")
            return True
        except Exception as e:
            print(f"❌ Failed to start WiFi server: {e}")
            return False

    def accept_connections(self):
        """Accept new client connections in background"""
        while True:
            try:
                client_socket, client_address = self.server_socket.accept()
                client_socket.settimeout(5.0)
                self.client_connections.append(client_socket)
                self.connected_clients += 1
                print(f"🔗 New client connected: {client_address[0]} (Total: {self.connected_clients})")
            except socket.timeout:
                continue
            except Exception as e:
                break

    def remove_dead_connections(self):
        """Remove disconnected clients"""
        active_connections = []
        for client in self.client_connections:
            try:
                # Test if connection is still alive
                client.send(b'')
                active_connections.append(client)
            except:
                client.close()
                self.connected_clients -= 1

        self.client_connections = active_connections

    def broadcast_corrections(self):
        """Main transmission loop - broadcast to all connected clients"""
        print("🚀 Starting WiFi RTK correction broadcast...")
        print("📡 Broadcasting RTCM corrections to connected clients")
        print("=" * 60)

        last_stats_time = time.time()
        last_cleanup_time = time.time()

        try:
            while True:
                # Receive data from RTCM server
                try:
                    data = self.rtcm_socket.recv(4096)  # Larger buffer for WiFi
                    if not data:
                        print("⚠️  RTCM server connection closed")
                        break

                    self.bytes_received += len(data)

                    # Broadcast to all connected clients
                    if self.client_connections:
                        dead_connections = []
                        for client in self.client_connections:
                            try:
                                client.send(data)
                                self.bytes_transmitted += len(data)
                            except Exception as e:
                                dead_connections.append(client)

                        # Remove dead connections
                        for dead_client in dead_connections:
                            try:
                                dead_client.close()
                            except:
                                pass
                            if dead_client in self.client_connections:
                                self.client_connections.remove(dead_client)
                                self.connected_clients -= 1

                        if dead_connections:
                            print(f"🔌 Removed {len(dead_connections)} disconnected clients")

                        self.packets_sent += 1

                    # Print statistics every 5 seconds
                    current_time = time.time()
                    if current_time - last_stats_time >= 5.0:
                        self.print_statistics()
                        last_stats_time = current_time

                    # Clean up connections every 30 seconds
                    if current_time - last_cleanup_time >= 30.0:
                        self.remove_dead_connections()
                        last_cleanup_time = current_time

                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"❌ Transmission error: {e}")
                    break

        except KeyboardInterrupt:
            print("\n🛑 Broadcast stopped by user")
        except Exception as e:
            print(f"❌ Unexpected error: {e}")
        finally:
            self.cleanup()

    def print_statistics(self):
        """Print transmission statistics"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        rate_kbps = (self.bytes_transmitted / 1024) / max(1, time.time() - getattr(self, 'start_time', time.time()))
        print(f"[{timestamp}] 📊 Clients: {self.connected_clients} | "
              f"RX: {self.bytes_received:,} bytes | "
              f"TX: {self.bytes_transmitted:,} bytes | "
              f"Rate: {rate_kbps:.1f} KB/s | "
              f"Packets: {self.packets_sent:,}")

    def cleanup(self):
        """Clean up connections"""
        print("\n🧹 Cleaning up connections...")

        if self.rtcm_socket:
            self.rtcm_socket.close()
            print("🔌 RTCM server connection closed")

        for client in self.client_connections:
            try:
                client.close()
            except:
                pass
        self.client_connections.clear()

        if self.server_socket:
            self.server_socket.close()
            print("📡 WiFi server socket closed")

        print(f"✅ Disconnected {self.connected_clients} clients")

    def run(self):
        """Run the WiFi RTK transmitter"""
        print("🚀 WiFi RTK Transmitter")
        print("📍 High-bandwidth RTK correction broadcasting over WiFi")
        print("💡 Much more reliable than SiK radio!")
        print()

        self.start_time = time.time()

        # Connect to RTCM server
        if not self.connect_rtcm_server():
            return False

        # Start WiFi server
        if not self.start_wifi_server():
            return False

        # Start connection acceptor thread
        connection_thread = threading.Thread(target=self.accept_connections, daemon=True)
        connection_thread.start()

        # Start broadcasting
        self.broadcast_corrections()
        return True

def main():
    transmitter = WiFiRTKTransmitter()
    try:
        transmitter.run()
    except KeyboardInterrupt:
        print("\n🛑 WiFi RTK Transmitter stopped")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()