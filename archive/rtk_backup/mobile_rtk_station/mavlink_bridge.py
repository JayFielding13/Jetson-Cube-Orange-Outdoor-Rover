#!/usr/bin/env python3
import socket
import serial
import threading
import time

class MAVLinkBridge:
    def __init__(self):
        self.sik_port = '/dev/ttyUSB0'
        self.sik_baud = 57600
        self.tcp_port = 5760
        self.running = True
        
    def start_bridge(self):
        print('🌉 Starting MAVLink Bridge')
        print(f'📡 SiK Radio: {self.sik_port} @ {self.sik_baud}')
        print(f'🌐 TCP Server: 0.0.0.0:{self.tcp_port}')
        
        try:
            # Connect to SiK radio
            ser = serial.Serial(self.sik_port, self.sik_baud, timeout=1)
            
            # Create TCP server
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', self.tcp_port))
            server.listen(1)
            
            print('✅ Waiting for QGroundControl connection...')
            
            while self.running:
                client, addr = server.accept()
                print(f'📱 QGroundControl connected from {addr}')
                
                # Start bidirectional data forwarding
                threading.Thread(target=self.serial_to_tcp, args=(ser, client)).start()
                threading.Thread(target=self.tcp_to_serial, args=(client, ser)).start()
                
        except Exception as e:
            print(f'❌ Bridge error: {e}')
            
    def serial_to_tcp(self, ser, client):
        while self.running:
            try:
                data = ser.read(1024)
                if data:
                    client.send(data)
            except:
                break
                
    def tcp_to_serial(self, client, ser):
        while self.running:
            try:
                data = client.recv(1024)
                if data:
                    ser.write(data)
            except:
                break

if __name__ == '__main__':
    bridge = MAVLinkBridge()
    bridge.start_bridge()
