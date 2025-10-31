#!/usr/bin/env python3
import os
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from rplidar import RPLidar
import math
import time
import threading
import queue

# Automatically set DISPLAY environment variable for X11 forwarding
os.environ['DISPLAY'] = '192.168.254.14:0.0'

class RealtimeLidarVisualizer:
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        self.lidar = None
        self.data_queue = queue.Queue(maxsize=5)
        self.running = False
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
        self.ax.set_ylim(0, 6000)
        self.ax.set_title('RPLidar A1 Real-time Visualization')
        self.ax.grid(True)
        
        # Set theta (angle) zero location to top and direction to clockwise
        self.ax.set_theta_zero_location('N')  # North (top)
        self.ax.set_theta_direction(-1)  # Clockwise
        
        # Initialize empty plot
        self.line, = self.ax.plot([], [], 'ro', markersize=1.5, alpha=0.8)
    
    def scan_thread(self):
        """Background thread to continuously collect scan data"""
        try:
            scan_count = 0
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                
                scan_count += 1
                angles = []
                distances = []
                
                for measurement in scan:
                    quality, angle, distance = measurement
                    if distance > 100 and distance < 8000:  # Filter noise and max range
                        angles.append(math.radians(angle))
                        distances.append(distance)
                
                if len(angles) > 20:  # Only update if we have enough points
                    try:
                        # Add new data to queue, remove old if full
                        if self.data_queue.full():
                            try:
                                self.data_queue.get_nowait()
                            except queue.Empty:
                                pass
                        
                        self.data_queue.put((angles, distances), block=False)
                        
                        if scan_count % 5 == 0:
                            print(f"Scan {scan_count}: {len(angles)} points")
                            
                    except queue.Full:
                        pass  # Skip this scan if queue is full
                        
        except Exception as e:
            print(f"Scan thread error: {e}")
            import traceback
            traceback.print_exc()
    
    def animate(self, frame):
        """Animation function called by matplotlib"""
        try:
            angles, distances = self.data_queue.get_nowait()
            self.line.set_data(angles, distances)
            return self.line,
        except queue.Empty:
            return self.line,
    
    def connect_and_run(self):
        try:
            print(f"Connecting to RPLidar A1 on {self.port}")
            self.lidar = RPLidar(self.port, baudrate=115200, timeout=3)
            
            # Get device info
            info = self.lidar.get_info()
            print(f"Device info: {info}")
            
            # Get health status
            health = self.lidar.get_health()
            print(f"Health: {health}")
            
            # Start motor and wait
            print("Starting motor...")
            self.lidar.start_motor()
            time.sleep(3)
            
            print("Starting real-time visualization...")
            print("Close the plot window to stop")
            
            self.running = True
            
            # Start background scanning thread
            scan_thread = threading.Thread(target=self.scan_thread)
            scan_thread.daemon = True
            scan_thread.start()
            
            # Start animation
            ani = animation.FuncAnimation(
                self.fig, self.animate, interval=100, blit=True, cache_frame_data=False
            )
            
            plt.show()
            
        except Exception as e:
            print(f"Connection error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
            if self.lidar:
                try:
                    self.lidar.stop()
                    self.lidar.stop_motor()
                    self.lidar.disconnect()
                    print("Disconnected")
                except:
                    pass

def main():
    print("Real-time RPLidar A1 Visualizer (Improved)")
    print("==========================================")
    
    visualizer = RealtimeLidarVisualizer('/dev/ttyUSB0')
    visualizer.connect_and_run()

if __name__ == "__main__":
    main()