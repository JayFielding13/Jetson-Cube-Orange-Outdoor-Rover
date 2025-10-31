#!/usr/bin/env python3
"""
Simple LiDAR Visualization Test
Based exactly on working rplidar_realtime.py to isolate visualization issues
"""

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

class SimpleLidarTest:
    def __init__(self, port='/dev/ttyUSB1'):
        self.port = port
        self.lidar = None
        self.data_queue = queue.Queue(maxsize=5)
        self.running = False
        
        # Setup plot with rover-oriented display
        self.fig, self.ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
        self.ax.set_ylim(0, 6000)
        self.ax.set_title('Simple LiDAR Test - Rover Oriented\n(0° = Forward, 90° = Right, 180° = Back, 270° = Left)')
        self.ax.grid(True)
        
        # Set 0° to point up (forward direction for rover)
        self.ax.set_theta_zero_location('N')  # North = up = forward
        self.ax.set_theta_direction(-1)  # Clockwise to match rover orientation
        
        # Add direction labels
        self.ax.set_thetagrids([0, 90, 180, 270], ['Forward', 'Right', 'Back', 'Left'])
        
        # Initialize empty plot
        self.line, = self.ax.plot([], [], 'ro', markersize=1.5, alpha=0.8)
        
        print("📊 Simple LiDAR test initialized")
    
    def scan_thread(self):
        """Background thread to continuously collect scan data - EXACT COPY"""
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
                        # Fix mirroring: flip left/right by negating angle
                        corrected_angle = -angle if angle != 0 else 0
                        angles.append(math.radians(corrected_angle))
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
                            print(f"📊 Test Scan {scan_count}: {len(angles)} points -> Queue size: {self.data_queue.qsize()}")
                            
                    except queue.Full:
                        pass  # Skip this scan if queue is full
                        
        except Exception as e:
            print(f"❌ Scan thread error: {e}")
            import traceback
            traceback.print_exc()
    
    def animate(self, frame):
        """Animation function EXACTLY like working version"""
        try:
            angles, distances = self.data_queue.get_nowait()
            self.line.set_data(angles, distances)
            print(f"🎬 Animation updated: {len(angles)} points displayed")
            return self.line,
        except queue.Empty:
            return self.line,
    
    def run_test(self):
        try:
            print(f"🔗 Connecting to RPLidar A1 on {self.port}")
            
            # Add reset and multiple connection attempts
            connection_attempts = 0
            max_attempts = 3
            
            while connection_attempts < max_attempts:
                try:
                    if connection_attempts > 0:
                        print(f"🔄 Connection attempt {connection_attempts + 1}/{max_attempts}")
                        time.sleep(2)  # Wait between attempts
                    
                    self.lidar = RPLidar(self.port, baudrate=115200, timeout=3)
                    
                    # Try to reset the LiDAR
                    try:
                        self.lidar.reset()
                        time.sleep(2)
                        print("🔄 LiDAR reset successful")
                    except:
                        print("⚠️ LiDAR reset failed, continuing...")
                    
                    break  # Exit loop if successful
                    
                except Exception as e:
                    print(f"❌ Connection attempt {connection_attempts + 1} failed: {e}")
                    connection_attempts += 1
                    if connection_attempts >= max_attempts:
                        raise e
                    
                    # Try to cleanup before next attempt
                    try:
                        if hasattr(self, 'lidar') and self.lidar:
                            self.lidar.disconnect()
                    except:
                        pass
            
            # Get device info
            info = self.lidar.get_info()
            print(f"📋 Device info: {info}")
            
            # Get health status
            health = self.lidar.get_health()
            print(f"💚 Health: {health}")
            
            # Start motor and wait
            print("🔄 Starting motor...")
            self.lidar.start_motor()
            time.sleep(3)
            
            print("📊 Starting simple LiDAR visualization test...")
            print("📊 Close the plot window to stop")
            
            self.running = True
            
            # Start background scanning thread EXACTLY like working version
            scan_thread = threading.Thread(target=self.scan_thread)
            scan_thread.daemon = True
            scan_thread.start()
            
            # Start animation EXACTLY like working version
            ani = animation.FuncAnimation(
                self.fig, self.animate, interval=100, blit=True, cache_frame_data=False
            )
            
            # Use BLOCKING show like working version
            plt.show()
            
        except Exception as e:
            print(f"❌ Connection error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
            if self.lidar:
                try:
                    self.lidar.stop()
                    self.lidar.stop_motor()
                    self.lidar.disconnect()
                    print("✅ Disconnected")
                except:
                    pass

def main():
    print("🧪 Simple LiDAR Visualization Test")
    print("=" * 40)
    
    test = SimpleLidarTest('/dev/ttyUSB1')
    test.run_test()

if __name__ == "__main__":
    main()