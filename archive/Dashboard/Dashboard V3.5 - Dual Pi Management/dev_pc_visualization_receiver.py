#!/usr/bin/env python3
"""
Development PC - Sensor Visualization Receiver
Connects to Companion Pi (192.168.254.70) to receive real-time sensor visualization

Displays:
- Real-time radar view from Companion Pi
- Sensor telemetry data
- Connection status and statistics
"""

import socket
import json
import base64
import io
import tkinter as tk
from tkinter import ttk
import threading
import time
from PIL import Image, ImageTk
from datetime import datetime
import sys

# Configuration
COMPANION_PI_IP = '192.168.254.70'
VISUALIZATION_PORT = 5555
RECONNECT_DELAY = 3  # seconds

class VisualizationReceiver:
    def __init__(self):
        self.root = tk.Tk()
        self.setup_gui()
        
        self.socket = None
        self.connected = False
        self.last_update = None
        self.frame_count = 0
        self.start_time = time.time()
        
        # Stats tracking
        self.connection_attempts = 0
        self.total_frames = 0
        
    def setup_gui(self):
        """Setup the main GUI window"""
        self.root.title("Rover Sensor Visualization - Development PC")
        self.root.geometry("1200x800")
        self.root.configure(bg='black')
        
        # Create main frame
        main_frame = tk.Frame(self.root, bg='black')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Title
        title_label = tk.Label(main_frame, 
                              text="🤖 Rover Dual Sensor Visualization", 
                              font=('Arial', 16, 'bold'),
                              fg='cyan', bg='black')
        title_label.pack(pady=10)
        
        # Status frame
        status_frame = tk.Frame(main_frame, bg='black')
        status_frame.pack(fill=tk.X, pady=5)
        
        # Connection status
        self.status_label = tk.Label(status_frame, 
                                   text="🔴 Disconnected", 
                                   font=('Arial', 12, 'bold'),
                                   fg='red', bg='black')
        self.status_label.pack(side=tk.LEFT)
        
        # Statistics
        self.stats_label = tk.Label(status_frame, 
                                  text="Frames: 0 | FPS: 0.0", 
                                  font=('Arial', 10),
                                  fg='white', bg='black')
        self.stats_label.pack(side=tk.RIGHT)
        
        # Control frame
        control_frame = tk.Frame(main_frame, bg='black')
        control_frame.pack(fill=tk.X, pady=5)
        
        # Connect button
        self.connect_btn = tk.Button(control_frame, 
                                   text="Connect to Companion Pi",
                                   command=self.toggle_connection,
                                   font=('Arial', 10, 'bold'),
                                   bg='green', fg='white')
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        # IP entry
        tk.Label(control_frame, text="IP:", fg='white', bg='black').pack(side=tk.LEFT, padx=5)
        self.ip_entry = tk.Entry(control_frame, width=15)
        self.ip_entry.insert(0, COMPANION_PI_IP)
        self.ip_entry.pack(side=tk.LEFT, padx=5)
        
        # Sensor data frame
        sensor_frame = tk.LabelFrame(main_frame, 
                                   text="Sensor Telemetry", 
                                   fg='cyan', bg='black',
                                   font=('Arial', 10, 'bold'))
        sensor_frame.pack(fill=tk.X, pady=10)
        
        # Sensor readings display
        self.sensor_text = tk.Text(sensor_frame, 
                                 height=4, width=80,
                                 bg='black', fg='green',
                                 font=('Courier', 9))
        self.sensor_text.pack(padx=10, pady=5)
        
        # Visualization frame
        viz_frame = tk.LabelFrame(main_frame, 
                                text="Real-time Sensor Visualization", 
                                fg='cyan', bg='black',
                                font=('Arial', 10, 'bold'))
        viz_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Image display
        self.image_label = tk.Label(viz_frame, 
                                  text="🔌 Connect to Companion Pi to see visualization",
                                  font=('Arial', 14),
                                  fg='gray', bg='black')
        self.image_label.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Log frame
        log_frame = tk.LabelFrame(main_frame, 
                                text="Connection Log", 
                                fg='cyan', bg='black',
                                font=('Arial', 10, 'bold'))
        log_frame.pack(fill=tk.X, pady=10)
        
        # Log display with scrollbar
        log_container = tk.Frame(log_frame, bg='black')
        log_container.pack(fill=tk.X, padx=10, pady=5)
        
        self.log_text = tk.Text(log_container, 
                              height=8, width=80,
                              bg='black', fg='white',
                              font=('Courier', 8))
        
        log_scrollbar = tk.Scrollbar(log_container, command=self.log_text.yview)
        self.log_text.config(yscrollcommand=log_scrollbar.set)
        
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
    def log_message(self, message, color='white'):
        """Add message to log display"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        formatted_msg = f"[{timestamp}] {message}\n"
        
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, formatted_msg)
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        
        # Also print to console
        print(formatted_msg.strip())
        
    def update_sensor_display(self, sensor_data):
        """Update sensor telemetry display"""
        try:
            # Format sensor information
            left_dist = f"{sensor_data.get('left_distance', 'NULL')}"
            right_dist = f"{sensor_data.get('right_distance', 'NULL')}"
            left_valid = "✅" if sensor_data.get('left_valid', False) else "❌"
            right_valid = "✅" if sensor_data.get('right_valid', False) else "❌"
            
            emergency = "🚨 YES" if sensor_data.get('emergency', False) else "✅ NO"
            
            direction_map = {-1: "← LEFT", 0: "CENTER", 1: "RIGHT →"}
            obstacle_dir = direction_map.get(sensor_data.get('obstacle_direction', 0), "UNKNOWN")
            
            # Update display
            sensor_info = f"""Left Sensor (30°):  {left_dist:>8} cm {left_valid}    Right Sensor (30°): {right_dist:>8} cm {right_valid}
Emergency Stop:     {emergency:>12}    Obstacle Direction: {obstacle_dir:>12}
Last Update:        {datetime.now().strftime('%H:%M:%S.%f')[:-3]:>12}    Total Frames: {self.total_frames:>8}"""
            
            self.sensor_text.config(state=tk.NORMAL)
            self.sensor_text.delete(1.0, tk.END)
            self.sensor_text.insert(1.0, sensor_info)
            self.sensor_text.config(state=tk.DISABLED)
            
        except Exception as e:
            self.log_message(f"❌ Error updating sensor display: {e}", 'red')
    
    def connect_to_companion_pi(self):
        """Connect to Companion Pi visualization server"""
        ip = self.ip_entry.get().strip()
        if not ip:
            ip = COMPANION_PI_IP
            
        self.connection_attempts += 1
        self.log_message(f"🔌 Attempting connection to {ip}:{VISUALIZATION_PORT} (attempt #{self.connection_attempts})")
        
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((ip, VISUALIZATION_PORT))
            
            self.connected = True
            self.log_message(f"✅ Connected to Companion Pi at {ip}:{VISUALIZATION_PORT}", 'green')
            
            # Update GUI
            self.status_label.config(text="🟢 Connected", fg='green')
            self.connect_btn.config(text="Disconnect", bg='red')
            
            return True
            
        except Exception as e:
            self.log_message(f"❌ Connection failed: {e}", 'red')
            self.socket = None
            return False
    
    def disconnect(self):
        """Disconnect from Companion Pi"""
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
            
        # Update GUI
        self.status_label.config(text="🔴 Disconnected", fg='red')
        self.connect_btn.config(text="Connect to Companion Pi", bg='green')
        self.image_label.config(image='', text="🔌 Connect to Companion Pi to see visualization")
        
        self.log_message("🔌 Disconnected from Companion Pi", 'yellow')
    
    def toggle_connection(self):
        """Toggle connection state"""
        if self.connected:
            self.disconnect()
        else:
            # Start connection in background thread
            threading.Thread(target=self.connection_thread, daemon=True).start()
    
    def connection_thread(self):
        """Background thread for maintaining connection"""
        while True:
            if not self.connected:
                if self.connect_to_companion_pi():
                    # Start data receiving
                    self.receive_data()
                else:
                    # Wait before retry
                    time.sleep(RECONNECT_DELAY)
            else:
                time.sleep(1)
    
    def receive_data(self):
        """Receive and process visualization data"""
        buffer = ""
        
        try:
            while self.connected:
                # Receive data
                data = self.socket.recv(4096).decode('utf-8')
                if not data:
                    break
                    
                buffer += data
                
                # Process complete messages (JSON lines)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        self.process_message(line.strip())
                        
        except Exception as e:
            self.log_message(f"⚠️  Data reception error: {e}", 'red')
        finally:
            self.disconnect()
    
    def process_message(self, message):
        """Process received visualization message"""
        try:
            data = json.loads(message)
            
            # Update frame statistics
            self.total_frames += 1
            self.last_update = time.time()
            
            # Calculate FPS
            elapsed = self.last_update - self.start_time
            fps = self.total_frames / elapsed if elapsed > 0 else 0
            
            # Update stats display
            self.stats_label.config(text=f"Frames: {self.total_frames} | FPS: {fps:.1f}")
            
            # Update sensor data
            if 'sensor_data' in data:
                self.update_sensor_display(data['sensor_data'])
            
            # Update visualization image
            if 'image' in data:
                self.update_image_display(data['image'])
                
        except Exception as e:
            self.log_message(f"⚠️  Message processing error: {e}", 'red')
    
    def update_image_display(self, image_data):
        """Update the visualization image display"""
        try:
            # Decode base64 image
            img_bytes = base64.b64decode(image_data)
            img = Image.open(io.BytesIO(img_bytes))
            
            # Resize to fit display area (maintain aspect ratio)
            display_size = (800, 600)
            img.thumbnail(display_size, Image.Resampling.LANCZOS)
            
            # Convert to PhotoImage for Tkinter
            photo = ImageTk.PhotoImage(img)
            
            # Update display
            self.image_label.config(image=photo, text="")
            self.image_label.image = photo  # Keep reference to prevent garbage collection
            
        except Exception as e:
            self.log_message(f"⚠️  Image display error: {e}", 'red')
    
    def run(self):
        """Start the application"""
        self.log_message("🚀 Development PC Visualization Receiver started")
        self.log_message(f"🎯 Ready to connect to Companion Pi at {COMPANION_PI_IP}:{VISUALIZATION_PORT}")
        
        # Start GUI event loop
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.log_message("🛑 Application stopped by user")
        finally:
            self.disconnect()

def main():
    print("🖥️  Starting Development PC Visualization Receiver...")
    print("📡 This will connect to the Companion Pi for real-time sensor visualization")
    
    app = VisualizationReceiver()
    app.run()

if __name__ == "__main__":
    main()