#!/usr/bin/env python3
"""
Visualization Pi Telemetry Receiver
================================================================================
Runs on the Visualization Pi to receive telemetry from Navigation Pi and 
optionally relay to base station or provide local visualization.

Features:
- Receives real-time telemetry from Navigation Pi via ethernet
- Local data logging and visualization
- Optional relay to base station
- Web interface for local monitoring
- Data storage for analysis

Part of the Mini Rover Development Project - Navigation V7 Dual Pi
Author: Developed with Jay Fielding
Version: 1.0
Date: 2025-01-22
================================================================================
"""

import time
import json
import threading
from datetime import datetime
from typing import Dict, Any
import logging

# Try to import visualization libraries
try:
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    print("⚠️ Matplotlib not available - install with: pip install matplotlib")
    MATPLOTLIB_AVAILABLE = False

# Import our communication module
try:
    from pi_communication import VisualizationDataReceiver
    COMMUNICATION_AVAILABLE = True
except ImportError:
    print("❌ Pi communication module not found")
    COMMUNICATION_AVAILABLE = False

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/home/jay/rover_project/telemetry.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

class VisualizationPiReceiver:
    """
    Telemetry receiver and processor for Visualization Pi
    
    Handles:
    - Real-time telemetry reception from Navigation Pi
    - Data logging and storage
    - Local visualization (if display available)
    - Statistics and monitoring
    """
    
    def __init__(self, log_file_path='/home/jay/rover_project/telemetry_data.json'):
        self.log_file_path = log_file_path
        self.receiver = None
        self.running = False
        
        # Data storage
        self.telemetry_data = []
        self.max_data_points = 1000
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'connections_received': 0,
            'start_time': None,
            'last_message_time': None,
            'data_rate': 0.0,
            'current_nav_state': 'UNKNOWN',
            'current_bluetooth_distance': None,
            'current_ultrasonic_distance': None,
            'current_rssi': None
        }
        
        # Visualization setup
        self.visualization_enabled = MATPLOTLIB_AVAILABLE
        self.fig = None
        self.axes = None
        self.animation = None
        
        logger.info("Visualization Pi Receiver initialized")
    
    def start(self):
        """Start the telemetry receiver"""
        if not COMMUNICATION_AVAILABLE:
            logger.error("Communication module not available")
            return False
        
        try:
            self.receiver = VisualizationDataReceiver()
            self.receiver.start_server(
                data_callback=self.handle_telemetry_data,
                connection_callback=self.handle_connection_event
            )
            
            self.running = True
            self.stats['start_time'] = time.time()
            
            logger.info("📡 Telemetry receiver started on port 8888")
            logger.info("🔄 Waiting for Navigation Pi connection...")
            
            # Start visualization if available
            if self.visualization_enabled:
                self.setup_visualization()
            
            # Start statistics thread
            stats_thread = threading.Thread(target=self.statistics_loop, daemon=True)
            stats_thread.start()
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to start receiver: {e}")
            return False
    
    def stop(self):
        """Stop the telemetry receiver"""
        self.running = False
        
        if self.receiver:
            self.receiver.stop_server()
            
        logger.info("📡 Telemetry receiver stopped")
    
    def handle_telemetry_data(self, data: Dict[str, Any]):
        """Handle incoming telemetry data from Navigation Pi"""
        try:
            # Update statistics
            self.stats['messages_received'] += 1
            self.stats['last_message_time'] = time.time()
            
            # Extract key values
            self.stats['current_nav_state'] = data.get('nav_state', 'UNKNOWN')
            self.stats['current_bluetooth_distance'] = data.get('bluetooth_distance')
            self.stats['current_ultrasonic_distance'] = data.get('ultrasonic_distance')
            self.stats['current_rssi'] = data.get('bluetooth_rssi')
            
            # Add timestamp if not present
            if 'received_timestamp' not in data:
                data['received_timestamp'] = time.time()
            
            # Store data
            self.telemetry_data.append(data)
            if len(self.telemetry_data) > self.max_data_points:
                self.telemetry_data.pop(0)
            
            # Log to file
            self.log_telemetry_data(data)
            
            # Log key events
            nav_state = data.get('nav_state', 'UNKNOWN')
            bt_distance = data.get('bluetooth_distance')
            us_distance = data.get('ultrasonic_distance')
            
            if self.stats['messages_received'] % 10 == 0:  # Log every 10th message
                log_msg = f"[NAV] {nav_state}"
                if bt_distance:
                    log_msg += f" | BT: {bt_distance:.1f}m"
                if us_distance:
                    log_msg += f" | US: {us_distance:.1f}cm"
                logger.info(log_msg)
            
        except Exception as e:
            logger.error(f"Error handling telemetry data: {e}")
    
    def handle_connection_event(self, message: str):
        """Handle connection events from Navigation Pi"""
        self.stats['connections_received'] += 1
        logger.info(f"🔌 Connection event: {message}")
    
    def log_telemetry_data(self, data: Dict[str, Any]):
        """Log telemetry data to file"""
        try:
            with open(self.log_file_path, 'a') as f:
                f.write(json.dumps(data) + '\n')
        except Exception as e:
            logger.warning(f"Failed to log telemetry data: {e}")
    
    def setup_visualization(self):
        """Setup real-time visualization plots"""
        if not self.visualization_enabled:
            return
        
        try:
            # Create figure with subplots
            self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
            self.fig.suptitle('Rover Telemetry - Live Data', fontsize=16)
            
            # Configure subplots
            self.axes[0, 0].set_title('Bluetooth Distance')
            self.axes[0, 0].set_ylabel('Distance (m)')
            self.axes[0, 0].grid(True)
            
            self.axes[0, 1].set_title('Ultrasonic Distance')
            self.axes[0, 1].set_ylabel('Distance (cm)')
            self.axes[0, 1].grid(True)
            
            self.axes[1, 0].set_title('RSSI Signal Strength')
            self.axes[1, 0].set_ylabel('RSSI (dBm)')
            self.axes[1, 0].grid(True)
            
            self.axes[1, 1].set_title('Navigation State')
            self.axes[1, 1].set_ylabel('State')
            self.axes[1, 1].grid(True)
            
            # Start animation
            self.animation = FuncAnimation(
                self.fig, self.update_plots, interval=1000, blit=False
            )
            
            # Show plot in separate thread
            plot_thread = threading.Thread(target=self.show_plots, daemon=True)
            plot_thread.start()
            
            logger.info("📊 Real-time visualization started")
            
        except Exception as e:
            logger.warning(f"Visualization setup failed: {e}")
            self.visualization_enabled = False
    
    def show_plots(self):
        """Show matplotlib plots (blocking call)"""
        try:
            plt.show()
        except Exception as e:
            logger.warning(f"Plot display error: {e}")
    
    def update_plots(self, frame):
        """Update visualization plots with latest data"""
        if not self.telemetry_data:
            return
        
        try:
            # Extract data for plotting
            timestamps = [d.get('timestamp', 0) for d in self.telemetry_data[-50:]]  # Last 50 points
            bt_distances = [d.get('bluetooth_distance') for d in self.telemetry_data[-50:]]
            us_distances = [d.get('ultrasonic_distance') for d in self.telemetry_data[-50:]]
            rssi_values = [d.get('bluetooth_rssi') for d in self.telemetry_data[-50:]]
            nav_states = [d.get('nav_state', 'UNKNOWN') for d in self.telemetry_data[-50:]]
            
            # Convert timestamps to datetime
            if timestamps:
                datetimes = [datetime.fromtimestamp(ts) for ts in timestamps]
            else:
                return
            
            # Clear and update plots
            for ax in self.axes.flat:
                ax.clear()
                ax.grid(True)
            
            # Bluetooth distance plot
            self.axes[0, 0].set_title('Bluetooth Distance')
            self.axes[0, 0].set_ylabel('Distance (m)')
            valid_bt = [(dt, d) for dt, d in zip(datetimes, bt_distances) if d is not None]
            if valid_bt:
                times, dists = zip(*valid_bt)
                self.axes[0, 0].plot(times, dists, 'b-', label='BT Distance')
                self.axes[0, 0].legend()
            
            # Ultrasonic distance plot
            self.axes[0, 1].set_title('Ultrasonic Distance')
            self.axes[0, 1].set_ylabel('Distance (cm)')
            valid_us = [(dt, d) for dt, d in zip(datetimes, us_distances) if d is not None]
            if valid_us:
                times, dists = zip(*valid_us)
                self.axes[0, 1].plot(times, dists, 'r-', label='US Distance')
                self.axes[0, 1].legend()
            
            # RSSI plot
            self.axes[1, 0].set_title('RSSI Signal Strength')
            self.axes[1, 0].set_ylabel('RSSI (dBm)')
            valid_rssi = [(dt, r) for dt, r in zip(datetimes, rssi_values) if r is not None]
            if valid_rssi:
                times, rssi = zip(*valid_rssi)
                self.axes[1, 0].plot(times, rssi, 'g-', label='RSSI')
                self.axes[1, 0].legend()
            
            # Navigation state plot
            self.axes[1, 1].set_title('Navigation State')
            self.axes[1, 1].set_ylabel('State')
            state_map = {
                'SEARCHING': 0, 'FORWARD_TRACKING': 1, 'TURN_TESTING': 2,
                'APPROACHING': 3, 'HOLDING': 4, 'SMART_AVOIDING': 5, 'BACKING': 6
            }
            state_nums = [state_map.get(s, -1) for s in nav_states]
            if datetimes and state_nums:
                self.axes[1, 1].plot(datetimes, state_nums, 'ko-', markersize=3)
                self.axes[1, 1].set_yticks(list(state_map.values()))
                self.axes[1, 1].set_yticklabels(list(state_map.keys()), fontsize=8)
            
            # Format x-axis for time
            for ax in self.axes.flat:
                ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
                ax.xaxis.set_major_locator(mdates.SecondLocator(interval=30))
                plt.setp(ax.xaxis.get_majorticklabels(), rotation=45)
            
            self.fig.tight_layout()
            
        except Exception as e:
            logger.warning(f"Plot update error: {e}")
    
    def statistics_loop(self):
        """Background thread for statistics and monitoring"""
        while self.running:
            try:
                # Calculate data rate
                if self.stats['start_time']:
                    runtime = time.time() - self.stats['start_time']
                    if runtime > 0:
                        self.stats['data_rate'] = self.stats['messages_received'] / runtime
                
                # Log statistics every 30 seconds
                if int(time.time()) % 30 == 0:
                    self.log_statistics()
                
                time.sleep(1)
                
            except Exception as e:
                logger.error(f"Statistics loop error: {e}")
    
    def log_statistics(self):
        """Log current statistics"""
        runtime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        
        logger.info("📊 === Telemetry Statistics ===")
        logger.info(f"   Runtime: {runtime:.0f} seconds")
        logger.info(f"   Messages received: {self.stats['messages_received']}")
        logger.info(f"   Data rate: {self.stats['data_rate']:.2f} msg/sec")
        logger.info(f"   Connections: {self.stats['connections_received']}")
        logger.info(f"   Current state: {self.stats['current_nav_state']}")
        
        if self.stats['current_bluetooth_distance']:
            logger.info(f"   BT Distance: {self.stats['current_bluetooth_distance']:.1f}m")
        if self.stats['current_ultrasonic_distance']:
            logger.info(f"   US Distance: {self.stats['current_ultrasonic_distance']:.1f}cm")
        if self.stats['current_rssi']:
            logger.info(f"   RSSI: {self.stats['current_rssi']:.0f}dBm")
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get current statistics"""
        return self.stats.copy()
    
    def save_data_to_file(self, filename: str = None):
        """Save current telemetry data to JSON file"""
        if not filename:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'/home/jay/rover_project/telemetry_export_{timestamp}.json'
        
        try:
            with open(filename, 'w') as f:
                json.dump({
                    'statistics': self.stats,
                    'telemetry_data': self.telemetry_data
                }, f, indent=2)
            
            logger.info(f"💾 Data saved to {filename}")
            return filename
            
        except Exception as e:
            logger.error(f"Failed to save data: {e}")
            return None

def main():
    """Main entry point for Visualization Pi Receiver"""
    print("=" * 80)
    print("📊 VISUALIZATION PI TELEMETRY RECEIVER")
    print("=" * 80)
    print()
    print("FEATURES:")
    print("  📡 Real-time telemetry reception from Navigation Pi")
    print("  💾 Automatic data logging and storage")
    print("  📊 Live data visualization (if matplotlib available)")
    print("  📈 Statistics and monitoring")
    print("  🔄 Connection management and recovery")
    print()
    
    if not COMMUNICATION_AVAILABLE:
        print("❌ ERROR: Pi communication module not available")
        print("   Ensure pi_communication.py is in the same directory")
        return
    
    receiver = VisualizationPiReceiver()
    
    try:
        if receiver.start():
            print("✅ Visualization Pi Receiver started")
            print("📡 Listening for Navigation Pi telemetry on port 8888")
            if MATPLOTLIB_AVAILABLE:
                print("📊 Real-time visualization available")
            print("Press Ctrl+C to stop...")
            
            while True:
                time.sleep(1)
        else:
            print("❌ Failed to start receiver")
    
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested by user...")
    
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    
    finally:
        receiver.stop()
        
        # Save data on exit
        print("💾 Saving telemetry data...")
        saved_file = receiver.save_data_to_file()
        if saved_file:
            print(f"✅ Data saved to {saved_file}")
        
        print("👋 Visualization Pi Receiver shutdown complete")

if __name__ == "__main__":
    main()