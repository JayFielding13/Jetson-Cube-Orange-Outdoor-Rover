#!/usr/bin/env python3
"""
Ultra-Stable GPS Display for Mobile Beacon
Handles corrupted GPS data with advanced filtering and validation
Optimized for 1024x600 touchscreen with rock-solid data stability
"""

import tkinter as tk
from tkinter import ttk
import serial
import threading
import time
import re
from datetime import datetime
from collections import deque

class UltraStableGPSDisplay:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Mobile Beacon GPS - Ultra Stable")
        self.root.geometry("1024x600")
        self.root.configure(bg='black')

        # Make it fullscreen
        self.root.attributes('-fullscreen', True)
        self.root.bind('<Escape>', self.exit_fullscreen)

        # GPS connection
        self.gps_port = '/dev/ttyACM0'
        self.gps_baudrate = 115200
        self.gps = None
        self.gps_lock = threading.Lock()

        # Data validation and filtering
        self.lat_history = deque(maxlen=20)
        self.lon_history = deque(maxlen=20)
        self.sat_history = deque(maxlen=15)
        self.hdop_history = deque(maxlen=15)
        self.alt_history = deque(maxlen=10)

        # Current stable values
        self.current_data = {
            'lat': None,
            'lon': None,
            'alt': 0.0,
            'satellites': 0,
            'hdop': 99.9,
            'fix_type': 0,
            'last_update': None
        }

        # Data validation ranges
        self.validation_ranges = {
            'lat': (40.0, 50.0),      # Pacific Northwest range
            'lon': (-130.0, -110.0),  # Pacific Northwest range
            'satellites': (0, 50),    # Max reasonable satellite count
            'hdop': (0.1, 50.0),      # HDOP range
            'alt': (-500, 10000)      # Altitude range in meters
        }

        self.setup_gui()
        self.connect_gps()
        self.start_gps_thread()
        self.update_display()

    def setup_gui(self):
        """Create the GUI layout optimized for 1024x600"""

        # Remove title bar - just show data

        # Status frame
        status_frame = tk.Frame(self.root, bg='black')
        status_frame.pack(pady=5)

        self.status_label = tk.Label(status_frame, text="CONNECTING...",
                                   font=('Arial', 18, 'bold'),
                                   fg='yellow', bg='black')
        self.status_label.pack()

        # Main data container
        main_frame = tk.Frame(self.root, bg='black')
        main_frame.pack(expand=True, fill='both', padx=20, pady=10)

        # Left column - Position
        left_frame = tk.Frame(main_frame, bg='black')
        left_frame.pack(side='left', expand=True, fill='both')

        pos_title = tk.Label(left_frame, text="POSITION",
                           font=('Arial', 20, 'bold'),
                           fg='cyan', bg='black')
        pos_title.pack(anchor='w', pady=(0,10))

        # Latitude
        lat_frame = tk.Frame(left_frame, bg='black')
        lat_frame.pack(anchor='w', pady=5)

        tk.Label(lat_frame, text="Latitude:",
                font=('Arial', 16), fg='white', bg='black').pack(side='left')
        self.lat_label = tk.Label(lat_frame, text="No Fix",
                                font=('Arial', 16, 'bold'),
                                fg='red', bg='black')
        self.lat_label.pack(side='left', padx=(10,0))

        # Longitude
        lon_frame = tk.Frame(left_frame, bg='black')
        lon_frame.pack(anchor='w', pady=5)

        tk.Label(lon_frame, text="Longitude:",
                font=('Arial', 16), fg='white', bg='black').pack(side='left')
        self.lon_label = tk.Label(lon_frame, text="No Fix",
                                font=('Arial', 16, 'bold'),
                                fg='red', bg='black')
        self.lon_label.pack(side='left', padx=(10,0))

        # Altitude
        alt_frame = tk.Frame(left_frame, bg='black')
        alt_frame.pack(anchor='w', pady=5)

        tk.Label(alt_frame, text="Altitude:",
                font=('Arial', 16), fg='white', bg='black').pack(side='left')
        self.alt_label = tk.Label(alt_frame, text="-- m",
                                font=('Arial', 16, 'bold'),
                                fg='orange', bg='black')
        self.alt_label.pack(side='left', padx=(10,0))

        # Right column - Signal Quality
        right_frame = tk.Frame(main_frame, bg='black')
        right_frame.pack(side='right', expand=True, fill='both')

        signal_title = tk.Label(right_frame, text="SIGNAL QUALITY",
                              font=('Arial', 20, 'bold'),
                              fg='cyan', bg='black')
        signal_title.pack(anchor='w', pady=(0,10))

        # Satellites
        sat_frame = tk.Frame(right_frame, bg='black')
        sat_frame.pack(anchor='w', pady=5)

        tk.Label(sat_frame, text="Satellites:",
                font=('Arial', 16), fg='white', bg='black').pack(side='left')
        self.sat_label = tk.Label(sat_frame, text="0",
                                font=('Arial', 16, 'bold'),
                                fg='red', bg='black')
        self.sat_label.pack(side='left', padx=(10,0))

        # HDOP
        hdop_frame = tk.Frame(right_frame, bg='black')
        hdop_frame.pack(anchor='w', pady=5)

        tk.Label(hdop_frame, text="HDOP:",
                font=('Arial', 16), fg='white', bg='black').pack(side='left')
        self.hdop_label = tk.Label(hdop_frame, text="99.9",
                                 font=('Arial', 16, 'bold'),
                                 fg='red', bg='black')
        self.hdop_label.pack(side='left', padx=(10,0))

        # Quality indicator
        quality_frame = tk.Frame(right_frame, bg='black')
        quality_frame.pack(anchor='w', pady=5)

        tk.Label(quality_frame, text="Quality:",
                font=('Arial', 16), fg='white', bg='black').pack(side='left')
        self.quality_label = tk.Label(quality_frame, text="POOR",
                                    font=('Arial', 16, 'bold'),
                                    fg='red', bg='black')
        self.quality_label.pack(side='left', padx=(10,0))

        # Bottom info
        bottom_frame = tk.Frame(self.root, bg='black')
        bottom_frame.pack(side='bottom', pady=10)

        self.time_label = tk.Label(bottom_frame, text="",
                                 font=('Arial', 14),
                                 fg='gray', bg='black')
        self.time_label.pack()

        # Debug info (smaller)
        self.debug_label = tk.Label(bottom_frame, text="",
                                  font=('Arial', 10),
                                  fg='gray', bg='black')
        self.debug_label.pack()

    def exit_fullscreen(self, event=None):
        """Exit fullscreen mode"""
        self.root.attributes('-fullscreen', False)

    def connect_gps(self):
        """Connect to GPS with robust error handling"""
        try:
            if self.gps:
                self.gps.close()
            self.gps = serial.Serial(self.gps_port, self.gps_baudrate, timeout=1)
            time.sleep(2)  # Give GPS time to start
            print(f"GPS connected on {self.gps_port}")
            return True
        except Exception as e:
            print(f"GPS connection failed: {e}")
            self.gps = None
            return False

    def parse_nmea_coordinate(self, coord_str, direction):
        """Parse NMEA coordinate format (DDMM.MMMM) to decimal degrees"""
        try:
            if not coord_str or not direction or len(coord_str) < 7:
                return None

            # For latitude: DDMM.MMMM (like 4525.82057)
            # For longitude: DDDMM.MMMM (like 12250.46152)

            if '.' not in coord_str:
                return None

            # Split into degrees and minutes parts
            parts = coord_str.split('.')
            if len(parts) != 2:
                return None

            degrees_minutes = parts[0]
            decimal_minutes = parts[1]

            # For latitude (2 digit degrees) vs longitude (3 digit degrees)
            if len(degrees_minutes) == 4:  # Latitude: DDMM
                degrees = float(degrees_minutes[:2])
                minutes = float(degrees_minutes[2:] + '.' + decimal_minutes)
            elif len(degrees_minutes) == 5:  # Longitude: DDDMM
                degrees = float(degrees_minutes[:3])
                minutes = float(degrees_minutes[3:] + '.' + decimal_minutes)
            else:
                return None

            decimal_degrees = degrees + minutes / 60.0

            # Apply direction
            if direction in ['S', 'W']:
                decimal_degrees = -decimal_degrees

            return decimal_degrees

        except (ValueError, IndexError):
            return None

    def validate_coordinate(self, value, coord_type):
        """Validate coordinate value against reasonable ranges"""
        if value is None:
            return False

        valid_range = self.validation_ranges.get(coord_type)
        if valid_range:
            return valid_range[0] <= value <= valid_range[1]
        return True

    def is_value_stable(self, new_value, history, max_change_pct=50):
        """Check if new value is stable compared to recent history"""
        if not history or new_value is None:
            return True

        recent_values = [v for v in list(history)[-5:] if v is not None]
        if len(recent_values) < 2:
            return True

        avg = sum(recent_values) / len(recent_values)
        if avg == 0:
            return abs(new_value) < 10  # Avoid division by zero

        change_pct = abs(new_value - avg) / abs(avg) * 100
        return change_pct <= max_change_pct

    def clean_nmea_sentence(self, line):
        """Clean and validate NMEA sentence"""
        try:
            # Remove any non-printable characters
            line = ''.join(char for char in line if ord(char) >= 32 and ord(char) <= 126)

            # Look for valid NMEA sentence pattern
            nmea_pattern = r'\$[A-Z]{2}[A-Z]{3},[^*]*\*[0-9A-F]{2}'
            matches = re.findall(nmea_pattern, line)

            if matches:
                return matches[0]  # Return first valid sentence found
            return None

        except Exception:
            return None

    def parse_gps_data(self):
        """Parse GPS data with enhanced error handling and validation"""
        if not self.gps:
            return

        try:
            # Read multiple lines to find a complete sentence
            for _ in range(10):  # Try up to 10 lines
                with self.gps_lock:
                    raw_line = self.gps.readline().decode('ascii', errors='ignore').strip()

                if not raw_line:
                    continue

                # Look for complete GGA sentences
                if '$GNGGA' in raw_line or '$GPGGA' in raw_line:
                    # Find the start of the sentence
                    start_pos = raw_line.find('$GNGGA')
                    if start_pos == -1:
                        start_pos = raw_line.find('$GPGGA')

                    if start_pos == -1:
                        continue

                    # Extract from $ to end or to next $
                    sentence = raw_line[start_pos:]
                    next_dollar = sentence.find('$', 1)
                    if next_dollar != -1:
                        sentence = sentence[:next_dollar]

                    # Verify sentence has minimum length and proper format
                    if len(sentence) < 50 or sentence.count(',') < 10:
                        continue

                    parts = sentence.split(',')
                    if len(parts) < 15:
                        continue

                    # Extract fix quality
                    try:
                        fix_quality = int(parts[6]) if parts[6].isdigit() else 0
                    except (ValueError, IndexError):
                        continue

                    # Parse all data regardless of fix quality for display
                    # Parse coordinates
                    lat = self.parse_nmea_coordinate(parts[2], parts[3])
                    lon = self.parse_nmea_coordinate(parts[4], parts[5])

                    # Only accept coordinates if they're valid and stable
                    if (lat and lon and
                        self.validate_coordinate(lat, 'lat') and
                        self.validate_coordinate(lon, 'lon') and
                        self.is_value_stable(lat, self.lat_history, 5) and
                        self.is_value_stable(lon, self.lon_history, 5)):

                        self.lat_history.append(lat)
                        self.lon_history.append(lon)
                        self.current_data['lat'] = lat
                        self.current_data['lon'] = lon

                    # Parse satellites
                    try:
                        satellites = int(parts[7]) if parts[7].isdigit() else 0
                        if (0 <= satellites <= 50 and
                            self.is_value_stable(satellites, self.sat_history, 30)):
                            self.sat_history.append(satellites)
                            self.current_data['satellites'] = satellites
                    except (ValueError, IndexError):
                        pass

                    # Parse HDOP
                    try:
                        hdop = float(parts[8]) if parts[8] and parts[8] != '' else 99.9
                        if (0.1 <= hdop <= 50.0 and
                            self.is_value_stable(hdop, self.hdop_history, 40)):
                            self.hdop_history.append(hdop)
                            self.current_data['hdop'] = hdop
                    except (ValueError, IndexError):
                        pass

                    # Parse altitude
                    try:
                        altitude = float(parts[9]) if parts[9] and parts[9] != '' else 0.0
                        if (self.validate_coordinate(altitude, 'alt') and
                            self.is_value_stable(altitude, self.alt_history, 50)):
                            self.alt_history.append(altitude)
                            self.current_data['alt'] = altitude
                    except (ValueError, IndexError):
                        pass

                    self.current_data['fix_type'] = fix_quality
                    self.current_data['last_update'] = time.time()
                    return  # Successfully parsed, exit

        except Exception as e:
            print(f"GPS parsing error: {e}")

    def gps_reader_thread(self):
        """GPS reading thread"""
        while True:
            try:
                if not self.gps and not self.connect_gps():
                    time.sleep(5)
                    continue

                self.parse_gps_data()
                time.sleep(0.1)  # 10Hz reading rate

            except Exception as e:
                print(f"GPS thread error: {e}")
                self.gps = None
                time.sleep(2)

    def start_gps_thread(self):
        """Start GPS reading thread"""
        gps_thread = threading.Thread(target=self.gps_reader_thread, daemon=True)
        gps_thread.start()

    def get_stable_average(self, history, default=None):
        """Get stable average from history, excluding outliers"""
        if not history:
            return default

        values = list(history)
        if len(values) < 3:
            return values[-1] if values else default

        # Remove outliers (simple method)
        sorted_vals = sorted(values)
        # Remove top and bottom 20% if we have enough data
        if len(sorted_vals) >= 5:
            start_idx = len(sorted_vals) // 5
            end_idx = len(sorted_vals) - start_idx
            filtered_vals = sorted_vals[start_idx:end_idx]
        else:
            filtered_vals = sorted_vals

        return sum(filtered_vals) / len(filtered_vals)

    def update_display(self):
        """Update GUI display with current GPS data"""
        try:
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.time_label.config(text=f"Updated: {current_time}")

            # Check if we have recent data
            last_update = self.current_data.get('last_update')
            data_age = time.time() - last_update if last_update else 999

            if data_age > 10:  # No data for 10 seconds
                self.status_label.config(text="NO GPS DATA", fg='red')
                self.lat_label.config(text="No Fix", fg='red')
                self.lon_label.config(text="No Fix", fg='red')
                self.sat_label.config(text="0", fg='red')
                self.hdop_label.config(text="99.9", fg='red')
                self.quality_label.config(text="DISCONNECTED", fg='red')
            else:
                # Update position with stable averages
                stable_lat = self.get_stable_average(self.lat_history)
                stable_lon = self.get_stable_average(self.lon_history)

                if stable_lat and stable_lon:
                    self.lat_label.config(text=f"{stable_lat:.8f}°", fg='lime')
                    self.lon_label.config(text=f"{stable_lon:.8f}°", fg='lime')
                    self.status_label.config(text="GPS FIXED", fg='lime')
                else:
                    self.lat_label.config(text="No Fix", fg='red')
                    self.lon_label.config(text="No Fix", fg='red')
                    self.status_label.config(text="NO FIX", fg='yellow')

                # Update signal quality
                stable_sats = int(self.get_stable_average(self.sat_history, 0))
                stable_hdop = self.get_stable_average(self.hdop_history, 99.9)
                stable_alt = self.get_stable_average(self.alt_history, 0)

                self.sat_label.config(text=str(stable_sats),
                                    fg='lime' if stable_sats >= 4 else 'yellow' if stable_sats > 0 else 'red')

                self.hdop_label.config(text=f"{stable_hdop:.1f}",
                                     fg='lime' if stable_hdop < 2.0 else 'yellow' if stable_hdop < 5.0 else 'red')

                self.alt_label.config(text=f"{stable_alt:.1f} m", fg='orange')

                # Overall quality assessment
                if stable_hdop < 2.0 and stable_sats >= 6:
                    quality = "EXCELLENT"
                    quality_color = 'lime'
                elif stable_hdop < 5.0 and stable_sats >= 4:
                    quality = "GOOD"
                    quality_color = 'yellow'
                elif stable_sats > 0:
                    quality = "FAIR"
                    quality_color = 'orange'
                else:
                    quality = "POOR"
                    quality_color = 'red'

                self.quality_label.config(text=quality, fg=quality_color)

            # Debug info
            debug_info = (f"Histories: LAT:{len(self.lat_history)} LON:{len(self.lon_history)} "
                         f"SAT:{len(self.sat_history)} HDOP:{len(self.hdop_history)}")
            self.debug_label.config(text=debug_info)

        except Exception as e:
            print(f"Display update error: {e}")

        # Schedule next update
        self.root.after(200, self.update_display)  # Update every 200ms

    def run(self):
        """Start the GPS display application"""
        self.root.mainloop()

if __name__ == "__main__":
    try:
        app = UltraStableGPSDisplay()
        app.run()
    except KeyboardInterrupt:
        print("GPS Display stopped.")
    except Exception as e:
        print(f"Application error: {e}")