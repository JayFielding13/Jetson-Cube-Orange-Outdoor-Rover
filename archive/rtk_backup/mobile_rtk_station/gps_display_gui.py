#!/usr/bin/env python3
"""
Mobile Beacon GPS Display GUI
Real-time GPS status display for touchscreen debugging
"""

import tkinter as tk
from tkinter import ttk
import serial
import threading
import time
from datetime import datetime

class GPSDisplayGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Mobile Beacon GPS Status")
        self.root.geometry("800x480")  # Common 7" touchscreen resolution
        self.root.configure(bg='black')

        # GPS data
        self.gps_port = '/dev/ttyACM0'
        self.gps_baud = 115200
        self.running = True

        # GPS status variables
        self.lat = tk.StringVar(value="No Fix")
        self.lon = tk.StringVar(value="No Fix")
        self.alt = tk.StringVar(value="--")
        self.satellites = tk.StringVar(value="0")
        self.fix_type = tk.StringVar(value="No Fix")
        self.hdop = tk.StringVar(value="--")
        self.last_update = tk.StringVar(value="Never")
        self.rtk_age = tk.StringVar(value="--")

        self.setup_gui()
        self.start_gps_reader()

    def setup_gui(self):
        """Create the GUI layout"""
        # Main title
        title = tk.Label(self.root, text="Mobile Beacon GPS Status",
                        font=("Arial", 24, "bold"), fg="white", bg="black")
        title.pack(pady=10)

        # Status frame
        status_frame = tk.Frame(self.root, bg="black")
        status_frame.pack(expand=True, fill="both", padx=20, pady=10)

        # GPS Fix Status (large and prominent)
        fix_frame = tk.Frame(status_frame, bg="black")
        fix_frame.pack(pady=15)

        tk.Label(fix_frame, text="GPS Status:", font=("Arial", 16),
                fg="white", bg="black").pack()
        self.fix_label = tk.Label(fix_frame, textvariable=self.fix_type,
                                 font=("Arial", 20, "bold"), bg="black")
        self.fix_label.pack()

        # Coordinates
        coords_frame = tk.Frame(status_frame, bg="black")
        coords_frame.pack(pady=10)

        # Latitude
        lat_frame = tk.Frame(coords_frame, bg="black")
        lat_frame.pack(pady=5)
        tk.Label(lat_frame, text="Latitude:", font=("Arial", 14),
                fg="white", bg="black").pack(side="left")
        tk.Label(lat_frame, textvariable=self.lat, font=("Arial", 14, "bold"),
                fg="cyan", bg="black").pack(side="left", padx=10)

        # Longitude
        lon_frame = tk.Frame(coords_frame, bg="black")
        lon_frame.pack(pady=5)
        tk.Label(lon_frame, text="Longitude:", font=("Arial", 14),
                fg="white", bg="black").pack(side="left")
        tk.Label(lon_frame, textvariable=self.lon, font=("Arial", 14, "bold"),
                fg="cyan", bg="black").pack(side="left", padx=10)

        # Altitude
        alt_frame = tk.Frame(coords_frame, bg="black")
        alt_frame.pack(pady=5)
        tk.Label(alt_frame, text="Altitude:", font=("Arial", 14),
                fg="white", bg="black").pack(side="left")
        tk.Label(alt_frame, textvariable=self.alt, font=("Arial", 14, "bold"),
                fg="cyan", bg="black").pack(side="left", padx=10)

        # Technical details frame
        tech_frame = tk.Frame(status_frame, bg="black")
        tech_frame.pack(pady=15)

        # Satellites and HDOP
        sats_hdop_frame = tk.Frame(tech_frame, bg="black")
        sats_hdop_frame.pack()

        tk.Label(sats_hdop_frame, text="Satellites:", font=("Arial", 12),
                fg="white", bg="black").pack(side="left")
        tk.Label(sats_hdop_frame, textvariable=self.satellites, font=("Arial", 12, "bold"),
                fg="green", bg="black").pack(side="left", padx=5)

        tk.Label(sats_hdop_frame, text="HDOP:", font=("Arial", 12),
                fg="white", bg="black").pack(side="left", padx=(20,0))
        tk.Label(sats_hdop_frame, textvariable=self.hdop, font=("Arial", 12, "bold"),
                fg="yellow", bg="black").pack(side="left", padx=5)

        # Last update
        update_frame = tk.Frame(status_frame, bg="black")
        update_frame.pack(pady=10)
        tk.Label(update_frame, text="Last Update:", font=("Arial", 10),
                fg="gray", bg="black").pack()
        tk.Label(update_frame, textvariable=self.last_update, font=("Arial", 10),
                fg="gray", bg="black").pack()

        # Exit button
        exit_btn = tk.Button(self.root, text="Exit", font=("Arial", 12),
                           command=self.exit_app, bg="red", fg="white")
        exit_btn.pack(side="bottom", pady=10)

    def update_fix_color(self, fix_type_num):
        """Update GPS fix status color based on fix type"""
        if fix_type_num == 0:
            self.fix_label.config(fg="red")
            self.fix_type.set("No Fix")
        elif fix_type_num == 1:
            self.fix_label.config(fg="orange")
            self.fix_type.set("GPS Fix")
        elif fix_type_num == 2:
            self.fix_label.config(fg="lime")
            self.fix_type.set("RTK FIXED")
        elif fix_type_num == 3:
            self.fix_label.config(fg="yellow")
            self.fix_type.set("RTK Float")
        else:
            self.fix_label.config(fg="gray")
            self.fix_type.set("Unknown")

    def parse_gps_data(self):
        """Parse GPS NMEA data in background thread"""
        try:
            gps = serial.Serial(self.gps_port, self.gps_baud, timeout=1)
            print(f"Connected to GPS on {self.gps_port}")

            while self.running:
                try:
                    line = gps.readline().decode('ascii', errors='ignore').strip()

                    if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                        parts = line.split(',')
                        if len(parts) >= 15:
                            # Parse latitude
                            if parts[2] and parts[3]:
                                lat_deg = float(parts[2][:2])
                                lat_min = float(parts[2][2:])
                                lat = lat_deg + lat_min/60.0
                                if parts[3] == 'S':
                                    lat = -lat
                                self.lat.set(f"{lat:.8f}°")
                            else:
                                self.lat.set("No Fix")

                            # Parse longitude
                            if parts[4] and parts[5]:
                                lon_deg = float(parts[4][:3])
                                lon_min = float(parts[4][3:])
                                lon = lon_deg + lon_min/60.0
                                if parts[5] == 'W':
                                    lon = -lon
                                self.lon.set(f"{lon:.8f}°")
                            else:
                                self.lon.set("No Fix")

                            # Parse other data
                            fix_type_num = int(parts[6]) if parts[6] else 0
                            sats = int(parts[7]) if parts[7] else 0
                            altitude = float(parts[9]) if parts[9] else 0.0
                            hdop_val = float(parts[8]) if parts[8] else 99.9

                            # Update display
                            self.update_fix_color(fix_type_num)
                            self.satellites.set(str(sats))
                            self.alt.set(f"{altitude:.1f}m")
                            self.hdop.set(f"{hdop_val:.1f}")
                            self.last_update.set(datetime.now().strftime("%H:%M:%S"))

                except Exception as e:
                    continue

        except Exception as e:
            print(f"GPS connection error: {e}")
            self.fix_type.set("GPS Error")
            self.fix_label.config(fg="red")

    def start_gps_reader(self):
        """Start GPS reading in background thread"""
        gps_thread = threading.Thread(target=self.parse_gps_data)
        gps_thread.daemon = True
        gps_thread.start()

    def exit_app(self):
        """Clean exit"""
        self.running = False
        self.root.quit()

    def run(self):
        """Start the GUI"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.exit_app()

if __name__ == "__main__":
    app = GPSDisplayGUI()
    app.run()