#!/usr/bin/env python3
"""
Rover Debug Dashboard
Real-time graphical monitoring tool for rover status and sensor data
Displays current mode, motor speeds, RC signals, and expandable sensor data
"""

import tkinter as tk
from tkinter import ttk, font, messagebox
import serial
import json
import time
import threading
from datetime import datetime
import queue
import math
import paramiko
import subprocess
import os

class RoverDashboard:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("🤖 Rover Control Dashboard")
        self.root.geometry("1400x900")
        self.root.configure(bg='#1a1a1a')
        
        # Serial connection
        self.serial_conn = None
        self.running = False
        self.data_queue = queue.Queue()
        
        # Current robot state
        self.robot_state = {
            'mode': 'UNKNOWN',
            'rc_ch1': 0,
            'rc_ch2': 0, 
            'rc_ch9': 0,
            'rc_valid': False,
            'motor_left': 0,
            'motor_right': 0,
            'last_update': None,
            'connection_status': 'DISCONNECTED',
            'ultrasonic_distance': None,
            'battery_voltage': None,
            'signal_strength': 0,
            'roam_state': 'UNKNOWN',
            'roam_speed': 0,
            'collision_detected': False
        }
        
        # SSH Configuration
        self.ssh_config = {
            'host': '192.168.254.62',
            'username': 'jay',
            'work_dir': '~/rover_project',
            'venv_activate': 'source venv/bin/activate'
        }
        
        # SSH connection
        self.ssh_client = None
        self.ssh_connected = False
        self.running_program = None
        self.program_output = []
        
        self.setup_ui()
        self.setup_serial()
        self.start_data_processor()
        
    def setup_ui(self):
        """Setup the dashboard UI"""
        # Configure styles
        style = ttk.Style()
        style.theme_use('clam')
        
        # Main container
        main_frame = tk.Frame(self.root, bg='#1a1a1a')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Title
        title_font = font.Font(family="Arial", size=24, weight="bold")
        title_label = tk.Label(main_frame, text="🤖 ROVER CONTROL DASHBOARD", 
                              font=title_font, fg='#00ff00', bg='#1a1a1a')
        title_label.pack(pady=(0, 20))
        
        # Top row - Status indicators
        self.setup_status_row(main_frame)
        
        # Second row - SSH Control Panel
        self.setup_ssh_control_row(main_frame)
        
        # Third row - Motor and RC controls
        self.setup_control_row(main_frame)
        
        # Fourth row - Sensor data
        self.setup_sensor_row(main_frame)
        
        # Log area
        self.setup_log_area(main_frame)
        
    def setup_status_row(self, parent):
        """Setup status indicators row"""
        status_frame = tk.Frame(parent, bg='#1a1a1a')
        status_frame.pack(fill=tk.X, pady=(0, 20))
        
        # Mode indicator
        mode_frame = tk.LabelFrame(status_frame, text="CONTROL MODE", 
                                  fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        mode_frame.pack(side=tk.LEFT, padx=(0, 20), fill=tk.BOTH, expand=True)
        
        self.mode_label = tk.Label(mode_frame, text="UNKNOWN", 
                                  font=('Arial', 18, 'bold'), fg='white', bg='#2a2a2a')
        self.mode_label.pack(pady=20)
        
        # Connection status
        conn_frame = tk.LabelFrame(status_frame, text="CONNECTION", 
                                  fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        conn_frame.pack(side=tk.LEFT, padx=(0, 20), fill=tk.BOTH, expand=True)
        
        self.conn_label = tk.Label(conn_frame, text="DISCONNECTED", 
                                  font=('Arial', 14, 'bold'), fg='red', bg='#2a2a2a')
        self.conn_label.pack(pady=20)
        
        # RC Signal status
        rc_frame = tk.LabelFrame(status_frame, text="RC SIGNAL", 
                                fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        rc_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.rc_status_label = tk.Label(rc_frame, text="INVALID", 
                                       font=('Arial', 14, 'bold'), fg='red', bg='#2a2a2a')
        self.rc_status_label.pack(pady=20)
    
    def setup_ssh_control_row(self, parent):
        """Setup SSH remote control panel"""
        ssh_frame = tk.Frame(parent, bg='#1a1a1a')
        ssh_frame.pack(fill=tk.X, pady=(0, 20))
        
        # SSH Connection Status
        ssh_status_frame = tk.LabelFrame(ssh_frame, text="SSH CONNECTION", 
                                        fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        ssh_status_frame.pack(side=tk.LEFT, padx=(0, 20), fill=tk.BOTH, expand=True)
        
        ssh_status_inner = tk.Frame(ssh_status_frame, bg='#2a2a2a')
        ssh_status_inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Host info
        host_frame = tk.Frame(ssh_status_inner, bg='#2a2a2a')
        host_frame.pack(fill=tk.X, pady=2)
        tk.Label(host_frame, text="Host:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=8, anchor='w').pack(side=tk.LEFT)
        self.ssh_host_label = tk.Label(host_frame, text=f"jay@{self.ssh_config['host']}", 
                                      font=('Arial', 10, 'bold'), fg='#00ff00', bg='#2a2a2a', anchor='w')
        self.ssh_host_label.pack(side=tk.LEFT)
        
        # Connection status
        conn_status_frame = tk.Frame(ssh_status_inner, bg='#2a2a2a')
        conn_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(conn_status_frame, text="Status:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=8, anchor='w').pack(side=tk.LEFT)
        self.ssh_status_label = tk.Label(conn_status_frame, text="DISCONNECTED", 
                                        font=('Arial', 10, 'bold'), fg='#ff0000', bg='#2a2a2a', anchor='w')
        self.ssh_status_label.pack(side=tk.LEFT)
        
        # Connect button
        self.ssh_connect_btn = tk.Button(ssh_status_inner, text="Connect SSH", 
                                        command=self.connect_ssh, bg='#004080', fg='white',
                                        font=('Arial', 10, 'bold'), relief='raised')
        self.ssh_connect_btn.pack(pady=5)
        
        # Program Launcher
        launcher_frame = tk.LabelFrame(ssh_frame, text="PROGRAM LAUNCHER", 
                                      fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        launcher_frame.pack(side=tk.LEFT, padx=(0, 20), fill=tk.BOTH, expand=True)
        
        launcher_inner = tk.Frame(launcher_frame, bg='#2a2a2a')
        launcher_inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Program selection
        tk.Label(launcher_inner, text="Select Program:", font=('Arial', 10, 'bold'), 
                fg='white', bg='#2a2a2a').pack(anchor='w')
        
        self.program_var = tk.StringVar()
        self.program_combo = ttk.Combobox(launcher_inner, textvariable=self.program_var, 
                                         font=('Arial', 10), width=35, state='readonly')
        self.program_combo['values'] = [
            'autonomous_always_thinking.py',
            'autonomous_controller_smooth.py', 
            'autonomous_controller_with_sensors.py',
            'autonomous_controller_lidar.py',
            'autonomous_test_simple.py',
            'rover_dashboard.py'
        ]
        self.program_combo.set('autonomous_always_thinking.py')
        self.program_combo.pack(pady=5, fill=tk.X)
        
        # Launch buttons
        button_frame = tk.Frame(launcher_inner, bg='#2a2a2a')
        button_frame.pack(fill=tk.X, pady=5)
        
        self.launch_btn = tk.Button(button_frame, text="🚀 Launch Program", 
                                   command=self.launch_program, bg='#008000', fg='white',
                                   font=('Arial', 10, 'bold'), relief='raised')
        self.launch_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.stop_btn = tk.Button(button_frame, text="🛑 Stop Program", 
                                 command=self.stop_program, bg='#800000', fg='white',
                                 font=('Arial', 10, 'bold'), relief='raised', state='disabled')
        self.stop_btn.pack(side=tk.LEFT)
        
        # Running program status
        status_frame = tk.Frame(launcher_inner, bg='#2a2a2a')
        status_frame.pack(fill=tk.X, pady=2)
        tk.Label(status_frame, text="Running:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=8, anchor='w').pack(side=tk.LEFT)
        self.running_program_label = tk.Label(status_frame, text="None", 
                                             font=('Arial', 10, 'bold'), fg='#808080', bg='#2a2a2a', anchor='w')
        self.running_program_label.pack(side=tk.LEFT)
        
        # Quick Commands
        quick_cmd_frame = tk.LabelFrame(ssh_frame, text="QUICK COMMANDS", 
                                       fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        quick_cmd_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        quick_cmd_inner = tk.Frame(quick_cmd_frame, bg='#2a2a2a')
        quick_cmd_inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Quick command buttons
        self.reboot_btn = tk.Button(quick_cmd_inner, text="🔄 Reboot Pi", 
                                   command=self.reboot_pi, bg='#404000', fg='white',
                                   font=('Arial', 9, 'bold'), relief='raised')
        self.reboot_btn.pack(fill=tk.X, pady=2)
        
        self.git_pull_btn = tk.Button(quick_cmd_inner, text="📥 Git Pull", 
                                     command=self.git_pull, bg='#404000', fg='white',
                                     font=('Arial', 9, 'bold'), relief='raised')
        self.git_pull_btn.pack(fill=tk.X, pady=2)
        
        self.list_processes_btn = tk.Button(quick_cmd_inner, text="📋 List Processes", 
                                           command=self.list_processes, bg='#404000', fg='white',
                                           font=('Arial', 9, 'bold'), relief='raised')
        self.list_processes_btn.pack(fill=tk.X, pady=2)
        
    def setup_control_row(self, parent):
        """Setup motor and RC control displays"""
        control_frame = tk.Frame(parent, bg='#1a1a1a')
        control_frame.pack(fill=tk.X, pady=(0, 20))
        
        # Motor speeds
        motor_frame = tk.LabelFrame(control_frame, text="MOTOR SPEEDS", 
                                   fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        motor_frame.pack(side=tk.LEFT, padx=(0, 20), fill=tk.BOTH, expand=True)
        
        self.setup_motor_gauges(motor_frame)
        
        # RC channels
        rc_frame = tk.LabelFrame(control_frame, text="RC CHANNELS", 
                                fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        rc_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.setup_rc_displays(rc_frame)
        
    def setup_motor_gauges(self, parent):
        """Setup motor speed gauges"""
        gauge_frame = tk.Frame(parent, bg='#2a2a2a')
        gauge_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left motor
        left_frame = tk.Frame(gauge_frame, bg='#2a2a2a')
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        tk.Label(left_frame, text="LEFT", font=('Arial', 14, 'bold'), 
                fg='#00ff00', bg='#2a2a2a').pack()
        
        self.left_motor_canvas = tk.Canvas(left_frame, width=120, height=120, 
                                          bg='#1a1a1a', highlightthickness=0)
        self.left_motor_canvas.pack(pady=5)
        
        self.left_motor_value = tk.Label(left_frame, text="0%", 
                                        font=('Arial', 12, 'bold'), fg='white', bg='#2a2a2a')
        self.left_motor_value.pack()
        
        # Right motor
        right_frame = tk.Frame(gauge_frame, bg='#2a2a2a')
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        tk.Label(right_frame, text="RIGHT", font=('Arial', 14, 'bold'), 
                fg='#00ff00', bg='#2a2a2a').pack()
        
        self.right_motor_canvas = tk.Canvas(right_frame, width=120, height=120, 
                                           bg='#1a1a1a', highlightthickness=0)
        self.right_motor_canvas.pack(pady=5)
        
        self.right_motor_value = tk.Label(right_frame, text="0%", 
                                         font=('Arial', 12, 'bold'), fg='white', bg='#2a2a2a')
        self.right_motor_value.pack()
        
    def setup_rc_displays(self, parent):
        """Setup RC channel displays"""
        rc_inner = tk.Frame(parent, bg='#2a2a2a')
        rc_inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # CH1 (Throttle)
        ch1_frame = tk.Frame(rc_inner, bg='#2a2a2a')
        ch1_frame.pack(fill=tk.X, pady=2)
        tk.Label(ch1_frame, text="CH1 (Throttle):", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=15, anchor='w').pack(side=tk.LEFT)
        self.ch1_label = tk.Label(ch1_frame, text="0", font=('Arial', 10, 'bold'), 
                                 fg='#00ff00', bg='#2a2a2a', width=8, anchor='e')
        self.ch1_label.pack(side=tk.RIGHT)
        
        # CH2 (Steering)
        ch2_frame = tk.Frame(rc_inner, bg='#2a2a2a')
        ch2_frame.pack(fill=tk.X, pady=2)
        tk.Label(ch2_frame, text="CH2 (Steering):", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=15, anchor='w').pack(side=tk.LEFT)
        self.ch2_label = tk.Label(ch2_frame, text="0", font=('Arial', 10, 'bold'), 
                                 fg='#00ff00', bg='#2a2a2a', width=8, anchor='e')
        self.ch2_label.pack(side=tk.RIGHT)
        
        # CH9 (Mode)
        ch9_frame = tk.Frame(rc_inner, bg='#2a2a2a')
        ch9_frame.pack(fill=tk.X, pady=2)
        tk.Label(ch9_frame, text="CH9 (Mode):", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=15, anchor='w').pack(side=tk.LEFT)
        self.ch9_label = tk.Label(ch9_frame, text="0", font=('Arial', 10, 'bold'), 
                                 fg='#00ff00', bg='#2a2a2a', width=8, anchor='e')
        self.ch9_label.pack(side=tk.RIGHT)
        
    def setup_sensor_row(self, parent):
        """Setup sensor data displays"""
        sensor_frame = tk.Frame(parent, bg='#1a1a1a')
        sensor_frame.pack(fill=tk.X, pady=(0, 20))
        
        # Ultrasonic sensor
        ultrasonic_frame = tk.LabelFrame(sensor_frame, text="ULTRASONIC SENSOR", 
                                        fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        ultrasonic_frame.pack(side=tk.LEFT, padx=(0, 20), fill=tk.BOTH, expand=True)
        
        self.ultrasonic_canvas = tk.Canvas(ultrasonic_frame, width=200, height=100, 
                                          bg='#1a1a1a', highlightthickness=0)
        self.ultrasonic_canvas.pack(pady=10)
        
        self.ultrasonic_label = tk.Label(ultrasonic_frame, text="No Data", 
                                        font=('Arial', 12, 'bold'), fg='white', bg='#2a2a2a')
        self.ultrasonic_label.pack()
        
        # Roaming status
        roaming_frame = tk.LabelFrame(sensor_frame, text="AUTONOMOUS ROAMING", 
                                     fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        roaming_frame.pack(side=tk.LEFT, padx=(0, 20), fill=tk.BOTH, expand=True)
        
        roaming_inner = tk.Frame(roaming_frame, bg='#2a2a2a')
        roaming_inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Roaming state
        state_frame = tk.Frame(roaming_inner, bg='#2a2a2a')
        state_frame.pack(fill=tk.X, pady=2)
        tk.Label(state_frame, text="State:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=8, anchor='w').pack(side=tk.LEFT)
        self.roam_state_label = tk.Label(state_frame, text="INACTIVE", font=('Arial', 10, 'bold'), 
                                        fg='#808080', bg='#2a2a2a', width=12, anchor='e')
        self.roam_state_label.pack(side=tk.RIGHT)
        
        # Roaming speed
        speed_frame = tk.Frame(roaming_inner, bg='#2a2a2a')
        speed_frame.pack(fill=tk.X, pady=2)
        tk.Label(speed_frame, text="Speed:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=8, anchor='w').pack(side=tk.LEFT)
        self.roam_speed_label = tk.Label(speed_frame, text="0%", font=('Arial', 10, 'bold'), 
                                        fg='#00ff00', bg='#2a2a2a', width=12, anchor='e')
        self.roam_speed_label.pack(side=tk.RIGHT)
        
        # Collision status
        collision_frame = tk.Frame(roaming_inner, bg='#2a2a2a')
        collision_frame.pack(fill=tk.X, pady=2)
        tk.Label(collision_frame, text="Collision:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=8, anchor='w').pack(side=tk.LEFT)
        self.collision_label = tk.Label(collision_frame, text="CLEAR", font=('Arial', 10, 'bold'), 
                                       fg='#00ff00', bg='#2a2a2a', width=12, anchor='e')
        self.collision_label.pack(side=tk.RIGHT)
        
        # System info
        system_frame = tk.LabelFrame(sensor_frame, text="SYSTEM INFO", 
                                    fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        system_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        system_inner = tk.Frame(system_frame, bg='#2a2a2a')
        system_inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Battery voltage
        batt_frame = tk.Frame(system_inner, bg='#2a2a2a')
        batt_frame.pack(fill=tk.X, pady=2)
        tk.Label(batt_frame, text="Battery:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=12, anchor='w').pack(side=tk.LEFT)
        self.battery_label = tk.Label(batt_frame, text="N/A", font=('Arial', 10, 'bold'), 
                                     fg='#00ff00', bg='#2a2a2a', width=10, anchor='e')
        self.battery_label.pack(side=tk.RIGHT)
        
        # Signal strength
        signal_frame = tk.Frame(system_inner, bg='#2a2a2a')
        signal_frame.pack(fill=tk.X, pady=2)
        tk.Label(signal_frame, text="Signal:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=12, anchor='w').pack(side=tk.LEFT)
        self.signal_label = tk.Label(signal_frame, text="N/A", font=('Arial', 10, 'bold'), 
                                    fg='#00ff00', bg='#2a2a2a', width=10, anchor='e')
        self.signal_label.pack(side=tk.RIGHT)
        
        # Uptime
        uptime_frame = tk.Frame(system_inner, bg='#2a2a2a')
        uptime_frame.pack(fill=tk.X, pady=2)
        tk.Label(uptime_frame, text="Uptime:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=12, anchor='w').pack(side=tk.LEFT)
        self.uptime_label = tk.Label(uptime_frame, text="00:00:00", font=('Arial', 10, 'bold'), 
                                    fg='#00ff00', bg='#2a2a2a', width=10, anchor='e')
        self.uptime_label.pack(side=tk.RIGHT)
        
    def setup_log_area(self, parent):
        """Setup log/message area"""
        log_frame = tk.LabelFrame(parent, text="MESSAGE LOG", 
                                 fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        # Scrollable text widget
        self.log_text = tk.Text(log_frame, height=8, bg='#1a1a1a', fg='#00ff00', 
                               font=('Courier', 9), wrap=tk.WORD)
        scrollbar = tk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, pady=5)
        
        # Add initial message
        self.log_message("🚀 Dashboard initialized. Attempting to connect to rover...")
        
    def setup_serial(self):
        """Setup serial connection to rover"""
        ports_to_try = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1']
        
        for port in ports_to_try:
            try:
                self.serial_conn = serial.Serial(port, 115200, timeout=1)
                time.sleep(2)  # Wait for Arduino
                self.robot_state['connection_status'] = 'CONNECTED'
                self.log_message(f"✅ Connected to rover on {port}")
                
                # Start serial reading thread
                self.running = True
                self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
                self.serial_thread.start()
                break
                
            except Exception as e:
                self.log_message(f"❌ Failed to connect on {port}: {e}")
                continue
        
        if not self.serial_conn:
            self.log_message("🔄 No rover found. Will keep trying...")
            self.root.after(5000, self.setup_serial)  # Retry in 5 seconds
    
    def read_serial_data(self):
        """Read data from rover in background thread"""
        while self.running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode().strip()
                    if line:
                        self.data_queue.put(('serial', line))
            except Exception as e:
                self.data_queue.put(('error', f"Serial error: {e}"))
                time.sleep(0.1)
            time.sleep(0.01)
    
    def start_data_processor(self):
        """Start processing queued data"""
        self.process_data_queue()
        self.update_displays()
        
    def process_data_queue(self):
        """Process incoming data from rover"""
        try:
            while not self.data_queue.empty():
                msg_type, data = self.data_queue.get_nowait()
                
                if msg_type == 'serial':
                    self.process_rover_message(data)
                elif msg_type == 'error':
                    self.log_message(f"❌ {data}")
                    
        except queue.Empty:
            pass
        
        # Schedule next processing
        self.root.after(50, self.process_data_queue)
    
    def process_rover_message(self, message):
        """Process incoming message from rover"""
        try:
            if message.startswith('{') and message.endswith('}'):
                data = json.loads(message)
                
                # RC data
                if 'ch1' in data:
                    self.robot_state['rc_ch1'] = data.get('ch1', 0)
                    self.robot_state['rc_ch2'] = data.get('ch2', 0)
                    self.robot_state['rc_ch9'] = data.get('ch9', 0)
                    self.robot_state['rc_valid'] = data.get('valid', False)
                    self.robot_state['last_update'] = time.time()
                
                # Mode changes
                elif 'mode_change' in data:
                    self.robot_state['mode'] = data['mode_change']
                    self.log_message(f"🔄 Mode: {self.robot_state['mode']}")
                
                # Motor acknowledgments
                elif 'motor_ack' in data:
                    ack = data['motor_ack']
                    self.robot_state['motor_left'] = ack.get('left', 0)
                    self.robot_state['motor_right'] = ack.get('right', 0)
                
                # Sensor data
                elif 'sensors' in data:
                    sensors = data['sensors']
                    if 'front_distance' in sensors:
                        self.robot_state['ultrasonic_distance'] = sensors['front_distance']
                    if 'collision_detected' in sensors:
                        self.robot_state['collision_detected'] = sensors['collision_detected']
                    if 'battery' in sensors:
                        self.robot_state['battery_voltage'] = sensors['battery']
                
                # Roaming status
                elif 'roaming' in data:
                    roaming = data['roaming']
                    self.robot_state['roam_state'] = roaming.get('state', 'UNKNOWN')
                    self.robot_state['roam_speed'] = roaming.get('speed', 0)
                    if 'collision_detected' in roaming:
                        self.robot_state['collision_detected'] = roaming['collision_detected']
            else:
                # Non-JSON messages
                self.log_message(f"🔧 {message}")
                
        except json.JSONDecodeError:
            self.log_message(f"📝 {message}")
    
    def update_displays(self):
        """Update all dashboard displays"""
        # Mode display
        mode = self.robot_state['mode']
        mode_colors = {
            'FAILSAFE': '#ff0000',
            'MANUAL': '#00ff00', 
            'AUTONOMOUS': '#0080ff',
            'UNKNOWN': '#808080'
        }
        self.mode_label.config(text=mode, fg=mode_colors.get(mode, '#808080'))
        
        # Connection status
        if self.robot_state['connection_status'] == 'CONNECTED':
            self.conn_label.config(text="CONNECTED", fg='#00ff00')
        else:
            self.conn_label.config(text="DISCONNECTED", fg='#ff0000')
        
        # RC signal status
        if self.robot_state['rc_valid']:
            self.rc_status_label.config(text="VALID", fg='#00ff00')
        else:
            self.rc_status_label.config(text="INVALID", fg='#ff0000')
        
        # RC channels
        self.ch1_label.config(text=f"{self.robot_state['rc_ch1']:+d}")
        self.ch2_label.config(text=f"{self.robot_state['rc_ch2']:+d}")
        self.ch9_label.config(text=f"{self.robot_state['rc_ch9']:+d}")
        
        # Motor gauges
        self.draw_motor_gauge(self.left_motor_canvas, self.robot_state['motor_left'])
        self.draw_motor_gauge(self.right_motor_canvas, self.robot_state['motor_right'])
        
        # Motor values
        left_pct = int((self.robot_state['motor_left'] / 255) * 100)
        right_pct = int((self.robot_state['motor_right'] / 255) * 100)
        self.left_motor_value.config(text=f"{left_pct:+d}%")
        self.right_motor_value.config(text=f"{right_pct:+d}%")
        
        # Ultrasonic display
        self.draw_ultrasonic_display()
        
        # Roaming status display
        self.update_roaming_display()
        
        # System info
        if self.robot_state['battery_voltage']:
            self.battery_label.config(text=f"{self.robot_state['battery_voltage']:.1f}V")
        
        # Schedule next update
        self.root.after(100, self.update_displays)
    
    def draw_motor_gauge(self, canvas, speed):
        """Draw circular motor speed gauge"""
        canvas.delete("all")
        
        # Gauge background
        canvas.create_oval(10, 10, 110, 110, outline='#404040', width=3)
        
        # Speed percentage
        speed_pct = (speed / 255) * 100
        
        # Determine color based on speed
        if speed == 0:
            color = '#404040'
        elif speed > 0:
            color = '#00ff00'  # Green for forward
        else:
            color = '#ff8000'  # Orange for reverse
        
        # Draw speed arc
        if speed != 0:
            # Convert speed to angle (0-360 degrees)
            angle = abs(speed_pct) * 3.6  # 100% = 360 degrees
            canvas.create_arc(15, 15, 105, 105, start=90, extent=-angle, 
                            outline=color, width=8, style='arc')
        
        # Center dot
        canvas.create_oval(55, 55, 65, 65, fill=color, outline=color)
        
        # Direction indicator
        if speed > 0:
            canvas.create_text(60, 25, text="↑", fill='#00ff00', font=('Arial', 16, 'bold'))
        elif speed < 0:
            canvas.create_text(60, 95, text="↓", fill='#ff8000', font=('Arial', 16, 'bold'))
    
    def draw_ultrasonic_display(self):
        """Draw ultrasonic sensor visualization"""
        self.ultrasonic_canvas.delete("all")
        
        distance = self.robot_state['ultrasonic_distance']
        
        if distance is None:
            self.ultrasonic_canvas.create_text(100, 50, text="NO DATA", 
                                             fill='#808080', font=('Arial', 12, 'bold'))
            self.ultrasonic_label.config(text="No Data", fg='#808080')
            return
        
        # Draw sensor visualization
        max_range = 400  # cm
        distance = min(distance, max_range)
        
        # Sensor position
        sensor_x, sensor_y = 20, 50
        
        # Draw sensor beam
        beam_length = (distance / max_range) * 160
        
        # Color based on distance
        if distance < 20:
            color = '#ff0000'  # Red - too close
        elif distance < 100:
            color = '#ff8000'  # Orange - close
        else:
            color = '#00ff00'  # Green - safe
        
        # Draw beam
        self.ultrasonic_canvas.create_line(sensor_x, sensor_y, 
                                         sensor_x + beam_length, sensor_y, 
                                         fill=color, width=4)
        
        # Draw object
        if distance < max_range:
            obj_x = sensor_x + beam_length
            self.ultrasonic_canvas.create_rectangle(obj_x-3, sensor_y-10, 
                                                  obj_x+3, sensor_y+10, 
                                                  fill=color, outline=color)
        
        # Distance text
        self.ultrasonic_label.config(text=f"{distance:.1f} cm", fg=color)
    
    def log_message(self, message):
        """Add message to log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        
        # Limit log size
        lines = self.log_text.get("1.0", tk.END).split('\n')
        if len(lines) > 100:
            self.log_text.delete("1.0", "10.0")
    
    def update_roaming_display(self):
        """Update roaming status display"""
        # Roaming state
        roam_state = self.robot_state.get('roam_state', 'INACTIVE')
        
        # State colors
        state_colors = {
            'STRAIGHT': '#00ff00',     # Green - confident forward movement
            'SLOWING': '#ffff00',      # Yellow - approaching obstacle
            'CONFIRMING': '#ff8000',   # Orange - stopped to confirm
            'AVOIDING': '#0080ff',     # Blue - turning to avoid
            'INACTIVE': '#808080'      # Gray
        }
        
        color = state_colors.get(roam_state, '#808080')
        self.roam_state_label.config(text=roam_state, fg=color)
        
        # Roaming speed
        roam_speed = self.robot_state.get('roam_speed', 0)
        speed_pct = int((roam_speed / 255) * 100) if roam_speed > 0 else 0
        self.roam_speed_label.config(text=f"{speed_pct}%")
        
        # Collision status
        collision = self.robot_state.get('collision_detected', False)
        if collision:
            self.collision_label.config(text="DETECTED", fg='#ff0000')
        else:
            self.collision_label.config(text="CLEAR", fg='#00ff00')
    
    # SSH Control Methods
    def connect_ssh(self):
        """Connect to Pi via SSH"""
        def ssh_connect_thread():
            try:
                self.ssh_client = paramiko.SSHClient()
                self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                
                # Try to connect
                self.ssh_client.connect(
                    hostname=self.ssh_config['host'],
                    username=self.ssh_config['username'],
                    timeout=10
                )
                
                self.ssh_connected = True
                self.root.after(0, lambda: self.update_ssh_status("CONNECTED", "#00ff00"))
                self.root.after(0, lambda: self.log_message(f"✅ SSH connected to {self.ssh_config['host']}"))
                
            except Exception as e:
                self.ssh_connected = False
                self.root.after(0, lambda: self.update_ssh_status("FAILED", "#ff0000"))
                self.root.after(0, lambda: self.log_message(f"❌ SSH connection failed: {e}"))
        
        # Update UI immediately
        self.update_ssh_status("CONNECTING...", "#ffff00")
        self.ssh_connect_btn.config(state='disabled')
        
        # Connect in background thread
        thread = threading.Thread(target=ssh_connect_thread, daemon=True)
        thread.start()
    
    def update_ssh_status(self, status, color):
        """Update SSH status display"""
        self.ssh_status_label.config(text=status, fg=color)
        
        if status == "CONNECTED":
            self.ssh_connect_btn.config(text="Disconnect SSH", command=self.disconnect_ssh, state='normal')
            self.launch_btn.config(state='normal')
            for btn in [self.reboot_btn, self.git_pull_btn, self.list_processes_btn]:
                btn.config(state='normal')
        else:
            self.ssh_connect_btn.config(text="Connect SSH", command=self.connect_ssh, state='normal')
            self.launch_btn.config(state='disabled')
            self.stop_btn.config(state='disabled')
            for btn in [self.reboot_btn, self.git_pull_btn, self.list_processes_btn]:
                btn.config(state='disabled')
    
    def disconnect_ssh(self):
        """Disconnect SSH"""
        if self.ssh_client:
            self.ssh_client.close()
        self.ssh_connected = False
        self.update_ssh_status("DISCONNECTED", "#ff0000")
        self.log_message("🔌 SSH disconnected")
    
    def launch_program(self):
        """Launch selected program on Pi"""
        if not self.ssh_connected:
            messagebox.showerror("Error", "SSH not connected!")
            return
        
        program = self.program_var.get()
        if not program:
            messagebox.showerror("Error", "No program selected!")
            return
        
        def launch_thread():
            try:
                # Build command using your exact workflow
                command = f"""
                cd {self.ssh_config['work_dir']} && 
                {self.ssh_config['venv_activate']} && 
                python3 {program}
                """
                
                self.root.after(0, lambda: self.log_message(f"🚀 Launching {program}..."))
                
                # Execute command
                stdin, stdout, stderr = self.ssh_client.exec_command(command)
                
                # Update UI
                self.running_program = program
                self.root.after(0, lambda: self.running_program_label.config(text=program, fg='#00ff00'))
                self.root.after(0, lambda: self.launch_btn.config(state='disabled'))
                self.root.after(0, lambda: self.stop_btn.config(state='normal'))
                
                # Monitor output
                self.monitor_program_output(stdout, stderr)
                
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"❌ Launch failed: {e}"))
        
        thread = threading.Thread(target=launch_thread, daemon=True)
        thread.start()
    
    def monitor_program_output(self, stdout, stderr):
        """Monitor program output in background"""
        def output_thread():
            try:
                # Read output lines
                for line in iter(stdout.readline, ""):
                    if line:
                        self.root.after(0, lambda l=line.strip(): self.log_message(f"📡 {l}"))
                
                # Program finished
                self.running_program = None
                self.root.after(0, lambda: self.running_program_label.config(text="None", fg='#808080'))
                self.root.after(0, lambda: self.launch_btn.config(state='normal'))
                self.root.after(0, lambda: self.stop_btn.config(state='disabled'))
                self.root.after(0, lambda: self.log_message("🏁 Program finished"))
                
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"❌ Output monitoring error: {e}"))
        
        thread = threading.Thread(target=output_thread, daemon=True)
        thread.start()
    
    def stop_program(self):
        """Stop running program"""
        if not self.ssh_connected or not self.running_program:
            return
        
        def stop_thread():
            try:
                # Kill python processes in the work directory
                kill_command = f"pkill -f 'python3.*{self.running_program}'"
                self.ssh_client.exec_command(kill_command)
                
                self.root.after(0, lambda: self.log_message(f"🛑 Stopped {self.running_program}"))
                
                # Update UI
                self.running_program = None
                self.root.after(0, lambda: self.running_program_label.config(text="None", fg='#808080'))
                self.root.after(0, lambda: self.launch_btn.config(state='normal'))
                self.root.after(0, lambda: self.stop_btn.config(state='disabled'))
                
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"❌ Stop failed: {e}"))
        
        thread = threading.Thread(target=stop_thread, daemon=True)
        thread.start()
    
    def reboot_pi(self):
        """Reboot the Pi"""
        if not self.ssh_connected:
            return
        
        if messagebox.askyesno("Confirm Reboot", "Are you sure you want to reboot the Pi?"):
            def reboot_thread():
                try:
                    self.ssh_client.exec_command("sudo reboot")
                    self.root.after(0, lambda: self.log_message("🔄 Pi reboot initiated"))
                    self.disconnect_ssh()
                except Exception as e:
                    self.root.after(0, lambda: self.log_message(f"❌ Reboot failed: {e}"))
            
            thread = threading.Thread(target=reboot_thread, daemon=True)
            thread.start()
    
    def git_pull(self):
        """Pull latest code from git"""
        if not self.ssh_connected:
            return
        
        def git_thread():
            try:
                command = f"cd {self.ssh_config['work_dir']} && git pull"
                stdin, stdout, stderr = self.ssh_client.exec_command(command)
                
                output = stdout.read().decode().strip()
                error = stderr.read().decode().strip()
                
                if output:
                    self.root.after(0, lambda: self.log_message(f"📥 Git: {output}"))
                if error and "Already up to date" not in error:
                    self.root.after(0, lambda: self.log_message(f"⚠️ Git: {error}"))
                
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"❌ Git pull failed: {e}"))
        
        thread = threading.Thread(target=git_thread, daemon=True)
        thread.start()
    
    def list_processes(self):
        """List running Python processes"""
        if not self.ssh_connected:
            return
        
        def ps_thread():
            try:
                command = "ps aux | grep python3 | grep -v grep"
                stdin, stdout, stderr = self.ssh_client.exec_command(command)
                
                output = stdout.read().decode().strip()
                if output:
                    processes = output.split('\n')
                    self.root.after(0, lambda: self.log_message(f"📋 Found {len(processes)} Python processes:"))
                    for proc in processes:
                        # Extract just the command part
                        parts = proc.split()
                        if len(parts) > 10:
                            cmd = ' '.join(parts[10:])
                            self.root.after(0, lambda c=cmd: self.log_message(f"   • {c}"))
                else:
                    self.root.after(0, lambda: self.log_message("📋 No Python processes found"))
                
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"❌ Process list failed: {e}"))
        
        thread = threading.Thread(target=ps_thread, daemon=True)
        thread.start()
    
    def run(self):
        """Start the dashboard"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if self.serial_conn:
                self.serial_conn.close()
            if self.ssh_client:
                self.ssh_client.close()

def main():
    print("🚀 Starting Rover Debug Dashboard...")
    dashboard = RoverDashboard()
    dashboard.run()

if __name__ == "__main__":
    main()