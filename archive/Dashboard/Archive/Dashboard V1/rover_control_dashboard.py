#!/usr/bin/env python3
"""
Rover Control Dashboard
Comprehensive GUI for rover monitoring and remote control
Includes SSH program launcher, real-time monitoring, and system controls
"""

import tkinter as tk
from tkinter import ttk, font, messagebox, filedialog
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
import stat
from collections import deque

class RoverControlDashboard:
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
            'collision_detected': False,
            # Pi telemetry data
            'pi_telemetry': None,
            'nav_state': 'UNKNOWN',
            'lidar_points': 0,
            'lidar_min_dist': None,
            'lidar_min_angle': None,
            'lidar_quality': 0,
            'lidar_sectors': {'front': True, 'left': True, 'right': True},
            'pi_cpu': 0,
            'pi_memory': 0,
            'pi_temperature': None,
            'pi_uptime': 0,
            'pi_disk': 0
        }
        
        # SSH Configuration
        self.ssh_config = {
            'host': '192.168.254.65',
            'username': 'jay',
            'work_dir': '~/rover_project',
            'venv_activate': 'source venv/bin/activate'
        }
        
        # SSH connection
        self.ssh_client = None
        self.ssh_connected = False
        self.running_program = None
        self.program_output = []
        
        # Terminal interface
        self.command_history = []
        self.history_index = -1
        
        # Debug logging system
        self.debug_log = deque(maxlen=2000)  # Keep last ~10 minutes of data
        self.session_start = datetime.now()
        self.log_file_path = None
        self.setup_debug_logging()
        
        self.setup_ui()
        self.setup_serial()
        self.start_data_processor()
        self.start_periodic_logging()
        
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
        
        # Fifth row - Pi Telemetry data
        self.setup_pi_telemetry_row(main_frame)
        
        # Sixth row - Terminal interface
        self.setup_terminal_row(main_frame)
        
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
        conn_frame = tk.LabelFrame(status_frame, text="SERIAL CONNECTION", 
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
            'autonomous_lidar_telemetry.py',  # NEW: LiDAR with telemetry
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
                                   font=('Arial', 10, 'bold'), relief='raised', state='disabled')
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
                                   font=('Arial', 9, 'bold'), relief='raised', state='disabled')
        self.reboot_btn.pack(fill=tk.X, pady=2)
        
        self.git_pull_btn = tk.Button(quick_cmd_inner, text="📥 Git Pull", 
                                     command=self.git_pull, bg='#404000', fg='white',
                                     font=('Arial', 9, 'bold'), relief='raised', state='disabled')
        self.git_pull_btn.pack(fill=tk.X, pady=2)
        
        self.list_processes_btn = tk.Button(quick_cmd_inner, text="📋 List Processes", 
                                           command=self.list_processes, bg='#404000', fg='white',
                                           font=('Arial', 9, 'bold'), relief='raised', state='disabled')
        self.list_processes_btn.pack(fill=tk.X, pady=2)
        
        self.upload_file_btn = tk.Button(quick_cmd_inner, text="📤 Upload Python File", 
                                        command=self.upload_python_file, bg='#004080', fg='white',
                                        font=('Arial', 9, 'bold'), relief='raised', state='disabled')
        self.upload_file_btn.pack(fill=tk.X, pady=2)
        
        self.save_log_btn = tk.Button(quick_cmd_inner, text="💾 Save Debug Log", 
                                     command=self.save_debug_log_manual, bg='#800080', fg='white',
                                     font=('Arial', 9, 'bold'), relief='raised')
        self.save_log_btn.pack(fill=tk.X, pady=2)
        
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
        
    def setup_pi_telemetry_row(self, parent):
        """Setup Pi telemetry data row"""
        telemetry_frame = tk.Frame(parent, bg='#1a1a1a')
        telemetry_frame.pack(fill=tk.X, pady=(10, 0))
        
        # LiDAR data panel
        lidar_frame = tk.LabelFrame(telemetry_frame, text="LIDAR DATA", 
                                   fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        lidar_frame.pack(side=tk.LEFT, padx=(0, 20), fill=tk.BOTH, expand=True)
        
        lidar_inner = tk.Frame(lidar_frame, bg='#2a2a2a')
        lidar_inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # LiDAR points
        points_frame = tk.Frame(lidar_inner, bg='#2a2a2a')
        points_frame.pack(fill=tk.X, pady=2)
        tk.Label(points_frame, text="Points:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=12, anchor='w').pack(side=tk.LEFT)
        self.lidar_points_label = tk.Label(points_frame, text="0", font=('Arial', 10, 'bold'), 
                                          fg='#00ff00', bg='#2a2a2a', width=10, anchor='e')
        self.lidar_points_label.pack(side=tk.RIGHT)
        
        # LiDAR minimum distance
        min_dist_frame = tk.Frame(lidar_inner, bg='#2a2a2a')
        min_dist_frame.pack(fill=tk.X, pady=2)
        tk.Label(min_dist_frame, text="Min Distance:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=12, anchor='w').pack(side=tk.LEFT)
        self.lidar_min_dist_label = tk.Label(min_dist_frame, text="N/A", font=('Arial', 10, 'bold'), 
                                            fg='#00ff00', bg='#2a2a2a', width=10, anchor='e')
        self.lidar_min_dist_label.pack(side=tk.RIGHT)
        
        # LiDAR quality
        quality_frame = tk.Frame(lidar_inner, bg='#2a2a2a')
        quality_frame.pack(fill=tk.X, pady=2)
        tk.Label(quality_frame, text="Quality:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=12, anchor='w').pack(side=tk.LEFT)
        self.lidar_quality_label = tk.Label(quality_frame, text="N/A", font=('Arial', 10, 'bold'), 
                                           fg='#00ff00', bg='#2a2a2a', width=10, anchor='e')
        self.lidar_quality_label.pack(side=tk.RIGHT)
        
        # Navigation state panel
        nav_frame = tk.LabelFrame(telemetry_frame, text="NAVIGATION STATE", 
                                 fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        nav_frame.pack(side=tk.LEFT, padx=(0, 20), fill=tk.BOTH, expand=True)
        
        nav_inner = tk.Frame(nav_frame, bg='#2a2a2a')
        nav_inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Navigation state
        nav_state_frame = tk.Frame(nav_inner, bg='#2a2a2a')
        nav_state_frame.pack(fill=tk.X, pady=2)
        tk.Label(nav_state_frame, text="State:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=10, anchor='w').pack(side=tk.LEFT)
        self.nav_state_label = tk.Label(nav_state_frame, text="UNKNOWN", font=('Arial', 10, 'bold'), 
                                       fg='#808080', bg='#2a2a2a', width=12, anchor='e')
        self.nav_state_label.pack(side=tk.RIGHT)
        
        # LiDAR sectors visualization
        sectors_frame = tk.Frame(nav_inner, bg='#2a2a2a')
        sectors_frame.pack(fill=tk.X, pady=5)
        tk.Label(sectors_frame, text="Sectors:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a').pack(anchor='w')
        
        self.sectors_canvas = tk.Canvas(sectors_frame, width=120, height=60, 
                                       bg='#1a1a1a', highlightthickness=0)
        self.sectors_canvas.pack(pady=(5, 0))
        
        # Pi system telemetry panel
        pi_system_frame = tk.LabelFrame(telemetry_frame, text="PI SYSTEM", 
                                       fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        pi_system_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        pi_system_inner = tk.Frame(pi_system_frame, bg='#2a2a2a')
        pi_system_inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Pi CPU
        cpu_frame = tk.Frame(pi_system_inner, bg='#2a2a2a')
        cpu_frame.pack(fill=tk.X, pady=2)
        tk.Label(cpu_frame, text="CPU:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=8, anchor='w').pack(side=tk.LEFT)
        self.pi_cpu_label = tk.Label(cpu_frame, text="0%", font=('Arial', 10, 'bold'), 
                                    fg='#00ff00', bg='#2a2a2a', width=10, anchor='e')
        self.pi_cpu_label.pack(side=tk.RIGHT)
        
        # Pi Memory
        memory_frame = tk.Frame(pi_system_inner, bg='#2a2a2a')
        memory_frame.pack(fill=tk.X, pady=2)
        tk.Label(memory_frame, text="Memory:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=8, anchor='w').pack(side=tk.LEFT)
        self.pi_memory_label = tk.Label(memory_frame, text="0%", font=('Arial', 10, 'bold'), 
                                       fg='#00ff00', bg='#2a2a2a', width=10, anchor='e')
        self.pi_memory_label.pack(side=tk.RIGHT)
        
        # Pi Temperature
        temp_frame = tk.Frame(pi_system_inner, bg='#2a2a2a')
        temp_frame.pack(fill=tk.X, pady=2)
        tk.Label(temp_frame, text="Temp:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a', width=8, anchor='w').pack(side=tk.LEFT)
        self.pi_temp_label = tk.Label(temp_frame, text="N/A", font=('Arial', 10, 'bold'), 
                                     fg='#00ff00', bg='#2a2a2a', width=10, anchor='e')
        self.pi_temp_label.pack(side=tk.RIGHT)
        
    def setup_terminal_row(self, parent):
        """Setup terminal/command line interface"""
        terminal_frame = tk.LabelFrame(parent, text="PI TERMINAL", 
                                      fg='white', bg='#2a2a2a', font=('Arial', 12, 'bold'))
        terminal_frame.pack(fill=tk.X, pady=(10, 0))
        
        terminal_inner = tk.Frame(terminal_frame, bg='#2a2a2a')
        terminal_inner.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Command input section
        input_frame = tk.Frame(terminal_inner, bg='#2a2a2a')
        input_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Command prompt label
        prompt_label = tk.Label(input_frame, text="jay@RoboPi:~$ ", 
                               font=('Courier', 10, 'bold'), fg='#00ff00', bg='#2a2a2a')
        prompt_label.pack(side=tk.LEFT)
        
        # Command input field
        self.command_entry = tk.Entry(input_frame, font=('Courier', 10), 
                                     bg='#1a1a1a', fg='#00ff00', insertbackground='#00ff00',
                                     relief=tk.FLAT, bd=5)
        self.command_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(5, 10))
        self.command_entry.bind('<Return>', self.execute_command)
        self.command_entry.bind('<Up>', self.command_history_up)
        self.command_entry.bind('<Down>', self.command_history_down)
        
        # Send command button
        self.send_cmd_btn = tk.Button(input_frame, text="Execute", 
                                     command=self.execute_command, bg='#008000', fg='white',
                                     font=('Arial', 9, 'bold'), relief='raised', state='disabled')
        self.send_cmd_btn.pack(side=tk.RIGHT)
        
        # Quick commands section
        quick_frame = tk.Frame(terminal_inner, bg='#2a2a2a')
        quick_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Label(quick_frame, text="Quick Commands:", font=('Arial', 10, 'bold'), 
                fg='white', bg='#2a2a2a').pack(anchor='w')
        
        # Quick command buttons
        quick_buttons_frame = tk.Frame(quick_frame, bg='#2a2a2a')
        quick_buttons_frame.pack(fill=tk.X, pady=(5, 0))
        
        quick_commands = [
            ("📦 pip list", "pip list"),
            ("📥 pip install [pkg]", "pip install "),
            ("📁 ls -la", "ls -la"),
            ("📂 pwd", "pwd"),
            ("🔍 ps aux", "ps aux | grep python"),
            ("🔄 source venv", "source venv/bin/activate")
        ]
        
        for i, (text, cmd) in enumerate(quick_commands):
            btn = tk.Button(quick_buttons_frame, text=text, 
                           command=lambda c=cmd: self.insert_command(c),
                           bg='#404040', fg='white', font=('Arial', 8),
                           relief='raised', state='disabled')
            btn.pack(side=tk.LEFT, padx=2, pady=2)
            
            # Store button reference for enabling/disabling
            if not hasattr(self, 'quick_cmd_buttons'):
                self.quick_cmd_buttons = []
            self.quick_cmd_buttons.append(btn)
        
        # Terminal output area
        output_frame = tk.Frame(terminal_inner, bg='#2a2a2a')
        output_frame.pack(fill=tk.BOTH, expand=True)
        
        tk.Label(output_frame, text="Terminal Output:", font=('Arial', 10, 'bold'), 
                fg='white', bg='#2a2a2a').pack(anchor='w')
        
        # Terminal output text widget
        self.terminal_output = tk.Text(output_frame, height=6, bg='#1a1a1a', fg='#00ff00', 
                                      font=('Courier', 9), wrap=tk.WORD, state=tk.DISABLED)
        terminal_scrollbar = tk.Scrollbar(output_frame, orient=tk.VERTICAL, command=self.terminal_output.yview)
        self.terminal_output.configure(yscrollcommand=terminal_scrollbar.set)
        
        self.terminal_output.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, pady=(5, 0))
        terminal_scrollbar.pack(side=tk.RIGHT, fill=tk.Y, pady=(5, 0))
        
        # Add initial terminal message
        self.add_terminal_output("💻 Pi Terminal ready - SSH connection required")
        self.add_terminal_output("💡 Tip: Use Up/Down arrows for command history")
        
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
        self.log_message("🚀 Rover Control Dashboard initialized")
        self.log_message("📡 Attempting to connect to rover...")
    
    # Debug Logging System
    def setup_debug_logging(self):
        """Initialize debug logging system"""
        try:
            # Create Data Logs directory
            self.logs_dir = "/mnt/d/Mini Rover Development/Data Logs"
            os.makedirs(self.logs_dir, exist_ok=True)
            
            # Initialize session log
            session_id = self.session_start.strftime("%Y%m%d_%H%M%S")
            self.log_file_path = os.path.join(self.logs_dir, f"rover_debug_{session_id}.txt")
            
            # Log session start
            self.log_debug("SESSION", "Debug logging initialized")
            self.log_debug("SYSTEM", f"Dashboard version: Rover Control v2.0")
            self.log_debug("SYSTEM", f"Session ID: {session_id}")
            self.log_debug("SYSTEM", f"Log file: {self.log_file_path}")
            
        except Exception as e:
            print(f"Debug logging setup failed: {e}")
    
    def log_debug(self, category, message, data=None):
        """Add entry to debug log with structured format"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        # Create log entry
        entry = {
            'timestamp': timestamp,
            'category': category,
            'message': message,
            'data': data
        }
        
        # Add to circular buffer
        self.debug_log.append(entry)
        
        # Format for display (optional - could be made configurable)
        if category in ['ERROR', 'CONNECTION', 'PROGRAM']:
            formatted = f"[{timestamp}] {category}: {message}"
            if data:
                formatted += f" | Data: {data}"
            print(f"DEBUG: {formatted}")
    
    def save_debug_log(self, reason="periodic"):
        """Save current debug log buffer to file"""
        if not self.log_file_path or not self.debug_log:
            return
        
        try:
            with open(self.log_file_path, 'w', encoding='utf-8') as f:
                # Write header
                f.write("=" * 80 + "\n")
                f.write("ROVER DEBUG LOG\n")
                f.write("=" * 80 + "\n")
                f.write(f"Session Start: {self.session_start.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Log Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Save Reason: {reason}\n")
                f.write(f"Total Entries: {len(self.debug_log)}\n")
                f.write("=" * 80 + "\n\n")
                
                # Write system info
                f.write("SYSTEM INFORMATION:\n")
                f.write("-" * 40 + "\n")
                f.write(f"Robot State: {self.robot_state['mode']}\n")
                f.write(f"Serial Status: {self.robot_state['connection_status']}\n")
                f.write(f"SSH Status: {'CONNECTED' if self.ssh_connected else 'DISCONNECTED'}\n")
                f.write(f"Running Program: {self.running_program or 'None'}\n")
                f.write(f"RC Valid: {self.robot_state['rc_valid']}\n")
                f.write(f"Last Update: {self.robot_state['last_update']}\n")
                f.write("\n")
                
                # Write current telemetry snapshot
                f.write("CURRENT TELEMETRY SNAPSHOT:\n")
                f.write("-" * 40 + "\n")
                f.write(f"Ultrasonic Distance: {self.robot_state.get('ultrasonic_distance', 'N/A')}\n")
                f.write(f"Motor Left: {self.robot_state.get('motor_left', 0)}\n")
                f.write(f"Motor Right: {self.robot_state.get('motor_right', 0)}\n")
                f.write(f"RC CH1: {self.robot_state.get('rc_ch1', 0)}\n")
                f.write(f"RC CH2: {self.robot_state.get('rc_ch2', 0)}\n")
                f.write(f"RC CH9: {self.robot_state.get('rc_ch9', 0)}\n")
                
                # Pi telemetry if available
                if self.robot_state.get('pi_telemetry'):
                    f.write(f"Navigation State: {self.robot_state.get('nav_state', 'N/A')}\n")
                    f.write(f"LiDAR Points: {self.robot_state.get('lidar_points', 0)}\n")
                    f.write(f"LiDAR Min Distance: {self.robot_state.get('lidar_min_dist', 'N/A')}\n")
                    f.write(f"Pi CPU: {self.robot_state.get('pi_cpu', 0)}%\n")
                    f.write(f"Pi Memory: {self.robot_state.get('pi_memory', 0)}%\n")
                    f.write(f"Pi Temperature: {self.robot_state.get('pi_temperature', 'N/A')}°C\n")
                f.write("\n")
                
                # Write detailed log entries
                f.write("DETAILED LOG ENTRIES:\n")
                f.write("-" * 40 + "\n")
                
                for entry in self.debug_log:
                    f.write(f"[{entry['timestamp']}] {entry['category']}: {entry['message']}\n")
                    if entry['data']:
                        # Pretty print data based on type
                        if isinstance(entry['data'], dict):
                            for key, value in entry['data'].items():
                                f.write(f"    {key}: {value}\n")
                        elif isinstance(entry['data'], str) and len(entry['data']) > 100:
                            # Truncate very long strings
                            f.write(f"    Data: {entry['data'][:100]}...\n")
                        else:
                            f.write(f"    Data: {entry['data']}\n")
                    f.write("\n")
                
                f.write("=" * 80 + "\n")
                f.write("END OF LOG\n")
                f.write("=" * 80 + "\n")
                
            self.log_debug("SYSTEM", f"Debug log saved to file ({reason})")
            
        except Exception as e:
            print(f"Failed to save debug log: {e}")
    
    def save_debug_log_manual(self):
        """Manual save triggered by user"""
        self.save_debug_log("manual_save")
        self.log_message("💾 Debug log saved manually")
        messagebox.showinfo("Debug Log", f"Debug log saved successfully!\n\nFile: {os.path.basename(self.log_file_path)}\nLocation: Data Logs folder")
    
    def start_periodic_logging(self):
        """Start periodic log saves"""
        def periodic_save():
            # Save log every 3 minutes
            self.save_debug_log("periodic_auto_save")
            # Schedule next save
            self.root.after(180000, periodic_save)  # 3 minutes
        
        # Start periodic saves after 3 minutes
        self.root.after(180000, periodic_save)
        
    def setup_serial(self):
        """Setup serial connection to rover"""
        # Platform-specific port names
        if os.name == 'nt':  # Windows
            ports_to_try = ['COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8', 'COM9', 'COM10']
        else:  # Linux/Unix
            ports_to_try = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1']
        
        for port in ports_to_try:
            try:
                self.log_debug("CONNECTION", f"Attempting serial connection on {port}")
                self.serial_conn = serial.Serial(port, 115200, timeout=1)
                time.sleep(2)  # Wait for Arduino
                self.robot_state['connection_status'] = 'CONNECTED'
                self.log_message(f"✅ Connected to rover on {port}")
                self.log_debug("CONNECTION", f"Serial connection established", {
                    "port": port,
                    "baudrate": 115200,
                    "timeout": 1
                })
                
                # Start serial reading thread
                self.running = True
                self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
                self.serial_thread.start()
                break
                
            except Exception as e:
                self.log_message(f"❌ Failed to connect on {port}: {e}")
                self.log_debug("ERROR", f"Serial connection failed on {port}", {"error": str(e)})
                continue
        
        if not self.serial_conn:
            self.log_message("🔄 No rover found. Will keep trying...")
            self.log_debug("CONNECTION", "No serial ports available, will retry")
            self.root.after(5000, self.setup_serial)  # Retry in 5 seconds
    
    def read_serial_data(self):
        """Read data from rover in background thread"""
        while self.running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode().strip()
                    if line:
                        # Log all incoming serial data
                        self.log_debug("SERIAL_RX", "Data received from rover", {"data": line})
                        self.data_queue.put(('serial', line))
            except Exception as e:
                error_msg = f"Serial error: {e}"
                self.log_debug("ERROR", "Serial communication error", {"error": str(e)})
                self.data_queue.put(('error', error_msg))
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
                
                # Pi acknowledgments (from gatekeeper)
                elif 'pi_ack' in data:
                    ack = data['pi_ack']
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
                
                # Gatekeeper data (new format)
                elif 'gatekeeper_mode' in data:
                    # Log mode changes
                    old_mode = self.robot_state.get('mode', 'UNKNOWN')
                    new_mode = data.get('gatekeeper_mode', 'UNKNOWN')
                    if old_mode != new_mode:
                        self.log_debug("MODE_CHANGE", f"Robot mode changed: {old_mode} → {new_mode}")
                    
                    self.robot_state['mode'] = new_mode
                    self.robot_state['rc_valid'] = data.get('valid', False)
                    if 'distance' in data and data['distance'] is not None:
                        self.robot_state['ultrasonic_distance'] = data['distance']
                    
                    # Log important state data periodically
                    current_time = time.time()
                    if not hasattr(self, '_last_state_log') or current_time - self._last_state_log > 5:
                        self.log_debug("STATE", "Robot state update", {
                            "mode": new_mode,
                            "rc_valid": data.get('valid', False),
                            "distance": data.get('distance'),
                            "emergency": data.get('emergency', False)
                        })
                        self._last_state_log = current_time
                    
                    # Process Pi telemetry data if present
                    if 'pi_telemetry' in data:
                        self.process_pi_telemetry(data['pi_telemetry'])
                        
            else:
                # Non-JSON messages
                if "gatekeeper" in message.lower() or "initialized" in message.lower():
                    self.log_message(f"🔧 {message}")
                    
        except json.JSONDecodeError:
            # Log non-JSON system messages
            if any(keyword in message.lower() for keyword in ['arduino', 'initialized', 'gatekeeper', 'safety']):
                self.log_message(f"📝 {message}")
    
    def process_pi_telemetry(self, telemetry_data):
        """Process Pi telemetry data"""
        try:
            # Log telemetry reception
            self.log_debug("TELEMETRY", "Pi telemetry data received", {
                "nav_state": telemetry_data.get('nav_state'),
                "lidar_points": telemetry_data.get('lidar_points'),
                "lidar_min_dist": telemetry_data.get('lidar_min_dist'),
                "system_cpu": telemetry_data.get('system', {}).get('cpu'),
                "system_memory": telemetry_data.get('system', {}).get('memory'),
                "system_temp": telemetry_data.get('system', {}).get('temp')
            })
            
            # Navigation state
            if 'nav_state' in telemetry_data:
                old_nav_state = self.robot_state.get('nav_state', 'UNKNOWN')
                new_nav_state = telemetry_data['nav_state']
                if old_nav_state != new_nav_state:
                    self.log_debug("NAV_CHANGE", f"Navigation state changed: {old_nav_state} → {new_nav_state}")
                self.robot_state['nav_state'] = new_nav_state
            
            # LiDAR data
            if 'lidar_points' in telemetry_data:
                self.robot_state['lidar_points'] = telemetry_data['lidar_points']
            if 'lidar_min_dist' in telemetry_data:
                self.robot_state['lidar_min_dist'] = telemetry_data['lidar_min_dist']
            if 'lidar_min_angle' in telemetry_data:
                self.robot_state['lidar_min_angle'] = telemetry_data['lidar_min_angle']
            if 'lidar_quality' in telemetry_data:
                self.robot_state['lidar_quality'] = telemetry_data['lidar_quality']
            
            # LiDAR sectors
            if 'sectors' in telemetry_data:
                self.robot_state['lidar_sectors'] = telemetry_data['sectors']
            
            # System telemetry
            if 'system' in telemetry_data:
                system_data = telemetry_data['system']
                if 'cpu' in system_data:
                    self.robot_state['pi_cpu'] = system_data['cpu']
                if 'memory' in system_data:
                    self.robot_state['pi_memory'] = system_data['memory']
                if 'temp' in system_data and system_data['temp'] is not None:
                    self.robot_state['pi_temperature'] = system_data['temp']
                if 'uptime' in system_data:
                    self.robot_state['pi_uptime'] = system_data['uptime']
                if 'disk' in system_data:
                    self.robot_state['pi_disk'] = system_data['disk']
            
            # Log telemetry update
            self.log_message(f"📡 Pi telemetry updated: nav={self.robot_state.get('nav_state', 'N/A')}, "
                           f"LiDAR={self.robot_state.get('lidar_points', 0)}pts")
                           
        except Exception as e:
            self.log_debug("ERROR", "Pi telemetry processing failed", {"error": str(e)})
            self.log_message(f"❌ Error processing Pi telemetry: {e}")
    
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
        left_pct = int((self.robot_state['motor_left'] / 255) * 100) if self.robot_state['motor_left'] else 0
        right_pct = int((self.robot_state['motor_right'] / 255) * 100) if self.robot_state['motor_right'] else 0
        self.left_motor_value.config(text=f"{left_pct:+d}%")
        self.right_motor_value.config(text=f"{right_pct:+d}%")
        
        # Ultrasonic display
        self.draw_ultrasonic_display()
        
        # Roaming status display
        self.update_roaming_display()
        
        # System info
        if self.robot_state['battery_voltage']:
            self.battery_label.config(text=f"{self.robot_state['battery_voltage']:.1f}V")
        
        # Pi telemetry updates
        self.update_pi_telemetry_displays()
        
        # Schedule next update
        self.root.after(100, self.update_displays)
    
    def draw_motor_gauge(self, canvas, speed):
        """Draw circular motor speed gauge"""
        canvas.delete("all")
        
        # Gauge background
        canvas.create_oval(10, 10, 110, 110, outline='#404040', width=3)
        
        # Speed percentage
        speed_pct = (speed / 255) * 100 if speed else 0
        
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
    
    def update_pi_telemetry_displays(self):
        """Update Pi telemetry displays"""
        # LiDAR data
        lidar_points = self.robot_state.get('lidar_points', 0)
        self.lidar_points_label.config(text=str(lidar_points))
        
        lidar_min_dist = self.robot_state.get('lidar_min_dist')
        if lidar_min_dist is not None:
            # Color code based on distance
            if lidar_min_dist < 30:
                color = '#ff0000'  # Red - danger
            elif lidar_min_dist < 100:
                color = '#ff8000'  # Orange - caution
            else:
                color = '#00ff00'  # Green - safe
            self.lidar_min_dist_label.config(text=f"{lidar_min_dist:.1f}cm", fg=color)
        else:
            self.lidar_min_dist_label.config(text="N/A", fg='#808080')
        
        lidar_quality = self.robot_state.get('lidar_quality', 0)
        self.lidar_quality_label.config(text=f"{lidar_quality:.1f}")
        
        # Navigation state
        nav_state = self.robot_state.get('nav_state', 'UNKNOWN')
        nav_colors = {
            'STRAIGHT': '#00ff00',
            'AVOIDING': '#ff8000', 
            'EXPLORING': '#0080ff',
            'TURNING': '#ffff00',
            'INITIALIZING': '#808080',
            'UNKNOWN': '#808080'
        }
        color = nav_colors.get(nav_state, '#808080')
        self.nav_state_label.config(text=nav_state, fg=color)
        
        # LiDAR sectors visualization
        self.draw_lidar_sectors()
        
        # Pi system data
        pi_cpu = self.robot_state.get('pi_cpu', 0)
        cpu_color = '#00ff00' if pi_cpu < 70 else '#ff8000' if pi_cpu < 90 else '#ff0000'
        self.pi_cpu_label.config(text=f"{pi_cpu:.1f}%", fg=cpu_color)
        
        pi_memory = self.robot_state.get('pi_memory', 0)
        mem_color = '#00ff00' if pi_memory < 70 else '#ff8000' if pi_memory < 90 else '#ff0000'
        self.pi_memory_label.config(text=f"{pi_memory:.1f}%", fg=mem_color)
        
        pi_temp = self.robot_state.get('pi_temperature')
        if pi_temp is not None:
            temp_color = '#00ff00' if pi_temp < 70 else '#ff8000' if pi_temp < 80 else '#ff0000'
            self.pi_temp_label.config(text=f"{pi_temp:.1f}°C", fg=temp_color)
        else:
            self.pi_temp_label.config(text="N/A", fg='#808080')
    
    def draw_lidar_sectors(self):
        """Draw LiDAR sector visualization"""
        self.sectors_canvas.delete("all")
        
        sectors = self.robot_state.get('lidar_sectors', {'front': True, 'left': True, 'right': True})
        
        # Draw rover (center rectangle)
        rover_x, rover_y = 60, 30
        rover_w, rover_h = 20, 15
        self.sectors_canvas.create_rectangle(rover_x - rover_w//2, rover_y - rover_h//2,
                                           rover_x + rover_w//2, rover_y + rover_h//2,
                                           fill='#0080ff', outline='#00ff00')
        
        # Front sector (rectangle above rover)
        front_color = '#00ff00' if sectors.get('front', True) else '#ff0000'
        self.sectors_canvas.create_rectangle(rover_x - 15, rover_y - 25,
                                           rover_x + 15, rover_y - rover_h//2,
                                           fill=front_color, outline=front_color)
        
        # Left sector (rectangle to left of rover)
        left_color = '#00ff00' if sectors.get('left', True) else '#ff0000'
        self.sectors_canvas.create_rectangle(rover_x - 35, rover_y - 10,
                                           rover_x - rover_w//2, rover_y + 10,
                                           fill=left_color, outline=left_color)
        
        # Right sector (rectangle to right of rover)
        right_color = '#00ff00' if sectors.get('right', True) else '#ff0000'
        self.sectors_canvas.create_rectangle(rover_x + rover_w//2, rover_y - 10,
                                           rover_x + 35, rover_y + 10,
                                           fill=right_color, outline=right_color)
        
        # Labels
        self.sectors_canvas.create_text(rover_x, rover_y - 18, text="F", 
                                       fill='white', font=('Arial', 8, 'bold'))
        self.sectors_canvas.create_text(rover_x - 25, rover_y, text="L", 
                                       fill='white', font=('Arial', 8, 'bold'))
        self.sectors_canvas.create_text(rover_x + 25, rover_y, text="R", 
                                       fill='white', font=('Arial', 8, 'bold'))
    
    # SSH Control Methods
    def connect_ssh(self):
        """Connect to Pi via SSH with password prompt"""
        self.log_debug("CONNECTION", f"SSH connection attempt initiated to {self.ssh_config['host']}")
        
        # First, prompt for password
        password = self.prompt_ssh_password()
        if not password:
            self.log_debug("CONNECTION", "SSH connection cancelled by user")
            return  # User cancelled
            
        def ssh_connect_thread():
            try:
                self.ssh_client = paramiko.SSHClient()
                self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                
                # Update status to show authentication attempt
                self.root.after(0, lambda: self.update_ssh_status("AUTHENTICATING...", "#ffff00"))
                self.root.after(0, lambda: self.log_message(f"🔐 Authenticating with {self.ssh_config['host']}..."))
                
                # Try to connect with password
                self.ssh_client.connect(
                    hostname=self.ssh_config['host'],
                    username=self.ssh_config['username'],
                    password=password,
                    timeout=10
                )
                
                self.ssh_connected = True
                self.root.after(0, lambda: self.update_ssh_status("CONNECTED", "#00ff00"))
                self.root.after(0, lambda: self.log_message(f"✅ SSH connection successful to {self.ssh_config['host']}"))
                self.log_debug("CONNECTION", "SSH connection established successfully", {
                    "host": self.ssh_config['host'],
                    "username": self.ssh_config['username'],
                    "timeout": 10
                })
                
            except paramiko.AuthenticationException:
                self.ssh_connected = False
                self.root.after(0, lambda: self.update_ssh_status("AUTH FAILED", "#ff0000"))
                self.root.after(0, lambda: self.log_message("❌ SSH authentication failed - incorrect password"))
                self.log_debug("ERROR", "SSH authentication failed", {"reason": "incorrect_password"})
            except paramiko.SSHException as e:
                self.ssh_connected = False
                self.root.after(0, lambda: self.update_ssh_status("SSH ERROR", "#ff0000"))
                self.root.after(0, lambda: self.log_message(f"❌ SSH error: {e}"))
                self.log_debug("ERROR", "SSH connection error", {"error": str(e)})
            except Exception as e:
                self.ssh_connected = False
                self.root.after(0, lambda: self.update_ssh_status("FAILED", "#ff0000"))
                self.root.after(0, lambda: self.log_message(f"❌ Connection failed: {e}"))
                self.log_debug("ERROR", "SSH connection failed", {"error": str(e)})
        
        # Update UI immediately
        self.update_ssh_status("CONNECTING...", "#ffff00")
        self.ssh_connect_btn.config(state='disabled')
        self.log_message(f"🔌 Connecting to {self.ssh_config['host']}...")
        
        # Connect in background thread
        thread = threading.Thread(target=ssh_connect_thread, daemon=True)
        thread.start()
    
    def prompt_ssh_password(self):
        """Prompt user for SSH password"""
        dialog = tk.Toplevel(self.root)
        dialog.title("SSH Authentication")
        dialog.geometry("400x200")
        dialog.configure(bg='#2a2a2a')
        dialog.transient(self.root)
        dialog.grab_set()
        
        # Center the dialog
        dialog.geometry("+%d+%d" % (
            self.root.winfo_rootx() + 50,
            self.root.winfo_rooty() + 50
        ))
        
        # Password result
        result = {"password": None}
        
        # Main frame
        main_frame = tk.Frame(dialog, bg='#2a2a2a')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # Title
        title_label = tk.Label(main_frame, text="SSH Password Required", 
                              font=('Arial', 14, 'bold'), fg='#00ff00', bg='#2a2a2a')
        title_label.pack(pady=(0, 10))
        
        # Info
        info_label = tk.Label(main_frame, 
                             text=f"Enter password for {self.ssh_config['username']}@{self.ssh_config['host']}", 
                             font=('Arial', 10), fg='white', bg='#2a2a2a')
        info_label.pack(pady=(0, 15))
        
        # Password entry
        password_frame = tk.Frame(main_frame, bg='#2a2a2a')
        password_frame.pack(fill=tk.X, pady=(0, 15))
        
        tk.Label(password_frame, text="Password:", font=('Arial', 10), 
                fg='white', bg='#2a2a2a').pack(anchor='w')
        
        password_entry = tk.Entry(password_frame, show='*', font=('Arial', 12), 
                                 bg='#3a3a3a', fg='white', insertbackground='white')
        password_entry.pack(fill=tk.X, pady=(5, 0))
        password_entry.focus_set()
        
        # Buttons
        button_frame = tk.Frame(main_frame, bg='#2a2a2a')
        button_frame.pack(fill=tk.X)
        
        def on_connect():
            result["password"] = password_entry.get()
            dialog.destroy()
        
        def on_cancel():
            dialog.destroy()
        
        # Bind Enter key to connect
        password_entry.bind('<Return>', lambda e: on_connect())
        
        connect_btn = tk.Button(button_frame, text="Connect", command=on_connect,
                               bg='#0080ff', fg='white', font=('Arial', 10, 'bold'),
                               relief=tk.RAISED, bd=2)
        connect_btn.pack(side=tk.RIGHT, padx=(5, 0))
        
        cancel_btn = tk.Button(button_frame, text="Cancel", command=on_cancel,
                              bg='#ff4040', fg='white', font=('Arial', 10),
                              relief=tk.RAISED, bd=2)
        cancel_btn.pack(side=tk.RIGHT)
        
        # Wait for dialog to close
        dialog.wait_window()
        
        return result["password"]
    
    def update_ssh_status(self, status, color):
        """Update SSH status display"""
        self.ssh_status_label.config(text=status, fg=color)
        
        if status == "CONNECTED":
            self.ssh_connect_btn.config(text="Disconnect SSH", command=self.disconnect_ssh, state='normal')
            self.launch_btn.config(state='normal')
            for btn in [self.reboot_btn, self.git_pull_btn, self.list_processes_btn, self.upload_file_btn]:
                btn.config(state='normal')
            # Enable terminal controls
            self.send_cmd_btn.config(state='normal')
            self.command_entry.config(state='normal')
            if hasattr(self, 'quick_cmd_buttons'):
                for btn in self.quick_cmd_buttons:
                    btn.config(state='normal')
            self.add_terminal_output("✅ SSH connected - Terminal ready for commands")
        else:
            self.ssh_connect_btn.config(text="Connect SSH", command=self.connect_ssh, state='normal')
            self.launch_btn.config(state='disabled')
            self.stop_btn.config(state='disabled')
            for btn in [self.reboot_btn, self.git_pull_btn, self.list_processes_btn, self.upload_file_btn]:
                btn.config(state='disabled')
            # Disable terminal controls
            self.send_cmd_btn.config(state='disabled')
            self.command_entry.config(state='disabled')
            if hasattr(self, 'quick_cmd_buttons'):
                for btn in self.quick_cmd_buttons:
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
    
    def upload_python_file(self):
        """Upload a Python file from local computer to Pi"""
        if not self.ssh_connected:
            messagebox.showerror("Error", "SSH not connected!")
            return
        
        # File selection dialog
        file_path = filedialog.askopenfilename(
            title="Select Python File to Upload",
            filetypes=[
                ("Python files", "*.py"),
                ("All files", "*.*")
            ],
            initialdir=os.path.expanduser("~")
        )
        
        if not file_path:
            return  # User cancelled
        
        filename = os.path.basename(file_path)
        
        # Validate it's a Python file
        if not filename.endswith('.py'):
            messagebox.showerror("Error", "Please select a Python (.py) file!")
            return
        
        # Read local file content
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                file_content = f.read()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to read local file: {e}")
            return
        
        # Check if file exists on Pi and handle overwrite
        remote_path = f"{self.ssh_config['work_dir']}/{filename}"
        
        def upload_thread():
            try:
                # Check if file exists on Pi
                stdin, stdout, stderr = self.ssh_client.exec_command(f"test -f {remote_path} && echo 'exists' || echo 'new'")
                result = stdout.read().decode().strip()
                
                if result == 'exists':
                    # File exists - ask user about overwrite
                    self.root.after(0, lambda: self.handle_file_overwrite(filename, file_content, remote_path))
                else:
                    # New file - proceed with upload
                    self.root.after(0, lambda: self.perform_file_upload(filename, file_content, remote_path))
                    
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"❌ Upload check failed: {e}"))
        
        self.log_message(f"📤 Checking upload target: {filename}")
        thread = threading.Thread(target=upload_thread, daemon=True)
        thread.start()
    
    def handle_file_overwrite(self, filename, file_content, remote_path):
        """Handle file overwrite confirmation"""
        result = messagebox.askyesno(
            "File Exists", 
            f"The file '{filename}' already exists on the Pi.\n\n"
            f"Do you want to overwrite it with the local version?\n\n"
            f"This will completely replace the existing file's contents.",
            icon='warning'
        )
        
        if result:
            self.perform_file_upload(filename, file_content, remote_path, overwrite=True)
        else:
            self.log_message(f"📤 Upload cancelled: {filename}")
    
    def perform_file_upload(self, filename, file_content, remote_path, overwrite=False):
        """Perform the actual file upload"""
        def upload_thread():
            try:
                # Create SFTP client
                sftp = self.ssh_client.open_sftp()
                
                # Write file content to Pi
                with sftp.open(remote_path, 'w') as remote_file:
                    remote_file.write(file_content)
                
                # Make file executable if it's a Python script
                sftp.chmod(remote_path, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | 
                          stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
                
                sftp.close()
                
                # Update program dropdown if it's a new file
                if not overwrite:
                    self.root.after(0, lambda: self.update_program_dropdown(filename))
                
                action = "overwritten" if overwrite else "uploaded"
                self.root.after(0, lambda: self.log_message(f"✅ File {action} successfully: {filename}"))
                self.root.after(0, lambda: self.log_message(f"📍 Location: {remote_path}"))
                
                # Refresh program list
                self.root.after(0, self.refresh_program_list)
                
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"❌ Upload failed: {e}"))
        
        action = "Overwriting" if overwrite else "Uploading"
        self.log_message(f"📤 {action} {filename}...")
        thread = threading.Thread(target=upload_thread, daemon=True)
        thread.start()
    
    def update_program_dropdown(self, filename):
        """Add new file to program dropdown if not already present"""
        current_values = list(self.program_combo['values'])
        if filename not in current_values:
            current_values.insert(0, filename)  # Add at top
            self.program_combo['values'] = current_values
            self.log_message(f"📋 Added {filename} to program list")
    
    def refresh_program_list(self):
        """Refresh the program dropdown with files from Pi"""
        if not self.ssh_connected:
            return
            
        def refresh_thread():
            try:
                # List Python files in work directory
                command = f"cd {self.ssh_config['work_dir']} && ls -1 *.py 2>/dev/null || echo 'no_py_files'"
                stdin, stdout, stderr = self.ssh_client.exec_command(command)
                result = stdout.read().decode().strip()
                
                if result != 'no_py_files':
                    py_files = [f.strip() for f in result.split('\n') if f.strip().endswith('.py')]
                    
                    # Update dropdown with current files
                    base_programs = [
                        'autonomous_lidar_telemetry.py',
                        'autonomous_always_thinking.py',
                        'autonomous_controller_smooth.py', 
                        'autonomous_controller_with_sensors.py',
                        'autonomous_controller_lidar.py',
                        'autonomous_test_simple.py',
                        'rover_dashboard.py'
                    ]
                    
                    # Combine uploaded files with base programs, removing duplicates
                    all_programs = py_files.copy()
                    for base_prog in base_programs:
                        if base_prog not in all_programs:
                            all_programs.append(base_prog)
                    
                    self.root.after(0, lambda: self.update_dropdown_list(all_programs))
                    self.root.after(0, lambda: self.log_message(f"📋 Program list refreshed: {len(py_files)} files found"))
                    
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"⚠️ Program list refresh failed: {e}"))
        
        thread = threading.Thread(target=refresh_thread, daemon=True)
        thread.start()
    
    def update_dropdown_list(self, programs):
        """Update the program dropdown list"""
        current_selection = self.program_var.get()
        self.program_combo['values'] = programs
        
        # Restore selection if it still exists
        if current_selection in programs:
            self.program_combo.set(current_selection)
        elif programs:
            self.program_combo.set(programs[0])
    
    # Terminal Interface Methods
    def add_terminal_output(self, text):
        """Add text to terminal output"""
        self.terminal_output.config(state=tk.NORMAL)
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.terminal_output.insert(tk.END, f"[{timestamp}] {text}\n")
        self.terminal_output.see(tk.END)
        self.terminal_output.config(state=tk.DISABLED)
    
    def insert_command(self, command):
        """Insert a command into the command entry field"""
        self.command_entry.delete(0, tk.END)
        self.command_entry.insert(0, command)
        self.command_entry.focus_set()
        
        # If it's a partial command, position cursor at the end
        if command.endswith(' '):
            self.command_entry.icursor(tk.END)
    
    def execute_command(self, event=None):
        """Execute command on Pi via SSH"""
        if not self.ssh_connected:
            self.add_terminal_output("❌ SSH not connected!")
            return
        
        command = self.command_entry.get().strip()
        if not command:
            return
        
        # Log command execution
        self.log_debug("COMMAND", f"Terminal command executed: {command}")
        
        # Add to command history
        if command not in self.command_history:
            self.command_history.append(command)
        self.history_index = len(self.command_history)
        
        # Clear command entry
        self.command_entry.delete(0, tk.END)
        
        # Show command being executed
        self.add_terminal_output(f"$ {command}")
        
        def execute_thread():
            try:
                # Determine if we need to activate virtual environment
                needs_venv = any(cmd in command for cmd in ['pip', 'python', 'python3'])
                
                if needs_venv and not command.startswith('source'):
                    # Prepend virtual environment activation for pip/python commands
                    full_command = f"cd {self.ssh_config['work_dir']} && {self.ssh_config['venv_activate']} && {command}"
                else:
                    # Run command as-is
                    full_command = command
                
                self.log_debug("SSH_TX", f"SSH command sent", {
                    "original_command": command,
                    "full_command": full_command,
                    "needs_venv": needs_venv
                })
                
                # Execute command
                stdin, stdout, stderr = self.ssh_client.exec_command(full_command, timeout=30)
                
                # Read output
                output = stdout.read().decode('utf-8', errors='replace').strip()
                error = stderr.read().decode('utf-8', errors='replace').strip()
                
                # Log command response
                self.log_debug("SSH_RX", f"SSH command response", {
                    "command": command,
                    "output": output[:200] if output else None,  # Truncate long output
                    "error": error[:200] if error else None,
                    "success": not error
                })
                
                # Display results
                if output:
                    self.root.after(0, lambda: self.add_terminal_output(output))
                if error:
                    self.root.after(0, lambda: self.add_terminal_output(f"⚠️ {error}"))
                
                if not output and not error:
                    self.root.after(0, lambda: self.add_terminal_output("✅ Command completed (no output)"))
                    
            except Exception as e:
                self.log_debug("ERROR", f"SSH command execution failed", {
                    "command": command,
                    "error": str(e)
                })
                self.root.after(0, lambda: self.add_terminal_output(f"❌ Command failed: {e}"))
        
        thread = threading.Thread(target=execute_thread, daemon=True)
        thread.start()
    
    def command_history_up(self, event):
        """Navigate up in command history"""
        if self.command_history and self.history_index > 0:
            self.history_index -= 1
            self.command_entry.delete(0, tk.END)
            self.command_entry.insert(0, self.command_history[self.history_index])
    
    def command_history_down(self, event):
        """Navigate down in command history"""
        if self.command_history:
            if self.history_index < len(self.command_history) - 1:
                self.history_index += 1
                self.command_entry.delete(0, tk.END)
                self.command_entry.insert(0, self.command_history[self.history_index])
            else:
                # Clear entry when going past the end
                self.history_index = len(self.command_history)
                self.command_entry.delete(0, tk.END)
    
    def run(self):
        """Start the dashboard"""
        try:
            self.log_debug("SYSTEM", "Dashboard GUI started")
            self.root.mainloop()
        except KeyboardInterrupt:
            self.log_debug("SYSTEM", "Dashboard shutdown requested by user (Ctrl+C)")
        except Exception as e:
            self.log_debug("ERROR", f"Dashboard error during runtime", {"error": str(e)})
        finally:
            self.log_debug("SYSTEM", "Dashboard shutdown initiated")
            self.running = False
            if self.serial_conn:
                self.log_debug("CONNECTION", "Closing serial connection")
                self.serial_conn.close()
            if self.ssh_client:
                self.log_debug("CONNECTION", "Closing SSH connection")
                self.ssh_client.close()
            
            # Save final debug log
            self.save_debug_log("session_end")
            print(f"Debug log saved: {self.log_file_path}")

def main():
    print("🚀 Starting Rover Control Dashboard...")
    dashboard = RoverControlDashboard()
    dashboard.run()

if __name__ == "__main__":
    main()