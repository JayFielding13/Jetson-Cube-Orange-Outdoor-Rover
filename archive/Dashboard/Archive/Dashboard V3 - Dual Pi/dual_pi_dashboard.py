#!/usr/bin/env python3
"""
Dual-Pi Rover Dashboard V3
================================================================================
Enhanced dashboard for managing dual-Pi rover setup with:
- Navigation Pi: Handles autonomous navigation, Bluetooth tracking, motor control
- Visualization Pi: Handles telemetry relay, data visualization, base station communication

Features:
- Dual SSH connection management
- Real-time telemetry monitoring from Pi-to-Pi communication
- Tabbed interface for each Pi system
- System overview with live data visualization
- File upload targeting specific Pi systems
- Integrated terminal for both Pi systems

Part of the Mini Rover Development Project
Author: Developed with Jay Fielding  
Version: 3.0 - Dual Pi Management
Date: 2025-01-22
================================================================================
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog, scrolledtext
import paramiko
from scp import SCPClient
import threading
import os
import time
import json
from datetime import datetime
from typing import Dict, Any, Optional
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

# Import our Pi communication module
from pi_communication import VisualizationDataReceiver

class DualPiDashboard:
    """
    Enhanced dashboard for dual-Pi rover management
    
    Provides independent SSH connections to both Navigation Pi and Visualization Pi,
    plus real-time telemetry monitoring through the Pi-to-Pi communication system.
    """
    
    def __init__(self, root):
        self.root = root
        self.root.title("Dual-Pi Rover Dashboard V3")
        self.root.geometry("1400x800")
        
        # Pi connection configurations
        self.pi_configs = {
            'navigation': {
                'name': 'Navigation Pi',
                'default_host': '192.168.1.10',
                'default_path': '/home/jay/rover_project/',
                'ssh': None,
                'scp': None,
                'connected': False,
                'last_file': None
            },
            'visualization': {
                'name': 'Visualization Pi', 
                'default_host': '192.168.1.11',
                'default_path': '/home/jay/rover_project/',
                'ssh': None,
                'scp': None,
                'connected': False,
                'last_file': None
            }
        }
        
        # Telemetry system
        self.telemetry_receiver = None
        self.telemetry_running = False
        self.telemetry_data = []
        self.max_telemetry_points = 100
        
        # Theme management
        self.dark_mode = False
        self.setup_themes()
        
        # Setup GUI
        self.setup_gui()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.exit_application)
        
        print("🤖 Dual-Pi Rover Dashboard V3 initialized")
    
    def setup_themes(self):
        """Setup light and dark theme configurations"""
        self.light_theme = {
            'bg': '#ffffff',
            'fg': '#000000',
            'select_bg': '#0078d4',
            'select_fg': '#ffffff',
            'entry_bg': '#ffffff',
            'entry_fg': '#000000',
            'button_bg': '#f0f0f0',
            'button_fg': '#000000',
            'frame_bg': '#f5f5f5',
            'terminal_bg': '#ffffff',
            'terminal_fg': '#000000'
        }
        
        self.dark_theme = {
            'bg': '#2d2d2d',
            'fg': '#ffffff',
            'select_bg': '#404040',
            'select_fg': '#ffffff',
            'entry_bg': '#404040',
            'entry_fg': '#ffffff',
            'button_bg': '#505050',
            'button_fg': '#ffffff',
            'frame_bg': '#353535',
            'terminal_bg': '#1e1e1e',
            'terminal_fg': '#ffffff'
        }
        
        self.apply_theme()
    
    def apply_theme(self):
        """Apply current theme to all widgets"""
        theme = self.dark_theme if self.dark_mode else self.light_theme
        
        self.root.configure(bg=theme['bg'])
        
        style = ttk.Style()
        style.configure('TFrame', background=theme['bg'])
        style.configure('TLabelFrame', background=theme['bg'], foreground=theme['fg'])
        style.configure('TLabelFrame.Label', background=theme['bg'], foreground=theme['fg'])
        style.configure('TLabel', background=theme['bg'], foreground=theme['fg'])
        style.configure('TEntry', fieldbackground=theme['entry_bg'], foreground=theme['entry_fg'])
        style.configure('TButton', background=theme['button_bg'], foreground=theme['button_fg'])
        style.map('TButton', background=[('active', theme['select_bg'])])
        
        # Configure notebook tabs
        style.configure('TNotebook', background=theme['bg'])
        style.configure('TNotebook.Tab', background=theme['button_bg'], foreground=theme['button_fg'])
        style.map('TNotebook.Tab', background=[('selected', theme['select_bg'])])
    
    def toggle_theme(self):
        """Toggle between light and dark themes"""
        self.dark_mode = not self.dark_mode
        self.apply_theme()
        
        # Update terminals
        theme = self.dark_theme if self.dark_mode else self.light_theme
        for pi_type in ['navigation', 'visualization']:
            terminal_name = f'{pi_type}_terminal'
            if hasattr(self, terminal_name):
                terminal = getattr(self, terminal_name)
                terminal.configure(
                    bg=theme['terminal_bg'],
                    fg=theme['terminal_fg'],
                    selectbackground=theme['select_bg'],
                    selectforeground=theme['select_fg'],
                    insertbackground=theme['fg']
                )
        
        # Update overview terminal
        if hasattr(self, 'overview_terminal'):
            self.overview_terminal.configure(
                bg=theme['terminal_bg'],
                fg=theme['terminal_fg'],
                selectbackground=theme['select_bg'],
                selectforeground=theme['select_fg'],
                insertbackground=theme['fg']
            )
        
        # Update theme button
        if hasattr(self, 'theme_btn'):
            self.theme_btn.configure(text="🌙 Dark" if not self.dark_mode else "☀️ Light")
    
    def setup_gui(self):
        """Setup the main GUI with tabbed interface"""
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # Top control panel
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        control_frame.columnconfigure(1, weight=1)
        
        # Global controls
        ttk.Label(control_frame, text="🤖 Dual-Pi Rover Dashboard V3", 
                 font=('Arial', 14, 'bold')).grid(row=0, column=0, sticky=tk.W)
        
        # Theme toggle
        self.theme_btn = ttk.Button(control_frame, text="🌙 Dark", command=self.toggle_theme)
        self.theme_btn.grid(row=0, column=2, padx=(10, 0))
        
        # Start/Stop telemetry
        self.telemetry_btn = ttk.Button(control_frame, text="📡 Start Telemetry", 
                                       command=self.toggle_telemetry)
        self.telemetry_btn.grid(row=0, column=3, padx=(10, 0))
        
        # Emergency stop all
        self.emergency_btn = ttk.Button(control_frame, text="🛑 Emergency Stop All", 
                                       command=self.emergency_stop_all,
                                       style='Emergency.TButton')
        self.emergency_btn.grid(row=0, column=4, padx=(10, 0))
        
        # Create notebook for tabs
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Create tabs
        self.setup_navigation_pi_tab()
        self.setup_visualization_pi_tab() 
        self.setup_system_overview_tab()
        
        print("✅ GUI setup complete")
    
    def setup_navigation_pi_tab(self):
        """Setup the Navigation Pi management tab"""
        # Navigation Pi frame
        nav_frame = ttk.Frame(self.notebook)
        self.notebook.add(nav_frame, text="🧭 Navigation Pi")
        
        nav_frame.columnconfigure(1, weight=1)
        nav_frame.rowconfigure(2, weight=1)
        
        # Connection section
        conn_frame = ttk.LabelFrame(nav_frame, text="Navigation Pi Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.setup_pi_connection_controls(conn_frame, 'navigation')
        
        # File operations
        file_frame = ttk.LabelFrame(nav_frame, text="Navigation Code Management", padding="5")
        file_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        file_frame.columnconfigure(0, weight=1)
        
        self.setup_pi_file_controls(file_frame, 'navigation')
        
        # Terminal
        terminal_frame = ttk.LabelFrame(nav_frame, text="Navigation Pi Terminal", padding="5")
        terminal_frame.grid(row=1, column=1, rowspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(10, 0))
        terminal_frame.columnconfigure(0, weight=1)
        terminal_frame.rowconfigure(0, weight=1)
        
        self.setup_pi_terminal(terminal_frame, 'navigation')
    
    def setup_visualization_pi_tab(self):
        """Setup the Visualization Pi management tab"""
        # Visualization Pi frame
        viz_frame = ttk.Frame(self.notebook)
        self.notebook.add(viz_frame, text="📊 Visualization Pi")
        
        viz_frame.columnconfigure(1, weight=1)
        viz_frame.rowconfigure(2, weight=1)
        
        # Connection section
        conn_frame = ttk.LabelFrame(viz_frame, text="Visualization Pi Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.setup_pi_connection_controls(conn_frame, 'visualization')
        
        # File operations
        file_frame = ttk.LabelFrame(viz_frame, text="Visualization Code Management", padding="5")
        file_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        file_frame.columnconfigure(0, weight=1)
        
        self.setup_pi_file_controls(file_frame, 'visualization')
        
        # Terminal
        terminal_frame = ttk.LabelFrame(viz_frame, text="Visualization Pi Terminal", padding="5")
        terminal_frame.grid(row=1, column=1, rowspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(10, 0))
        terminal_frame.columnconfigure(0, weight=1)
        terminal_frame.rowconfigure(0, weight=1)
        
        self.setup_pi_terminal(terminal_frame, 'visualization')
    
    def setup_system_overview_tab(self):
        """Setup the system overview tab with live telemetry"""
        # Overview frame
        overview_frame = ttk.Frame(self.notebook)
        self.notebook.add(overview_frame, text="📈 System Overview")
        
        overview_frame.columnconfigure(0, weight=1)
        overview_frame.columnconfigure(1, weight=1)
        overview_frame.rowconfigure(1, weight=1)
        
        # Status panel
        status_frame = ttk.LabelFrame(overview_frame, text="System Status", padding="5")
        status_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Connection status
        ttk.Label(status_frame, text="Navigation Pi:").grid(row=0, column=0, sticky=tk.W)
        self.nav_status_var = tk.StringVar(value="Disconnected")
        ttk.Label(status_frame, textvariable=self.nav_status_var).grid(row=0, column=1, sticky=tk.W, padx=(10, 0))
        
        ttk.Label(status_frame, text="Visualization Pi:").grid(row=0, column=2, sticky=tk.W, padx=(20, 0))
        self.viz_status_var = tk.StringVar(value="Disconnected")
        ttk.Label(status_frame, textvariable=self.viz_status_var).grid(row=0, column=3, sticky=tk.W, padx=(10, 0))
        
        ttk.Label(status_frame, text="Telemetry:").grid(row=0, column=4, sticky=tk.W, padx=(20, 0))
        self.telemetry_status_var = tk.StringVar(value="Stopped")
        ttk.Label(status_frame, textvariable=self.telemetry_status_var).grid(row=0, column=5, sticky=tk.W, padx=(10, 0))
        
        # Live data visualization
        viz_frame = ttk.LabelFrame(overview_frame, text="Live Telemetry", padding="5")
        viz_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        viz_frame.columnconfigure(0, weight=1)
        viz_frame.rowconfigure(0, weight=1)
        
        self.setup_telemetry_visualization(viz_frame)
        
        # Telemetry log
        log_frame = ttk.LabelFrame(overview_frame, text="Telemetry Log", padding="5")
        log_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(10, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        self.setup_overview_terminal(log_frame)
    
    def setup_pi_connection_controls(self, parent, pi_type):
        """Setup connection controls for a specific Pi"""
        config = self.pi_configs[pi_type]
        
        # Host
        ttk.Label(parent, text="Host:").grid(row=0, column=0, sticky=tk.W, padx=(0, 5))
        host_var = tk.StringVar(value=config['default_host'])
        setattr(self, f'{pi_type}_host_var', host_var)
        host_entry = ttk.Entry(parent, textvariable=host_var, width=20)
        host_entry.grid(row=0, column=1, padx=(0, 10))
        
        # Port
        ttk.Label(parent, text="Port:").grid(row=0, column=2, sticky=tk.W, padx=(0, 5))
        port_var = tk.StringVar(value="22")
        setattr(self, f'{pi_type}_port_var', port_var)
        port_entry = ttk.Entry(parent, textvariable=port_var, width=8)
        port_entry.grid(row=0, column=3, padx=(0, 10))
        
        # Username
        ttk.Label(parent, text="Username:").grid(row=1, column=0, sticky=tk.W, padx=(0, 5))
        user_var = tk.StringVar(value="jay")
        setattr(self, f'{pi_type}_user_var', user_var)
        user_entry = ttk.Entry(parent, textvariable=user_var, width=20)
        user_entry.grid(row=1, column=1, padx=(0, 10))
        
        # Password
        ttk.Label(parent, text="Password:").grid(row=1, column=2, sticky=tk.W, padx=(0, 5))
        pass_var = tk.StringVar()
        setattr(self, f'{pi_type}_pass_var', pass_var)
        pass_entry = ttk.Entry(parent, textvariable=pass_var, width=20, show="*")
        pass_entry.grid(row=1, column=3, padx=(0, 10))
        
        # Connect button
        connect_btn = ttk.Button(parent, text="Connect", 
                               command=lambda: self.connect_pi(pi_type))
        connect_btn.grid(row=0, column=4, rowspan=2, padx=(10, 0))
        setattr(self, f'{pi_type}_connect_btn', connect_btn)
        
        # Status
        status_var = tk.StringVar(value="Disconnected")
        setattr(self, f'{pi_type}_status_var', status_var)
        status_label = ttk.Label(parent, textvariable=status_var)
        status_label.grid(row=2, column=0, columnspan=5, pady=(5, 0))
    
    def setup_pi_file_controls(self, parent, pi_type):
        """Setup file operation controls for a specific Pi"""
        # Upload button
        upload_btn = ttk.Button(parent, text="Upload File", 
                              command=lambda: self.upload_file(pi_type), state="disabled")
        upload_btn.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        setattr(self, f'{pi_type}_upload_btn', upload_btn)
        
        # Remote path
        ttk.Label(parent, text="Remote Path:").grid(row=1, column=0, sticky=tk.W)
        path_var = tk.StringVar(value=self.pi_configs[pi_type]['default_path'])
        setattr(self, f'{pi_type}_path_var', path_var)
        path_entry = ttk.Entry(parent, textvariable=path_var)
        path_entry.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # Run button
        run_btn = ttk.Button(parent, text="Run Last File", 
                           command=lambda: self.run_file(pi_type), state="disabled")
        run_btn.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        setattr(self, f'{pi_type}_run_btn', run_btn)
        
        # Stop button
        stop_btn = ttk.Button(parent, text="Stop Program", 
                            command=lambda: self.stop_program(pi_type), state="disabled")
        stop_btn.grid(row=4, column=0, sticky=(tk.W, tk.E))
        setattr(self, f'{pi_type}_stop_btn', stop_btn)
    
    def setup_pi_terminal(self, parent, pi_type):
        """Setup terminal for a specific Pi"""
        # Terminal output
        theme = self.dark_theme if self.dark_mode else self.light_theme
        terminal = scrolledtext.ScrolledText(
            parent, height=15, width=50,
            bg=theme['terminal_bg'], fg=theme['terminal_fg'],
            selectbackground=theme['select_bg'], selectforeground=theme['select_fg'],
            insertbackground=theme['fg']
        )
        terminal.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 5))
        setattr(self, f'{pi_type}_terminal', terminal)
        
        # Command entry
        command_var = tk.StringVar()
        setattr(self, f'{pi_type}_command_var', command_var)
        command_entry = ttk.Entry(parent, textvariable=command_var)
        command_entry.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        command_entry.bind('<Return>', lambda e: self.execute_command(pi_type))
        
        # Execute button
        exec_btn = ttk.Button(parent, text="Execute", 
                            command=lambda: self.execute_command(pi_type), state="disabled")
        exec_btn.grid(row=2, column=0, sticky=(tk.W, tk.E))
        setattr(self, f'{pi_type}_exec_btn', exec_btn)
    
    def setup_telemetry_visualization(self, parent):
        """Setup live telemetry visualization"""
        # Create matplotlib figure
        self.telemetry_fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(8, 6))
        self.telemetry_fig.tight_layout()
        
        # Setup plots
        self.ax1.set_title("Bluetooth Distance")
        self.ax1.set_ylabel("Distance (m)")
        self.ax1.grid(True)
        
        self.ax2.set_title("Ultrasonic Distance") 
        self.ax2.set_ylabel("Distance (cm)")
        self.ax2.grid(True)
        
        self.ax3.set_title("RSSI Signal")
        self.ax3.set_ylabel("RSSI (dBm)")
        self.ax3.grid(True)
        
        self.ax4.set_title("Navigation State")
        self.ax4.set_ylabel("State")
        self.ax4.grid(True)
        
        # Embed in tkinter
        self.telemetry_canvas = FigureCanvasTkAgg(self.telemetry_fig, parent)
        self.telemetry_canvas.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Start update timer
        self.update_telemetry_plots()
    
    def setup_overview_terminal(self, parent):
        """Setup overview terminal for telemetry logs"""
        theme = self.dark_theme if self.dark_mode else self.light_theme
        self.overview_terminal = scrolledtext.ScrolledText(
            parent, height=15, width=50,
            bg=theme['terminal_bg'], fg=theme['terminal_fg'],
            selectbackground=theme['select_bg'], selectforeground=theme['select_fg'],
            insertbackground=theme['fg']
        )
        self.overview_terminal.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
    
    def connect_pi(self, pi_type):
        """Connect to a specific Pi"""
        config = self.pi_configs[pi_type]
        
        if config['connected']:
            self.disconnect_pi(pi_type)
            return
        
        # Get connection parameters
        host = getattr(self, f'{pi_type}_host_var').get()
        port = int(getattr(self, f'{pi_type}_port_var').get())
        username = getattr(self, f'{pi_type}_user_var').get()
        password = getattr(self, f'{pi_type}_pass_var').get()
        
        if not all([host, username, password]):
            messagebox.showerror("Error", f"Please fill in all {config['name']} connection fields")
            return
        
        def connect_thread():
            try:
                # Create SSH connection
                ssh_client = paramiko.SSHClient()
                ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh_client.connect(host, port=port, username=username, password=password)
                
                # Create SCP client
                scp_client = SCPClient(ssh_client.get_transport())
                
                # Update configuration
                config['ssh'] = ssh_client
                config['scp'] = scp_client
                config['connected'] = True
                
                self.root.after(0, lambda: self.on_pi_connection_success(pi_type))
                
            except Exception as e:
                self.root.after(0, lambda: self.on_pi_connection_error(pi_type, str(e)))
        
        # Update UI
        getattr(self, f'{pi_type}_status_var').set("Connecting...")
        getattr(self, f'{pi_type}_connect_btn').config(state="disabled")
        
        # Start connection in background
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def on_pi_connection_success(self, pi_type):
        """Handle successful Pi connection"""
        config = self.pi_configs[pi_type]
        
        # Update UI
        getattr(self, f'{pi_type}_status_var').set("Connected")
        getattr(self, f'{pi_type}_connect_btn').config(text="Disconnect", state="normal")
        getattr(self, f'{pi_type}_upload_btn').config(state="normal")
        getattr(self, f'{pi_type}_exec_btn').config(state="normal")
        getattr(self, f'{pi_type}_stop_btn').config(state="normal")
        
        # Update status in overview
        if pi_type == 'navigation':
            self.nav_status_var.set("Connected")
        else:
            self.viz_status_var.set("Connected")
        
        # Log connection
        terminal = getattr(self, f'{pi_type}_terminal')
        terminal.insert(tk.END, f"Connected to {config['name']}\n")
        terminal.see(tk.END)
        
        print(f"✅ {config['name']} connected")
    
    def on_pi_connection_error(self, pi_type, error_msg):
        """Handle Pi connection error"""
        config = self.pi_configs[pi_type]
        
        # Update UI
        getattr(self, f'{pi_type}_status_var').set("Connection Failed")
        getattr(self, f'{pi_type}_connect_btn').config(state="normal")
        
        messagebox.showerror("Connection Error", f"Failed to connect to {config['name']}: {error_msg}")
        print(f"❌ {config['name']} connection failed: {error_msg}")
    
    def disconnect_pi(self, pi_type):
        """Disconnect from a specific Pi"""
        config = self.pi_configs[pi_type]
        
        # Close connections
        if config['scp']:
            config['scp'].close()
        if config['ssh']:
            config['ssh'].close()
        
        # Reset configuration
        config['ssh'] = None
        config['scp'] = None
        config['connected'] = False
        config['last_file'] = None
        
        # Update UI
        getattr(self, f'{pi_type}_status_var').set("Disconnected")
        getattr(self, f'{pi_type}_connect_btn').config(text="Connect", state="normal")
        getattr(self, f'{pi_type}_upload_btn').config(state="disabled")
        getattr(self, f'{pi_type}_run_btn').config(state="disabled")
        getattr(self, f'{pi_type}_exec_btn').config(state="disabled")
        getattr(self, f'{pi_type}_stop_btn').config(state="disabled")
        
        # Update status in overview
        if pi_type == 'navigation':
            self.nav_status_var.set("Disconnected")
        else:
            self.viz_status_var.set("Disconnected")
        
        # Log disconnection
        terminal = getattr(self, f'{pi_type}_terminal')
        terminal.insert(tk.END, f"Disconnected from {config['name']}\n")
        terminal.see(tk.END)
        
        print(f"🔌 {config['name']} disconnected")
    
    def upload_file(self, pi_type):
        """Upload file to specific Pi"""
        config = self.pi_configs[pi_type]
        
        if not config['connected']:
            messagebox.showerror("Error", f"Not connected to {config['name']}")
            return
        
        # Select file
        file_path = filedialog.askopenfilename(
            title=f"Select file for {config['name']}",
            filetypes=[("Python files", "*.py"), ("All files", "*.*")]
        )
        
        if not file_path:
            return
        
        def upload_thread():
            try:
                filename = os.path.basename(file_path)
                remote_path = getattr(self, f'{pi_type}_path_var').get() + filename
                
                config['scp'].put(file_path, remote_path)
                config['last_file'] = remote_path
                
                self.root.after(0, lambda: self.on_upload_success(pi_type, filename, remote_path))
                
            except Exception as e:
                self.root.after(0, lambda: self.on_upload_error(pi_type, str(e)))
        
        # Log upload start
        terminal = getattr(self, f'{pi_type}_terminal')
        terminal.insert(tk.END, f"Uploading {os.path.basename(file_path)}...\n")
        terminal.see(tk.END)
        
        threading.Thread(target=upload_thread, daemon=True).start()
    
    def on_upload_success(self, pi_type, filename, remote_path):
        """Handle successful file upload"""
        config = self.pi_configs[pi_type]
        
        terminal = getattr(self, f'{pi_type}_terminal')
        terminal.insert(tk.END, f"✅ Successfully uploaded {filename} to {remote_path}\n")
        terminal.see(tk.END)
        
        getattr(self, f'{pi_type}_run_btn').config(state="normal")
        print(f"✅ File uploaded to {config['name']}: {filename}")
    
    def on_upload_error(self, pi_type, error_msg):
        """Handle file upload error"""
        terminal = getattr(self, f'{pi_type}_terminal')
        terminal.insert(tk.END, f"❌ Upload failed: {error_msg}\n")
        terminal.see(tk.END)
        print(f"❌ Upload to {self.pi_configs[pi_type]['name']} failed: {error_msg}")
    
    def run_file(self, pi_type):
        """Run last uploaded file on specific Pi"""
        config = self.pi_configs[pi_type]
        
        if not config['connected'] or not config['last_file']:
            messagebox.showerror("Error", f"No file uploaded to {config['name']} or not connected")
            return
        
        # Build run command
        remote_path = getattr(self, f'{pi_type}_path_var').get()
        command = f"cd {remote_path} && source venv/bin/activate && python3 -u {config['last_file']}"
        
        self.execute_command_internal(pi_type, command)
    
    def stop_program(self, pi_type):
        """Stop running program on specific Pi"""
        config = self.pi_configs[pi_type]
        
        if not config['connected']:
            messagebox.showerror("Error", f"Not connected to {config['name']}")
            return
        
        def stop_thread():
            try:
                # Kill Python processes
                ssh_client = config['ssh']
                stdin, stdout, stderr = ssh_client.exec_command("pkill -SIGTERM -f 'python.*\\.py'")
                
                self.root.after(0, lambda: self.log_to_terminal(pi_type, f"🛑 Stop signal sent to {config['name']}\n"))
                
            except Exception as e:
                self.root.after(0, lambda: self.log_to_terminal(pi_type, f"❌ Stop failed: {str(e)}\n"))
        
        threading.Thread(target=stop_thread, daemon=True).start()
    
    def execute_command(self, pi_type):
        """Execute command on specific Pi"""
        command = getattr(self, f'{pi_type}_command_var').get().strip()
        if command:
            self.execute_command_internal(pi_type, command)
            getattr(self, f'{pi_type}_command_var').set("")
    
    def execute_command_internal(self, pi_type, command):
        """Execute command on specific Pi (internal)"""
        config = self.pi_configs[pi_type]
        
        if not config['connected']:
            return
        
        def execute_thread():
            try:
                self.root.after(0, lambda: self.log_to_terminal(pi_type, f"$ {command}\n"))
                
                stdin, stdout, stderr = config['ssh'].exec_command(command, get_pty=True)
                
                # Read output in real-time
                stdout.channel.settimeout(0.1)
                stderr.channel.settimeout(0.1)
                
                while not stdout.channel.exit_status_ready():
                    if stdout.channel.recv_ready():
                        data = stdout.channel.recv(1024).decode('utf-8', errors='replace')
                        if data:
                            self.root.after(0, lambda d=data: self.log_to_terminal(pi_type, d))
                    
                    if stderr.channel.recv_stderr_ready():
                        error_data = stderr.channel.recv_stderr(1024).decode('utf-8', errors='replace')
                        if error_data:
                            self.root.after(0, lambda d=error_data: self.log_to_terminal(pi_type, f"[ERROR] {d}"))
                    
                    time.sleep(0.01)
                
                # Read remaining output
                remaining_out = stdout.read().decode('utf-8', errors='replace')
                if remaining_out:
                    self.root.after(0, lambda d=remaining_out: self.log_to_terminal(pi_type, d))
                
                remaining_err = stderr.read().decode('utf-8', errors='replace')
                if remaining_err:
                    self.root.after(0, lambda d=remaining_err: self.log_to_terminal(pi_type, f"[ERROR] {d}"))
                    
            except Exception as e:
                self.root.after(0, lambda: self.log_to_terminal(pi_type, f"❌ Command failed: {str(e)}\n"))
        
        threading.Thread(target=execute_thread, daemon=True).start()
    
    def log_to_terminal(self, pi_type, message):
        """Log message to specific Pi terminal"""
        terminal = getattr(self, f'{pi_type}_terminal')
        terminal.insert(tk.END, message)
        terminal.see(tk.END)
    
    def toggle_telemetry(self):
        """Toggle telemetry monitoring on/off"""
        if not self.telemetry_running:
            self.start_telemetry()
        else:
            self.stop_telemetry()
    
    def start_telemetry(self):
        """Start telemetry monitoring"""
        try:
            from pi_communication import VisualizationDataReceiver
            
            self.telemetry_receiver = VisualizationDataReceiver()
            self.telemetry_receiver.start_server(
                data_callback=self.handle_telemetry_data,
                connection_callback=self.handle_telemetry_connection
            )
            
            self.telemetry_running = True
            self.telemetry_btn.configure(text="📡 Stop Telemetry")
            self.telemetry_status_var.set("Running")
            
            self.log_to_overview("📡 Telemetry monitoring started on port 8888\n")
            print("📡 Telemetry monitoring started")
            
        except Exception as e:
            messagebox.showerror("Telemetry Error", f"Failed to start telemetry: {e}")
            print(f"❌ Telemetry start failed: {e}")
    
    def stop_telemetry(self):
        """Stop telemetry monitoring"""
        if self.telemetry_receiver:
            self.telemetry_receiver.stop_server()
            self.telemetry_receiver = None
        
        self.telemetry_running = False
        self.telemetry_btn.configure(text="📡 Start Telemetry")
        self.telemetry_status_var.set("Stopped")
        
        self.log_to_overview("📡 Telemetry monitoring stopped\n")
        print("📡 Telemetry monitoring stopped")
    
    def handle_telemetry_data(self, data: Dict[str, Any]):
        """Handle incoming telemetry data"""
        # Add to telemetry buffer
        self.telemetry_data.append(data)
        if len(self.telemetry_data) > self.max_telemetry_points:
            self.telemetry_data.pop(0)
        
        # Log to overview terminal
        timestamp = datetime.fromtimestamp(data.get('timestamp', time.time())).strftime('%H:%M:%S')
        nav_state = data.get('nav_state', 'UNKNOWN')
        bt_distance = data.get('bluetooth_distance')
        us_distance = data.get('ultrasonic_distance')
        
        log_msg = f"[{timestamp}] {nav_state}"
        if bt_distance:
            log_msg += f" | BT: {bt_distance:.1f}m"
        if us_distance:
            log_msg += f" | US: {us_distance:.1f}cm"
        log_msg += "\n"
        
        self.log_to_overview(log_msg)
    
    def handle_telemetry_connection(self, message: str):
        """Handle telemetry connection events"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_to_overview(f"[{timestamp}] {message}\n")
    
    def log_to_overview(self, message):
        """Log message to overview terminal"""
        self.overview_terminal.insert(tk.END, message)
        self.overview_terminal.see(tk.END)
    
    def update_telemetry_plots(self):
        """Update telemetry visualization plots"""
        if not self.telemetry_data:
            self.root.after(1000, self.update_telemetry_plots)
            return
        
        # Extract data for plotting
        timestamps = [d.get('timestamp', 0) for d in self.telemetry_data]
        bt_distances = [d.get('bluetooth_distance') for d in self.telemetry_data]
        us_distances = [d.get('ultrasonic_distance') for d in self.telemetry_data]
        rssi_values = [d.get('bluetooth_rssi') for d in self.telemetry_data]
        nav_states = [d.get('nav_state', 'UNKNOWN') for d in self.telemetry_data]
        
        # Convert timestamps to relative time
        if timestamps:
            base_time = timestamps[0]
            rel_times = [(t - base_time)/60 for t in timestamps]  # Minutes
        else:
            rel_times = []
        
        # Clear and update plots
        self.ax1.clear()
        self.ax1.set_title("Bluetooth Distance")
        self.ax1.set_ylabel("Distance (m)")
        self.ax1.grid(True)
        if any(d is not None for d in bt_distances):
            valid_bt = [(t, d) for t, d in zip(rel_times, bt_distances) if d is not None]
            if valid_bt:
                times, dists = zip(*valid_bt)
                self.ax1.plot(times, dists, 'b-', label='BT Distance')
                self.ax1.legend()
        
        self.ax2.clear()
        self.ax2.set_title("Ultrasonic Distance")
        self.ax2.set_ylabel("Distance (cm)")
        self.ax2.grid(True)
        if any(d is not None for d in us_distances):
            valid_us = [(t, d) for t, d in zip(rel_times, us_distances) if d is not None]
            if valid_us:
                times, dists = zip(*valid_us)
                self.ax2.plot(times, dists, 'r-', label='US Distance')
                self.ax2.legend()
        
        self.ax3.clear()
        self.ax3.set_title("RSSI Signal")
        self.ax3.set_ylabel("RSSI (dBm)")
        self.ax3.grid(True)
        if any(r is not None for r in rssi_values):
            valid_rssi = [(t, r) for t, r in zip(rel_times, rssi_values) if r is not None]
            if valid_rssi:
                times, rssi = zip(*valid_rssi)
                self.ax3.plot(times, rssi, 'g-', label='RSSI')
                self.ax3.legend()
        
        self.ax4.clear()
        self.ax4.set_title("Navigation State")
        self.ax4.set_ylabel("State")
        self.ax4.grid(True)
        
        # Convert states to numbers for plotting
        state_map = {'SEARCHING': 0, 'FORWARD_TRACKING': 1, 'TURN_TESTING': 2, 
                    'APPROACHING': 3, 'HOLDING': 4, 'SMART_AVOIDING': 5, 'BACKING': 6}
        state_nums = [state_map.get(s, -1) for s in nav_states]
        if rel_times and state_nums:
            self.ax4.plot(rel_times, state_nums, 'ko-', markersize=3)
            self.ax4.set_yticks(list(state_map.values()))
            self.ax4.set_yticklabels(list(state_map.keys()), fontsize=8)
        
        self.telemetry_canvas.draw()
        
        # Schedule next update
        self.root.after(1000, self.update_telemetry_plots)
    
    def emergency_stop_all(self):
        """Emergency stop all connected Pi systems"""
        for pi_type, config in self.pi_configs.items():
            if config['connected']:
                self.stop_program(pi_type)
        
        self.log_to_overview(f"[{datetime.now().strftime('%H:%M:%S')}] 🛑 EMERGENCY STOP ALL SYSTEMS\n")
        print("🛑 Emergency stop all systems")
    
    def exit_application(self):
        """Clean exit with resource cleanup"""
        # Stop telemetry
        if self.telemetry_running:
            self.stop_telemetry()
        
        # Disconnect all Pi connections
        for pi_type in self.pi_configs:
            if self.pi_configs[pi_type]['connected']:
                self.disconnect_pi(pi_type)
        
        # Close application
        self.root.quit()
        self.root.destroy()
        print("👋 Dual-Pi Dashboard V3 shutdown complete")


def main():
    """Main entry point for Dual-Pi Dashboard V3"""
    print("=" * 80)
    print("🤖 DUAL-PI ROVER DASHBOARD V3")
    print("=" * 80)
    print()
    print("FEATURES:")
    print("  ✅ Independent SSH connections to Navigation Pi and Visualization Pi")
    print("  ✅ File upload targeting specific Pi systems")
    print("  ✅ Real-time telemetry monitoring via Pi-to-Pi communication")
    print("  ✅ Live data visualization with system overview")
    print("  ✅ Emergency stop functionality for all systems")
    print("  ✅ Tabbed interface for organized Pi management")
    print("  ✅ Dark/Light theme support")
    print()
    print("SETUP REQUIREMENTS:")
    print("  📋 Navigation Pi: 192.168.1.10 (default)")
    print("  📋 Visualization Pi: 192.168.1.11 (default)")
    print("  📋 Telemetry Port: 8888 (Pi-to-Pi communication)")
    print()
    
    root = tk.Tk()
    app = DualPiDashboard(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested...")
        app.exit_application()

if __name__ == "__main__":
    main()