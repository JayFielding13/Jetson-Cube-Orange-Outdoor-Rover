#!/usr/bin/env python3
"""
Raspberry Pi SSH Dashboard V3.7 - Real-time Terminal Fixed Edition
=================================================================

Enhanced SSH dashboard with separate dedicated tabs for each Pi AND working real-time terminal:
- 📡 Navigation Pi Tab: Dedicated file management for 192.168.254.65
- 🎨 Companion Pi Tab: Dedicated file management for 192.168.254.70
- 📊 System Monitoring: Both Pi system monitoring with REAL-TIME terminal updates
- ⚙️ Settings: Configuration and preferences

KEY FIX: Real-time terminal updates using channel-based reading with PTY allocation
Based on working Dashboard V2 terminal implementation

Author: Claude Code Assistant
Version: 3.7 - Real-time Terminal Fixed Edition
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog, scrolledtext
import tkinter.dnd as dnd
from tkinter.constants import *
import paramiko
from scp import SCPClient
import threading
import os
import io
import time
import socket
import stat
from datetime import datetime
from pathlib import Path
import subprocess
import json
import os.path
import select


class RemoteFileTree:
    """Remote file tree browser for rover files"""
    
    def __init__(self, ssh_client, tree_widget, pi_name="Pi"):
        self.ssh = ssh_client
        self.tree = tree_widget
        self.pi_name = pi_name
        self.file_cache = {}  # Cache for file listings
        
    def refresh_tree(self, path="/home/jay"):
        """Refresh the remote file tree"""
        try:
            if not self.ssh or not self.ssh.get_transport() or not self.ssh.get_transport().is_active():
                print(f"{self.pi_name}: SSH connection not active")
                return
                
            print(f"{self.pi_name}: Refreshing tree for path: {path}")
            
            # Clear existing tree
            for item in self.tree.get_children():
                self.tree.delete(item)
            
            # Build tree starting from root path
            self._build_tree_node("", path)
            print(f"{self.pi_name}: Successfully refreshed tree")
            
        except Exception as e:
            print(f"Error refreshing {self.pi_name} tree: {e}")
            # Show error in tree
            self.tree.insert("", "end", text=f"❌ Error: {str(e)}", values=(False, "", ""))
    
    def _build_tree_node(self, parent, path):
        """Build a tree node with its children"""
        try:
            # Add parent directory option for remote tree (if not at root)
            if parent == "" and path != "/" and "/" in path.rstrip("/"):
                parent_path = os.path.dirname(path.rstrip("/"))
                if not parent_path:
                    parent_path = "/"
                self.tree.insert("", 0, text="📁 ..", 
                               values=(True, "", parent_path))
            
            # Get directory listing
            stdin, stdout, stderr = self.ssh.exec_command(f"ls -la '{path}'")
            output = stdout.read().decode()
            
            items = []
            for line in output.strip().split('\n')[1:]:  # Skip first line (total)
                if line.strip():
                    parts = line.split()
                    if len(parts) >= 9:
                        permissions = parts[0]
                        size = parts[4]
                        name = " ".join(parts[8:])  # Handle names with spaces
                        
                        if name not in [".", ".."]:
                            is_dir = permissions.startswith('d')
                            full_path = os.path.join(path, name)
                            
                            items.append({
                                'name': name,
                                'is_dir': is_dir,
                                'size': size if not is_dir else "",
                                'path': full_path,
                                'permissions': permissions
                            })
            
            # Sort: directories first, then by name
            items.sort(key=lambda x: (not x['is_dir'], x['name'].lower()))
            
            # Insert items into tree
            for item in items:
                icon = "📁" if item['is_dir'] else "📄"
                display_name = f"{icon} {item['name']}"
                
                self.tree.insert("", "end", text=display_name,
                               values=(item['is_dir'], item['size'], item['path']))
                               
        except Exception as e:
            print(f"Error building tree node for {self.pi_name}: {e}")


class LocalFileTree:
    """Local file tree browser"""
    
    def __init__(self, tree_widget):
        self.tree = tree_widget
        
    def refresh_tree(self, path=None):
        """Refresh the local file tree"""
        if path is None:
            path = Path.home()
        else:
            path = Path(path)
            
        try:
            print(f"LocalFileTree: Refreshing tree for path: {path}")
            
            # Clear existing tree
            for item in self.tree.get_children():
                self.tree.delete(item)
            
            # Add parent directory option (if not at root)
            if path.parent != path:
                self.tree.insert("", 0, text="📁 ..", 
                               values=(True, "", str(path.parent)))
            
            # Get directory contents
            items = []
            try:
                for item in path.iterdir():
                    try:
                        is_dir = item.is_dir()
                        size = "" if is_dir else self._format_size(item.stat().st_size)
                        
                        items.append({
                            'name': item.name,
                            'is_dir': is_dir,
                            'size': size,
                            'path': str(item)
                        })
                    except (PermissionError, OSError):
                        # Skip items we can't access
                        continue
                        
            except PermissionError:
                self.tree.insert("", "end", text="❌ Permission denied", 
                               values=(False, "", ""))
                return
            
            # Sort: directories first, then by name
            items.sort(key=lambda x: (not x['is_dir'], x['name'].lower()))
            
            # Insert items into tree
            for item in items:
                icon = "📁" if item['is_dir'] else "📄"
                display_name = f"{icon} {item['name']}"
                
                self.tree.insert("", "end", text=display_name,
                               values=(item['is_dir'], item['size'], item['path']))
                               
            print(f"LocalFileTree: Successfully refreshed tree")
            
        except Exception as e:
            print(f"Error refreshing local tree: {e}")
            self.tree.insert("", "end", text=f"❌ Error: {str(e)}", 
                           values=(False, "", ""))
    
    def _format_size(self, size_bytes):
        """Format file size in human readable format"""
        if size_bytes == 0:
            return "0 B"
        
        for unit in ['B', 'KB', 'MB', 'GB']:
            if size_bytes < 1024.0:
                return f"{size_bytes:.1f} {unit}"
            size_bytes /= 1024.0
        return f"{size_bytes:.1f} TB"


class FileOperationDialog:
    """Dialog for file operations with progress tracking"""
    
    def __init__(self, parent, title="File Operation"):
        self.parent = parent
        self.dialog = tk.Toplevel(parent)
        self.dialog.title(title)
        self.dialog.geometry("500x300")
        self.dialog.transient(parent)
        self.dialog.grab_set()
        
        # Center the dialog
        self.dialog.geometry("+%d+%d" % (parent.winfo_rootx() + 50, parent.winfo_rooty() + 50))
        
        # Create GUI elements
        self.setup_gui()
        
        # Operation state
        self.cancelled = False
        
    def setup_gui(self):
        """Setup the dialog GUI"""
        main_frame = ttk.Frame(self.dialog, padding="10")
        main_frame.pack(fill='both', expand=True)
        
        # Status label
        self.status_label = ttk.Label(main_frame, text="Preparing operation...")
        self.status_label.pack(pady=(0, 10))
        
        # Progress bar
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(main_frame, variable=self.progress_var, 
                                          maximum=100, length=400)
        self.progress_bar.pack(pady=(0, 10), fill='x')
        
        # Log area
        log_frame = ttk.LabelFrame(main_frame, text="Operation Log", padding="5")
        log_frame.pack(fill='both', expand=True, pady=(0, 10))
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, width=60)
        self.log_text.pack(fill='both', expand=True)
        
        # Buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill='x')
        
        self.cancel_btn = ttk.Button(button_frame, text="Cancel", command=self.cancel_operation)
        self.cancel_btn.pack(side='right', padx=(5, 0))
        
        self.close_btn = ttk.Button(button_frame, text="Close", command=self.close_dialog, state='disabled')
        self.close_btn.pack(side='right')
        
    def update_status(self, message):
        """Update status message"""
        self.status_label.config(text=message)
        self.log_text.insert('end', f"{datetime.now().strftime('%H:%M:%S')} - {message}\n")
        self.log_text.see('end')
        self.dialog.update()
        
    def update_progress(self, value):
        """Update progress bar"""
        self.progress_var.set(value)
        self.dialog.update()
        
    def cancel_operation(self):
        """Cancel the operation"""
        self.cancelled = True
        self.update_status("Cancelling operation...")
        
    def operation_complete(self):
        """Mark operation as complete"""
        self.cancel_btn.config(state='disabled')
        self.close_btn.config(state='normal')
        self.update_progress(100)
        
    def close_dialog(self):
        """Close the dialog"""
        self.dialog.destroy()


class PiConnection:
    """Manages SSH connection to a specific Pi"""
    
    def __init__(self, name, host, port=22, username="jay"):
        self.name = name
        self.host = host
        self.port = port
        self.username = username
        self.ssh = None
        self.sftp = None
        self.connected = False
        self.last_connection_attempt = 0
        self.connection_lock = threading.Lock()
        self.current_channel = None  # Track current execution channel
        
    def connect(self, password=None):
        """Connect to the Pi"""
        with self.connection_lock:
            try:
                print(f"Connecting to {self.name} ({self.host})...")
                
                self.ssh = paramiko.SSHClient()
                self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                
                # Try to connect
                self.ssh.connect(self.host, port=self.port, username=self.username, 
                               password=password, timeout=10)
                
                # Create SFTP client
                self.sftp = self.ssh.open_sftp()
                
                self.connected = True
                self.last_connection_attempt = time.time()
                
                print(f"✅ Connected to {self.name}")
                return True
                
            except Exception as e:
                print(f"❌ Failed to connect to {self.name}: {e}")
                self.connected = False
                if self.ssh:
                    self.ssh.close()
                    self.ssh = None
                if self.sftp:
                    self.sftp.close()
                    self.sftp = None
                return False
    
    def disconnect(self):
        """Disconnect from the Pi"""
        with self.connection_lock:
            try:
                if self.current_channel:
                    try:
                        self.current_channel.close()
                    except:
                        pass
                    self.current_channel = None
                    
                if self.sftp:
                    self.sftp.close()
                    self.sftp = None
                    
                if self.ssh:
                    self.ssh.close()
                    self.ssh = None
                    
                self.connected = False
                print(f"Disconnected from {self.name}")
                
            except Exception as e:
                print(f"Error disconnecting from {self.name}: {e}")
    
    def is_connected(self):
        """Check if connection is active"""
        if not self.connected or not self.ssh:
            return False
            
        try:
            # Test connection with a simple command
            transport = self.ssh.get_transport()
            return transport and transport.is_active()
        except:
            return False


class EnhancedSSHDashboard:
    """Enhanced SSH Dashboard with dual Pi dedicated tabs and REAL-TIME terminal"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Rover SSH Dashboard V3.7 - Real-time Terminal Fixed")
        self.root.geometry("1400x900")
        
        # Pi connections
        self.nav_pi = PiConnection("Navigation Pi", "192.168.254.65")
        self.comp_pi = PiConnection("Companion Pi", "192.168.254.70")
        
        # GUI state
        self.dark_mode = False
        self.password = None
        
        # File trees (will be initialized per tab)
        self.nav_remote_tree = None
        self.comp_remote_tree = None
        self.local_trees = {}  # One local tree per tab
        
        # Setup GUI
        self.setup_styles()
        self.setup_menu()
        self.setup_main_interface()
        
        # Try initial connections
        threading.Thread(target=self.initial_connections, daemon=True).start()
    
    def setup_styles(self):
        """Setup ttk styles"""
        style = ttk.Style()
        style.theme_use('clam')  # Modern looking theme
        
        # Configure custom styles
        style.configure('Title.TLabel', font=('Arial', 12, 'bold'))
        style.configure('Status.TLabel', font=('Arial', 9))
        
    def setup_menu(self):
        """Setup menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Connect to Navigation Pi", command=self.connect_nav_pi)
        file_menu.add_command(label="Connect to Companion Pi", command=self.connect_comp_pi)
        file_menu.add_separator()
        file_menu.add_command(label="Disconnect All", command=self.disconnect_all)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)
        
        # Tools menu
        tools_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Tools", menu=tools_menu)
        tools_menu.add_command(label="Terminal", command=self.open_terminal)
        tools_menu.add_command(label="System Info", command=self.show_system_info)
        
        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)
    
    def setup_main_interface(self):
        """Setup the main interface with dedicated Pi tabs"""
        # Status bar
        self.setup_status_bar()
        
        # Main notebook with dedicated Pi tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=(0, 25))
        
        # Setup individual tabs
        self.setup_navigation_pi_tab()
        self.setup_companion_pi_tab()
        self.setup_system_monitoring_tab()
        self.setup_settings_tab()
        
    def setup_status_bar(self):
        """Setup status bar showing Pi connection status"""
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill='x', side='bottom', padx=5, pady=2)
        
        # Connection status indicators
        self.nav_status_label = ttk.Label(status_frame, text="📡 Navigation Pi: Disconnected", 
                                        style='Status.TLabel')
        self.nav_status_label.pack(side='left', padx=(0, 20))
        
        self.comp_status_label = ttk.Label(status_frame, text="🎨 Companion Pi: Disconnected", 
                                         style='Status.TLabel')
        self.comp_status_label.pack(side='left', padx=(0, 20))
        
        # General status
        self.general_status_label = ttk.Label(status_frame, text="Ready", style='Status.TLabel')
        self.general_status_label.pack(side='right')
        
        # Start status update thread
        threading.Thread(target=self.update_status_thread, daemon=True).start()
    
    def setup_navigation_pi_tab(self):
        """Setup dedicated Navigation Pi file management tab"""
        nav_frame = ttk.Frame(self.notebook)
        self.notebook.add(nav_frame, text="📡 Navigation Pi (192.168.254.65)")
        
        # Header
        header_frame = ttk.Frame(nav_frame)
        header_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(header_frame, text="📡 Navigation Pi File Management", 
                 style='Title.TLabel').pack(side='left')
        
        # Connection button
        self.nav_connect_btn = ttk.Button(header_frame, text="Connect", 
                                        command=self.connect_nav_pi)
        self.nav_connect_btn.pack(side='right', padx=(5, 0))
        
        # File management panes
        self.setup_pi_file_management(nav_frame, "nav")
        
    def setup_companion_pi_tab(self):
        """Setup dedicated Companion Pi file management tab"""
        comp_frame = ttk.Frame(self.notebook)
        self.notebook.add(comp_frame, text="🎨 Companion Pi (192.168.254.70)")
        
        # Header
        header_frame = ttk.Frame(comp_frame)
        header_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(header_frame, text="🎨 Companion Pi File Management", 
                 style='Title.TLabel').pack(side='left')
        
        # Connection button
        self.comp_connect_btn = ttk.Button(header_frame, text="Connect", 
                                         command=self.connect_comp_pi)
        self.comp_connect_btn.pack(side='right', padx=(5, 0))
        
        # File management panes
        self.setup_pi_file_management(comp_frame, "comp")
    
    def setup_pi_file_management(self, parent, pi_type):
        """Setup file management interface for a specific Pi"""
        # Create main paned window for dual-pane layout
        main_paned = ttk.PanedWindow(parent, orient='horizontal')
        main_paned.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Left pane - Local files
        self.setup_local_file_pane(main_paned, pi_type)
        
        # Right pane - Remote files for this Pi
        self.setup_remote_file_pane(main_paned, pi_type)
        
        # Bottom pane - File operations
        self.setup_file_operations_pane(parent, pi_type)
    
    def setup_local_file_pane(self, parent, pi_type):
        """Setup local file browser pane for a specific Pi tab"""
        local_frame = ttk.LabelFrame(parent, text="🖥️ Local Files", padding="5")
        parent.add(local_frame, weight=1)
        
        # Path navigation
        nav_frame = ttk.Frame(local_frame)
        nav_frame.pack(fill='x', pady=(0, 5))
        
        ttk.Label(nav_frame, text="Path:").pack(side='left')
        
        # Store path variable per Pi
        path_var = tk.StringVar(value=str(Path.home()))
        setattr(self, f"{pi_type}_local_path_var", path_var)
        
        path_entry = ttk.Entry(nav_frame, textvariable=path_var)
        path_entry.pack(side='left', fill='x', expand=True, padx=(5, 5))
        
        ttk.Button(nav_frame, text="Go", 
                  command=lambda: self.navigate_local(pi_type)).pack(side='right')
        ttk.Button(nav_frame, text="Home", 
                  command=lambda: self.go_local_home(pi_type)).pack(side='right', padx=(0, 5))
        
        # File tree
        tree_frame = ttk.Frame(local_frame)
        tree_frame.pack(fill='both', expand=True)
        
        # Create tree widget for this Pi
        tree_widget = ttk.Treeview(tree_frame, columns=('is_dir', 'size', 'path'), show='tree')
        tree_widget.heading('#0', text='File Name', anchor='w')
        tree_widget.column('#0', width=300, minwidth=200)
        tree_widget.column('is_dir', width=0, minwidth=0, stretch=False)  # Hidden
        tree_widget.column('size', width=120, minwidth=80)
        tree_widget.column('path', width=0, minwidth=0, stretch=False)  # Hidden path column
        
        # Store tree widget reference
        setattr(self, f"{pi_type}_local_tree_widget", tree_widget)
        
        # Scrollbars
        v_scroll = ttk.Scrollbar(tree_frame, orient='vertical', command=tree_widget.yview)
        h_scroll = ttk.Scrollbar(tree_frame, orient='horizontal', command=tree_widget.xview)
        tree_widget.configure(yscrollcommand=v_scroll.set, xscrollcommand=h_scroll.set)
        
        tree_widget.pack(side='left', fill='both', expand=True)
        v_scroll.pack(side='right', fill='y')
        h_scroll.pack(side='bottom', fill='x')
        
        # Bind events
        tree_widget.bind('<Double-1>', lambda e: self.on_local_double_click(pi_type, e))
        tree_widget.bind('<Button-3>', lambda e: self.on_local_right_click(pi_type, e))
        
        # Initialize local tree
        local_tree = LocalFileTree(tree_widget)
        self.local_trees[pi_type] = local_tree
        local_tree.refresh_tree()
    
    def setup_remote_file_pane(self, parent, pi_type):
        """Setup remote file browser pane for a specific Pi"""
        pi_name = "Navigation Pi" if pi_type == "nav" else "Companion Pi"
        pi_ip = "192.168.254.65" if pi_type == "nav" else "192.168.254.70"
        
        remote_frame = ttk.LabelFrame(parent, text=f"🤖 {pi_name} Files ({pi_ip})", padding="5")
        parent.add(remote_frame, weight=1)
        
        # Path navigation
        nav_frame = ttk.Frame(remote_frame)
        nav_frame.pack(fill='x', pady=(0, 5))
        
        ttk.Label(nav_frame, text="Path:").pack(side='left')
        
        # Store remote path variable per Pi
        remote_path_var = tk.StringVar(value="/home/jay")
        setattr(self, f"{pi_type}_remote_path_var", remote_path_var)
        
        remote_path_entry = ttk.Entry(nav_frame, textvariable=remote_path_var)
        remote_path_entry.pack(side='left', fill='x', expand=True, padx=(5, 5))
        
        ttk.Button(nav_frame, text="Refresh", 
                  command=lambda: self.refresh_remote_tree(pi_type)).pack(side='right')
        
        # File tree
        tree_frame = ttk.Frame(remote_frame)
        tree_frame.pack(fill='both', expand=True)
        
        # Create remote tree widget for this Pi
        remote_tree_widget = ttk.Treeview(tree_frame, columns=('is_dir', 'size', 'path'), show='tree')
        remote_tree_widget.heading('#0', text='File Name', anchor='w')
        remote_tree_widget.column('#0', width=300, minwidth=200)
        remote_tree_widget.column('is_dir', width=0, minwidth=0, stretch=False)  # Hidden
        remote_tree_widget.column('size', width=120, minwidth=80)
        remote_tree_widget.column('path', width=0, minwidth=0, stretch=False)  # Hidden path column
        
        # Store remote tree widget reference
        setattr(self, f"{pi_type}_remote_tree_widget", remote_tree_widget)
        
        # Scrollbars
        v_scroll = ttk.Scrollbar(tree_frame, orient='vertical', command=remote_tree_widget.yview)
        h_scroll = ttk.Scrollbar(tree_frame, orient='horizontal', command=remote_tree_widget.xview)
        remote_tree_widget.configure(yscrollcommand=v_scroll.set, xscrollcommand=h_scroll.set)
        
        remote_tree_widget.pack(side='left', fill='both', expand=True)
        v_scroll.pack(side='right', fill='y')
        h_scroll.pack(side='bottom', fill='x')
        
        # Bind events
        remote_tree_widget.bind('<Double-1>', lambda e: self.on_remote_double_click(pi_type, e))
        remote_tree_widget.bind('<Button-3>', lambda e: self.on_remote_right_click(pi_type, e))
        
        # Initialize remote file tree
        pi_connection = self.nav_pi if pi_type == "nav" else self.comp_pi
        remote_tree = RemoteFileTree(pi_connection.ssh, remote_tree_widget, pi_name)
        setattr(self, f"{pi_type}_remote_tree", remote_tree)
    
    def setup_file_operations_pane(self, parent, pi_type):
        """Setup file operations panel for a specific Pi"""
        pi_name = "Navigation Pi" if pi_type == "nav" else "Companion Pi"
        
        ops_frame = ttk.LabelFrame(parent, text=f"🚀 {pi_name} Operations", padding="5")
        ops_frame.pack(fill='x', padx=5, pady=(0, 5))
        
        # File operation buttons
        button_frame = ttk.Frame(ops_frame)
        button_frame.pack(fill='x', pady=5)
        
        ttk.Button(button_frame, text="📤 Upload Selected", 
                  command=lambda: self.upload_selected_files(pi_type)).pack(side='left', padx=2)
        ttk.Button(button_frame, text="📥 Download Selected", 
                  command=lambda: self.download_selected_files(pi_type)).pack(side='left', padx=2)
        ttk.Button(button_frame, text="🗑️ Delete Selected", 
                  command=lambda: self.delete_selected_files(pi_type)).pack(side='left', padx=2)
        ttk.Button(button_frame, text="▶️ Run Selected", 
                  command=lambda: self.run_selected_script(pi_type)).pack(side='left', padx=2)
        
        # Quick navigation buttons
        nav_frame = ttk.Frame(ops_frame)
        nav_frame.pack(fill='x', pady=5)
        
        ttk.Label(nav_frame, text="Quick Navigation:").pack(side='left')
        
        if pi_type == "nav":
            # Navigation Pi specific paths
            ttk.Button(nav_frame, text="📡 Pi Code", 
                      command=lambda: self.quick_navigate(pi_type, "/home/jay/rover_project/Pi Code")).pack(side='left', padx=2)
            ttk.Button(nav_frame, text="🤖 Arduino Code", 
                      command=lambda: self.quick_navigate(pi_type, "/home/jay/rover_project/Arduino Code")).pack(side='left', padx=2)
        else:
            # Companion Pi specific paths  
            ttk.Button(nav_frame, text="🎨 Visualization", 
                      command=lambda: self.quick_navigate(pi_type, "/home/jay/rover_project/Visualization")).pack(side='left', padx=2)
            ttk.Button(nav_frame, text="📊 Data Logs", 
                      command=lambda: self.quick_navigate(pi_type, "/home/jay/rover_project/Data Logs")).pack(side='left', padx=2)
        
        ttk.Button(nav_frame, text="🏠 Home", 
                  command=lambda: self.quick_navigate(pi_type, "/home/jay")).pack(side='left', padx=2)
    
    def setup_system_monitoring_tab(self):
        """Setup system monitoring tab for both Pis with REAL-TIME terminals"""
        monitoring_frame = ttk.Frame(self.notebook)
        self.notebook.add(monitoring_frame, text="📊 System Monitoring")
        
        # Split into two monitoring panels
        monitor_paned = ttk.PanedWindow(monitoring_frame, orient='horizontal')
        monitor_paned.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Navigation Pi monitoring
        nav_monitor_frame = ttk.LabelFrame(monitor_paned, text="📡 Navigation Pi Terminal", padding="5")
        monitor_paned.add(nav_monitor_frame, weight=1)
        
        # Real-time terminal for Navigation Pi
        self.nav_monitor_text = scrolledtext.ScrolledText(nav_monitor_frame, height=25, width=60,
                                                        bg='black', fg='green', insertbackground='green')
        self.nav_monitor_text.pack(fill='both', expand=True, pady=(0, 5))
        
        # Command entry for Navigation Pi
        nav_cmd_frame = ttk.Frame(nav_monitor_frame)
        nav_cmd_frame.pack(fill='x', pady=(0, 5))
        
        ttk.Label(nav_cmd_frame, text="Command:").pack(side='left')
        self.nav_command_var = tk.StringVar()
        nav_cmd_entry = ttk.Entry(nav_cmd_frame, textvariable=self.nav_command_var)
        nav_cmd_entry.pack(side='left', fill='x', expand=True, padx=(5, 5))
        nav_cmd_entry.bind('<Return>', lambda e: self.execute_command('nav'))
        
        ttk.Button(nav_cmd_frame, text="Execute", 
                  command=lambda: self.execute_command('nav')).pack(side='right')
        ttk.Button(nav_cmd_frame, text="Stop", 
                  command=lambda: self.stop_remote_program('nav')).pack(side='right', padx=(0, 5))
        
        # Companion Pi monitoring
        comp_monitor_frame = ttk.LabelFrame(monitor_paned, text="🎨 Companion Pi Terminal", padding="5")
        monitor_paned.add(comp_monitor_frame, weight=1)
        
        # Real-time terminal for Companion Pi
        self.comp_monitor_text = scrolledtext.ScrolledText(comp_monitor_frame, height=25, width=60,
                                                         bg='black', fg='cyan', insertbackground='cyan')
        self.comp_monitor_text.pack(fill='both', expand=True, pady=(0, 5))
        
        # Command entry for Companion Pi
        comp_cmd_frame = ttk.Frame(comp_monitor_frame)
        comp_cmd_frame.pack(fill='x', pady=(0, 5))
        
        ttk.Label(comp_cmd_frame, text="Command:").pack(side='left')
        self.comp_command_var = tk.StringVar()
        comp_cmd_entry = ttk.Entry(comp_cmd_frame, textvariable=self.comp_command_var)
        comp_cmd_entry.pack(side='left', fill='x', expand=True, padx=(5, 5))
        comp_cmd_entry.bind('<Return>', lambda e: self.execute_command('comp'))
        
        ttk.Button(comp_cmd_frame, text="Execute", 
                  command=lambda: self.execute_command('comp')).pack(side='right')
        ttk.Button(comp_cmd_frame, text="Stop", 
                  command=lambda: self.stop_remote_program('comp')).pack(side='right', padx=(0, 5))
        
        # Control buttons
        control_frame = ttk.Frame(monitoring_frame)
        control_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Button(control_frame, text="🔄 Clear Nav Terminal", 
                  command=lambda: self.clear_terminal('nav')).pack(side='left', padx=2)
        ttk.Button(control_frame, text="🔄 Clear Comp Terminal", 
                  command=lambda: self.clear_terminal('comp')).pack(side='left', padx=2)
        ttk.Button(control_frame, text="📊 System Health Check", 
                  command=self.system_health_check).pack(side='left', padx=2)
    
    def setup_settings_tab(self):
        """Setup settings and preferences tab"""
        settings_frame = ttk.Frame(self.notebook)
        self.notebook.add(settings_frame, text="⚙️ Settings")
        
        # Connection settings
        conn_frame = ttk.LabelFrame(settings_frame, text="Connection Settings", padding="10")
        conn_frame.pack(fill='x', padx=10, pady=10)
        
        # Navigation Pi settings
        nav_settings_frame = ttk.LabelFrame(conn_frame, text="📡 Navigation Pi", padding="5")
        nav_settings_frame.pack(fill='x', pady=5)
        
        ttk.Label(nav_settings_frame, text="Host:").grid(row=0, column=0, sticky='w')
        self.nav_host_var = tk.StringVar(value="192.168.254.65")
        ttk.Entry(nav_settings_frame, textvariable=self.nav_host_var, width=20).grid(row=0, column=1, padx=5)
        
        # Companion Pi settings
        comp_settings_frame = ttk.LabelFrame(conn_frame, text="🎨 Companion Pi", padding="5")
        comp_settings_frame.pack(fill='x', pady=5)
        
        ttk.Label(comp_settings_frame, text="Host:").grid(row=0, column=0, sticky='w')
        self.comp_host_var = tk.StringVar(value="192.168.254.70")
        ttk.Entry(comp_settings_frame, textvariable=self.comp_host_var, width=20).grid(row=0, column=1, padx=5)
        
        # Apply settings button
        ttk.Button(conn_frame, text="Apply Settings", command=self.apply_settings).pack(pady=10)
        
        # About section
        about_frame = ttk.LabelFrame(settings_frame, text="About", padding="10")
        about_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        about_text = """
Dashboard V3.7 - Real-time Terminal Fixed Edition

Key Features:
📡 Dedicated Navigation Pi tab with independent file management
🎨 Dedicated Companion Pi tab with independent file management  
📊 REAL-TIME terminal updates using channel-based reading
🔄 Individual connection management per Pi
🚀 Pi-specific quick navigation and operations
📁 Parallel file operations across both systems
⚡ Fixed terminal buffering issues with PTY allocation

Perfect for dual Pi rover development with live debugging!

Version: 3.7 - Real-time Terminal Fixed Edition
Author: Claude Code Assistant
        """
        
        about_label = ttk.Label(about_frame, text=about_text, justify='left')
        about_label.pack(fill='both', expand=True)

    # REAL-TIME TERMINAL IMPLEMENTATION - THE FIX!
    def execute_command(self, pi_type):
        """Execute command with REAL-TIME output using channel-based reading"""
        pi_connection = self.nav_pi if pi_type == "nav" else self.comp_pi
        command_var = self.nav_command_var if pi_type == "nav" else self.comp_command_var
        terminal_text = self.nav_monitor_text if pi_type == "nav" else self.comp_monitor_text
        
        command = command_var.get().strip()
        if not command:
            return
            
        if not pi_connection.is_connected():
            terminal_text.insert(tk.END, f"❌ {pi_connection.name} not connected\n")
            terminal_text.see(tk.END)
            return
        
        # Clear command entry
        command_var.set("")
        
        def execute_thread():
            try:
                # Insert command prompt
                self.root.after(0, lambda: terminal_text.insert(tk.END, f"$ {command}\n"))
                self.root.after(0, lambda: terminal_text.see(tk.END))
                
                # Execute command with PTY (CRITICAL FOR REAL-TIME OUTPUT)
                stdin, stdout, stderr = pi_connection.ssh.exec_command(command, get_pty=True)
                pi_connection.current_channel = stdout.channel
                
                # Set non-blocking mode for REAL-TIME output (THE KEY FIX!)
                stdout.channel.settimeout(0.1)
                stderr.channel.settimeout(0.1)
                
                # Real-time output reading loop
                while not stdout.channel.exit_status_ready():
                    # Check for stdout data
                    if stdout.channel.recv_ready():
                        data = stdout.channel.recv(1024).decode('utf-8', errors='replace')
                        if data:
                            # Immediate GUI update using root.after(0, ...)
                            self.root.after(0, lambda d=data: terminal_text.insert(tk.END, d))
                            self.root.after(0, lambda: terminal_text.see(tk.END))
                    
                    # Check for stderr data
                    if stderr.channel.recv_stderr_ready():
                        error_data = stderr.channel.recv_stderr(1024).decode('utf-8', errors='replace')
                        if error_data:
                            self.root.after(0, lambda d=error_data: terminal_text.insert(tk.END, f"[ERROR] {d}"))
                            self.root.after(0, lambda: terminal_text.see(tk.END))
                    
                    # Small delay to prevent excessive CPU usage
                    time.sleep(0.01)
                
                # Read any remaining output
                remaining_out = stdout.read().decode('utf-8', errors='replace')
                if remaining_out:
                    self.root.after(0, lambda d=remaining_out: terminal_text.insert(tk.END, d))
                
                remaining_err = stderr.read().decode('utf-8', errors='replace')
                if remaining_err:
                    self.root.after(0, lambda d=remaining_err: terminal_text.insert(tk.END, f"[ERROR] {d}"))
                    
                self.root.after(0, lambda: terminal_text.see(tk.END))
                
            except Exception as e:
                error_msg = str(e)
                self.root.after(0, lambda: terminal_text.insert(tk.END, f"Command failed: {error_msg}\n"))
                self.root.after(0, lambda: terminal_text.see(tk.END))
        
        # Execute in background thread
        threading.Thread(target=execute_thread, daemon=True).start()
    
    def stop_remote_program(self, pi_type):
        """Stop running program on specified Pi"""
        pi_connection = self.nav_pi if pi_type == "nav" else self.comp_pi
        terminal_text = self.nav_monitor_text if pi_type == "nav" else self.comp_monitor_text
        
        if not pi_connection.is_connected():
            terminal_text.insert(tk.END, f"❌ {pi_connection.name} not connected\n")
            terminal_text.see(tk.END)
            return
        
        def stop_thread():
            try:
                # Strategy 1: Send Ctrl+C to current channel
                if pi_connection.current_channel and not pi_connection.current_channel.closed:
                    try:
                        pi_connection.current_channel.send('\x03')  # Ctrl+C character
                        self.root.after(0, lambda: terminal_text.insert(tk.END, "\n^C (Interrupt signal sent)\n"))
                        self.root.after(0, lambda: terminal_text.see(tk.END))
                    except Exception as e:
                        self.root.after(0, lambda: terminal_text.insert(tk.END, f"\nChannel interrupt failed: {str(e)}\n"))
                
                # Strategy 2: Kill Python processes
                try:
                    stdin, stdout, stderr = pi_connection.ssh.exec_command("pkill -SIGTERM -f 'python.*\\.py'")
                    self.root.after(0, lambda: terminal_text.insert(tk.END, "SIGTERM sent to Python processes\n"))
                except Exception as e:
                    self.root.after(0, lambda: terminal_text.insert(tk.END, f"SIGTERM failed: {str(e)}\n"))
                
                self.root.after(0, lambda: terminal_text.see(tk.END))
                
            except Exception as e:
                self.root.after(0, lambda: terminal_text.insert(tk.END, f"Stop operation failed: {str(e)}\n"))
                self.root.after(0, lambda: terminal_text.see(tk.END))
        
        terminal_text.insert(tk.END, f"\n🛑 Stopping programs on {pi_connection.name}...\n")
        terminal_text.see(tk.END)
        threading.Thread(target=stop_thread, daemon=True).start()
    
    def clear_terminal(self, pi_type):
        """Clear terminal for specified Pi"""
        terminal_text = self.nav_monitor_text if pi_type == "nav" else self.comp_monitor_text
        terminal_text.delete(1.0, tk.END)
        
        pi_name = "Navigation Pi" if pi_type == "nav" else "Companion Pi"
        terminal_text.insert(tk.END, f"=== {pi_name} Terminal - {datetime.now().strftime('%H:%M:%S')} ===\n")
    
    def run_selected_script(self, pi_type):
        """Run selected Python script on specified Pi"""
        pi_connection = self.nav_pi if pi_type == "nav" else self.comp_pi
        remote_tree_widget = getattr(self, f"{pi_type}_remote_tree_widget")
        
        if not pi_connection.is_connected():
            messagebox.showwarning("Warning", f"{pi_connection.name} is not connected")
            return
        
        # Get selected file
        selection = remote_tree_widget.selection()
        if not selection:
            messagebox.showinfo("Info", "Please select a Python script to run")
            return
        
        item = selection[0]
        values = remote_tree_widget.item(item, 'values')
        is_dir = values[0] == 'True' if values else False
        file_path = values[2] if len(values) > 2 else None
        
        if is_dir or not file_path or not file_path.endswith('.py'):
            messagebox.showinfo("Info", "Please select a Python (.py) file")
            return
        
        # Run the script with real-time output
        command = f"cd ~ && python3 -u '{file_path}'"
        
        # Set the command in the appropriate command variable and execute
        if pi_type == "nav":
            self.nav_command_var.set(command)
        else:
            self.comp_command_var.set(command)
        
        self.execute_command(pi_type)

    # Event handlers and utility methods
    
    def initial_connections(self):
        """Attempt initial connections to both Pis"""
        self.general_status_label.config(text="Attempting initial connections...")
        
        # Try to connect to both Pis
        nav_success = self.nav_pi.connect()
        comp_success = self.comp_pi.connect()
        
        if nav_success and comp_success:
            self.general_status_label.config(text="Both Pis connected successfully")
            # Refresh file trees
            self.root.after(1000, lambda: self.refresh_remote_tree("nav"))
            self.root.after(1500, lambda: self.refresh_remote_tree("comp"))
        elif nav_success:
            self.general_status_label.config(text="Navigation Pi connected, Companion Pi failed")
            self.root.after(1000, lambda: self.refresh_remote_tree("nav"))
        elif comp_success:
            self.general_status_label.config(text="Companion Pi connected, Navigation Pi failed")
            self.root.after(1000, lambda: self.refresh_remote_tree("comp"))
        else:
            self.general_status_label.config(text="Both Pi connections failed - enter credentials")
    
    def connect_nav_pi(self):
        """Connect to Navigation Pi"""
        if self.nav_pi.is_connected():
            messagebox.showinfo("Info", "Navigation Pi is already connected")
            return
        
        # Get password if not cached
        if not self.password:
            password = self.get_password_dialog("Navigation Pi")
            if not password:
                return
            self.password = password
        
        def connect_thread():
            success = self.nav_pi.connect(self.password)
            if success:
                self.root.after(100, lambda: self.refresh_remote_tree("nav"))
                self.general_status_label.config(text="Navigation Pi connected")
            else:
                messagebox.showerror("Error", "Failed to connect to Navigation Pi")
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def connect_comp_pi(self):
        """Connect to Companion Pi"""
        if self.comp_pi.is_connected():
            messagebox.showinfo("Info", "Companion Pi is already connected")
            return
        
        # Get password if not cached
        if not self.password:
            password = self.get_password_dialog("Companion Pi")
            if not password:
                return
            self.password = password
        
        def connect_thread():
            success = self.comp_pi.connect(self.password)
            if success:
                self.root.after(100, lambda: self.refresh_remote_tree("comp"))
                self.general_status_label.config(text="Companion Pi connected")
            else:
                messagebox.showerror("Error", "Failed to connect to Companion Pi")
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def get_password_dialog(self, pi_name):
        """Get password from user"""
        dialog = tk.Toplevel(self.root)
        dialog.title(f"Connect to {pi_name}")
        dialog.geometry("300x150")
        dialog.transient(self.root)
        dialog.grab_set()
        
        # Center dialog
        dialog.geometry("+%d+%d" % (self.root.winfo_rootx() + 200, self.root.winfo_rooty() + 200))
        
        password = tk.StringVar()
        
        ttk.Label(dialog, text=f"Enter password for {pi_name}:").pack(pady=10)
        entry = ttk.Entry(dialog, textvariable=password, show="*", width=30)
        entry.pack(pady=5)
        entry.focus()
        
        def ok_clicked():
            dialog.destroy()
        
        def cancel_clicked():
            password.set("")
            dialog.destroy()
        
        button_frame = ttk.Frame(dialog)
        button_frame.pack(pady=10)
        
        ttk.Button(button_frame, text="OK", command=ok_clicked).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Cancel", command=cancel_clicked).pack(side='left', padx=5)
        
        # Bind Enter key
        entry.bind('<Return>', lambda e: ok_clicked())
        
        dialog.wait_window()
        return password.get()
    
    def disconnect_all(self):
        """Disconnect from all Pis"""
        self.nav_pi.disconnect()
        self.comp_pi.disconnect()
        self.general_status_label.config(text="All connections closed")
    
    def refresh_remote_tree(self, pi_type):
        """Refresh remote file tree for specified Pi"""
        pi_connection = self.nav_pi if pi_type == "nav" else self.comp_pi
        remote_tree = getattr(self, f"{pi_type}_remote_tree", None)
        remote_path_var = getattr(self, f"{pi_type}_remote_path_var", None)
        
        if not pi_connection.is_connected():
            messagebox.showwarning("Warning", f"{'Navigation' if pi_type == 'nav' else 'Companion'} Pi is not connected")
            return
        
        if remote_tree and remote_path_var:
            remote_tree.refresh_tree(remote_path_var.get())
    
    def navigate_local(self, pi_type):
        """Navigate to local path for specified Pi tab"""
        local_tree = self.local_trees.get(pi_type)
        local_path_var = getattr(self, f"{pi_type}_local_path_var", None)
        
        if local_tree and local_path_var:
            path = local_path_var.get()
            if os.path.exists(path):
                local_tree.refresh_tree(path)
            else:
                messagebox.showerror("Error", f"Path does not exist: {path}")
    
    def go_local_home(self, pi_type):
        """Navigate to local home directory for specified Pi tab"""
        local_path_var = getattr(self, f"{pi_type}_local_path_var", None)
        if local_path_var:
            local_path_var.set(str(Path.home()))
            self.navigate_local(pi_type)
    
    def quick_navigate(self, pi_type, path):
        """Quick navigation to common paths"""
        remote_path_var = getattr(self, f"{pi_type}_remote_path_var", None)
        if remote_path_var:
            remote_path_var.set(path)
            self.refresh_remote_tree(pi_type)
    
    def on_local_double_click(self, pi_type, event):
        """Handle local tree double-click"""
        tree_widget = getattr(self, f"{pi_type}_local_tree_widget")
        item = tree_widget.selection()[0] if tree_widget.selection() else None
        
        if item:
            values = tree_widget.item(item, 'values')
            is_dir = values[0] == 'True' if values else False
            path = values[2] if len(values) > 2 else None
            
            if is_dir and path:
                local_path_var = getattr(self, f"{pi_type}_local_path_var", None)
                if local_path_var:
                    local_path_var.set(path)
                    self.navigate_local(pi_type)
    
    def on_remote_double_click(self, pi_type, event):
        """Handle remote tree double-click"""
        remote_tree_widget = getattr(self, f"{pi_type}_remote_tree_widget")
        item = remote_tree_widget.selection()[0] if remote_tree_widget.selection() else None
        
        if item:
            values = remote_tree_widget.item(item, 'values')
            is_dir = values[0] == 'True' if values else False
            path = values[2] if len(values) > 2 else None
            
            if is_dir and path:
                remote_path_var = getattr(self, f"{pi_type}_remote_path_var", None)
                if remote_path_var:
                    remote_path_var.set(path)
                    self.refresh_remote_tree(pi_type)
    
    def on_local_right_click(self, pi_type, event):
        """Handle local tree right-click context menu"""
        # TODO: Implement context menu for local files
        pass
    
    def on_remote_right_click(self, pi_type, event):
        """Handle remote tree right-click context menu"""  
        # TODO: Implement context menu for remote files
        pass
    
    def upload_selected_files(self, pi_type):
        """Upload selected local files to the specified Pi"""
        pi_connection = self.nav_pi if pi_type == "nav" else self.comp_pi
        pi_name = "Navigation Pi" if pi_type == "nav" else "Companion Pi"
        
        if not pi_connection.is_connected():
            messagebox.showwarning("Warning", f"{pi_name} is not connected")
            return
        
        # Get selected local files
        tree_widget = getattr(self, f"{pi_type}_local_tree_widget")
        selection = tree_widget.selection()
        
        if not selection:
            messagebox.showinfo("Info", "Please select files to upload")
            return
        
        # Get remote destination path
        remote_path_var = getattr(self, f"{pi_type}_remote_path_var", None)
        remote_path = remote_path_var.get() if remote_path_var else "/home/jay"
        
        # Perform upload
        def upload_thread():
            try:
                dialog = FileOperationDialog(self.root, f"Uploading to {pi_name}")
                
                total_files = len(selection)
                for i, item in enumerate(selection):
                    if dialog.cancelled:
                        break
                        
                    values = tree_widget.item(item, 'values')
                    local_file_path = values[2] if len(values) > 2 else None
                    
                    if local_file_path and os.path.exists(local_file_path):
                        filename = os.path.basename(local_file_path)
                        remote_file_path = os.path.join(remote_path, filename)
                        
                        dialog.update_status(f"Uploading {filename}...")
                        
                        with SCPClient(pi_connection.ssh.get_transport()) as scp:
                            scp.put(local_file_path, remote_file_path)
                        
                        progress = ((i + 1) / total_files) * 100
                        dialog.update_progress(progress)
                
                if not dialog.cancelled:
                    dialog.update_status("Upload completed successfully!")
                    # Refresh remote tree
                    self.root.after(500, lambda: self.refresh_remote_tree(pi_type))
                
                dialog.operation_complete()
                
            except Exception as e:
                messagebox.showerror("Error", f"Upload failed: {e}")
        
        threading.Thread(target=upload_thread, daemon=True).start()
    
    def download_selected_files(self, pi_type):
        """Download selected remote files from the specified Pi"""
        pi_connection = self.nav_pi if pi_type == "nav" else self.comp_pi
        pi_name = "Navigation Pi" if pi_type == "nav" else "Companion Pi"
        
        if not pi_connection.is_connected():
            messagebox.showwarning("Warning", f"{pi_name} is not connected")
            return
        
        # Get selected remote files
        remote_tree_widget = getattr(self, f"{pi_type}_remote_tree_widget")
        selection = remote_tree_widget.selection()
        
        if not selection:
            messagebox.showinfo("Info", "Please select files to download")
            return
        
        # Get local destination path
        local_path_var = getattr(self, f"{pi_type}_local_path_var", None)
        local_path = local_path_var.get() if local_path_var else str(Path.home())
        
        # Perform download
        def download_thread():
            try:
                dialog = FileOperationDialog(self.root, f"Downloading from {pi_name}")
                
                total_files = len(selection)
                for i, item in enumerate(selection):
                    if dialog.cancelled:
                        break
                        
                    values = remote_tree_widget.item(item, 'values')
                    remote_file_path = values[2] if len(values) > 2 else None
                    
                    if remote_file_path:
                        filename = os.path.basename(remote_file_path)
                        local_file_path = os.path.join(local_path, filename)
                        
                        dialog.update_status(f"Downloading {filename}...")
                        
                        with SCPClient(pi_connection.ssh.get_transport()) as scp:
                            scp.get(remote_file_path, local_file_path)
                        
                        progress = ((i + 1) / total_files) * 100
                        dialog.update_progress(progress)
                
                if not dialog.cancelled:
                    dialog.update_status("Download completed successfully!")
                    # Refresh local tree
                    self.root.after(500, lambda: self.navigate_local(pi_type))
                
                dialog.operation_complete()
                
            except Exception as e:
                messagebox.showerror("Error", f"Download failed: {e}")
        
        threading.Thread(target=download_thread, daemon=True).start()
    
    def delete_selected_files(self, pi_type):
        """Delete selected remote files from the specified Pi"""
        pi_connection = self.nav_pi if pi_type == "nav" else self.comp_pi
        pi_name = "Navigation Pi" if pi_type == "nav" else "Companion Pi"
        
        if not pi_connection.is_connected():
            messagebox.showwarning("Warning", f"{pi_name} is not connected")
            return
        
        # Get selected remote files
        remote_tree_widget = getattr(self, f"{pi_type}_remote_tree_widget")
        selection = remote_tree_widget.selection()
        
        if not selection:
            messagebox.showinfo("Info", "Please select files to delete")
            return
        
        # Confirm deletion
        file_count = len(selection)
        if not messagebox.askyesno("Confirm Delete", 
                                 f"Are you sure you want to delete {file_count} file(s) from {pi_name}?"):
            return
        
        # Perform deletion
        def delete_thread():
            try:
                for item in selection:
                    values = remote_tree_widget.item(item, 'values')
                    remote_file_path = values[2] if len(values) > 2 else None
                    
                    if remote_file_path:
                        # Execute rm command
                        stdin, stdout, stderr = pi_connection.ssh.exec_command(f"rm -rf '{remote_file_path}'")
                        
                        # Check for errors
                        error_output = stderr.read().decode().strip()
                        if error_output:
                            print(f"Delete warning for {remote_file_path}: {error_output}")
                
                # Refresh remote tree
                self.root.after(500, lambda: self.refresh_remote_tree(pi_type))
                messagebox.showinfo("Success", f"Files deleted from {pi_name}")
                
            except Exception as e:
                messagebox.showerror("Error", f"Delete failed: {e}")
        
        threading.Thread(target=delete_thread, daemon=True).start()
    
    def update_status_thread(self):
        """Update status bar continuously"""
        while True:
            try:
                # Update connection status
                if self.nav_pi.is_connected():
                    self.nav_status_label.config(text="📡 Navigation Pi: Connected ✅")
                    self.nav_connect_btn.config(text="Disconnect")
                else:
                    self.nav_status_label.config(text="📡 Navigation Pi: Disconnected ❌")
                    self.nav_connect_btn.config(text="Connect")
                
                if self.comp_pi.is_connected():
                    self.comp_status_label.config(text="🎨 Companion Pi: Connected ✅")
                    self.comp_connect_btn.config(text="Disconnect")
                else:
                    self.comp_status_label.config(text="🎨 Companion Pi: Disconnected ❌")
                    self.comp_connect_btn.config(text="Connect")
                
                time.sleep(2)
                
            except Exception as e:
                print(f"Status update error: {e}")
                time.sleep(5)
    
    def system_health_check(self):
        """Perform comprehensive system health check on both Pis"""
        def health_check_thread():
            try:
                health_report = f"=== System Health Check - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n\n"
                
                # Check Navigation Pi
                if self.nav_pi.is_connected():
                    health_report += "📡 NAVIGATION PI HEALTH:\n"
                    
                    # Check key services and hardware
                    checks = [
                        ("Arduino Connection", "ls -la /dev/ttyUSB*"),
                        ("WiFi Status", "iwconfig"),
                        ("Temperature", "vcgencmd measure_temp"),
                        ("Voltage", "vcgencmd measure_volts core")
                    ]
                    
                    for check_name, command in checks:
                        try:
                            stdin, stdout, stderr = self.nav_pi.ssh.exec_command(command)
                            output = stdout.read().decode().strip()
                            error = stderr.read().decode().strip()
                            
                            status = "✅ OK" if not error else "⚠️ Warning"
                            health_report += f"  {check_name}: {status}\n"
                            if error:
                                health_report += f"    Error: {error}\n"
                            elif output:
                                health_report += f"    {output[:100]}\n"
                        except:
                            health_report += f"  {check_name}: ❌ Failed\n"
                else:
                    health_report += "📡 NAVIGATION PI: ❌ Not Connected\n"
                
                health_report += "\n"
                
                # Check Companion Pi
                if self.comp_pi.is_connected():
                    health_report += "🎨 COMPANION PI HEALTH:\n"
                    
                    # Check key services
                    checks = [
                        ("Display Server", "echo $DISPLAY"),
                        ("WiFi Status", "iwconfig"),
                        ("Temperature", "vcgencmd measure_temp"),
                        ("Voltage", "vcgencmd measure_volts core")
                    ]
                    
                    for check_name, command in checks:
                        try:
                            stdin, stdout, stderr = self.comp_pi.ssh.exec_command(command)
                            output = stdout.read().decode().strip()
                            error = stderr.read().decode().strip()
                            
                            status = "✅ OK" if not error else "⚠️ Warning"
                            health_report += f"  {check_name}: {status}\n"
                            if error:
                                health_report += f"    Error: {error}\n"
                            elif output:
                                health_report += f"    {output[:100]}\n"
                        except:
                            health_report += f"  {check_name}: ❌ Failed\n"
                else:
                    health_report += "🎨 COMPANION PI: ❌ Not Connected\n"
                
                # Show results in both monitor windows
                self.nav_monitor_text.delete(1.0, 'end')
                self.nav_monitor_text.insert(1.0, health_report)
                
                self.comp_monitor_text.delete(1.0, 'end')
                self.comp_monitor_text.insert(1.0, health_report)
                
                messagebox.showinfo("Health Check", "System health check completed. See monitoring tabs for details.")
                
            except Exception as e:
                messagebox.showerror("Error", f"Health check failed: {e}")
        
        threading.Thread(target=health_check_thread, daemon=True).start()
    
    def apply_settings(self):
        """Apply connection settings"""
        # Update Pi connection settings
        self.nav_pi.host = self.nav_host_var.get()
        self.comp_pi.host = self.comp_host_var.get()
        
        # Disconnect existing connections so they can reconnect with new settings
        self.disconnect_all()
        
        messagebox.showinfo("Settings", "Settings applied. Please reconnect to use new settings.")
    
    def open_terminal(self):
        """Open system terminal"""
        try:
            subprocess.Popen(['gnome-terminal'])
        except:
            try:
                subprocess.Popen(['xterm'])
            except:
                messagebox.showerror("Error", "Could not open terminal")
    
    def show_system_info(self):
        """Show system information"""
        info = f"""
Rover SSH Dashboard V3.7 - Real-time Terminal Fixed Edition

Current Connections:
📡 Navigation Pi ({self.nav_pi.host}): {"Connected" if self.nav_pi.is_connected() else "Disconnected"}
🎨 Companion Pi ({self.comp_pi.host}): {"Connected" if self.comp_pi.is_connected() else "Disconnected"}

System Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
        """
        
        messagebox.showinfo("System Information", info)
    
    def show_about(self):
        """Show about dialog"""
        about_text = """
Rover SSH Dashboard V3.7
Real-time Terminal Fixed Edition

Key Features:
• Separate dedicated tabs for Navigation Pi and Companion Pi
• Independent file management for each Pi system
• REAL-TIME terminal updates with channel-based reading
• Parallel upload/download operations
• Pi-specific quick navigation shortcuts
• Dual Pi system monitoring and health checks
• Fixed terminal buffering issues using PTY allocation

Perfect for dual Pi rover development with live debugging!

Version: 3.7 - Real-time Terminal Fixed Edition
Author: Claude Code Assistant
        """
        
        messagebox.showinfo("About", about_text)


def main():
    """Main entry point"""
    root = tk.Tk()
    app = EnhancedSSHDashboard(root)
    
    # Handle window closing
    def on_closing():
        app.disconnect_all()
        root.quit()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\nShutting down dashboard...")
        app.disconnect_all()


if __name__ == "__main__":
    main()