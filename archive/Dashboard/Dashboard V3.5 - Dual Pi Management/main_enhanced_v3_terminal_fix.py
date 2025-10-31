#!/usr/bin/env python3
"""
Raspberry Pi SSH Dashboard V3.5 - Terminal Fix Edition
=======================================================

Conservative fix for terminal real-time output based on working Dashboard V2
Only modifies terminal execution - keeps all existing functionality intact

TERMINAL FIX: Real-time output using channel-based reading with PTY allocation

Author: Claude Code Assistant
Version: 3.5 - Terminal Fix Edition
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


class RemoteFileTree:
    """Remote file tree browser for rover files"""
    
    def __init__(self, ssh_client, tree_widget):
        self.ssh = ssh_client
        self.tree = tree_widget
        self.file_cache = {}  # Cache for file listings
        
    def refresh_tree(self, path="/home/jay"):
        """Refresh the remote file tree"""
        try:
            if not self.ssh or not self.ssh.get_transport() or not self.ssh.get_transport().is_active():
                print("SSH connection not active")
                return
                
            print(f"RemoteFileTree: Refreshing tree for path: {path}")
            
            # Clear existing tree
            for item in self.tree.get_children():
                self.tree.delete(item)
            
            # Build tree starting from root path
            self._build_tree_node("", path)
            print(f"RemoteFileTree: Successfully refreshed tree")
            
        except Exception as e:
            print(f"Error refreshing remote tree: {e}")
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
            print(f"Error building tree node: {e}")


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


class SSHConnection:
    """Manages SSH connection with improved stability"""
    
    def __init__(self, name, host, port=22, username="jay"):
        self.name = name
        self.host = host
        self.port = port
        self.username = username
        self.ssh_client = None
        self.connected = False
        self.current_channel = None  # Track current execution channel
        
    def connect(self, password=None):
        """Connect to the host"""
        try:
            print(f"Connecting to {self.name} ({self.host})...")
            
            self.ssh_client = paramiko.SSHClient()
            self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            
            # Try to connect
            self.ssh_client.connect(self.host, port=self.port, username=self.username, 
                                  password=password, timeout=10, allow_agent=False, look_for_keys=False)
            
            self.connected = True
            print(f"✅ Connected to {self.name}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to connect to {self.name}: {e}")
            self.connected = False
            if self.ssh_client:
                try:
                    self.ssh_client.close()
                except:
                    pass
                self.ssh_client = None
            return False
    
    def disconnect(self):
        """Disconnect from the host"""
        try:
            if self.current_channel:
                try:
                    self.current_channel.close()
                except:
                    pass
                self.current_channel = None
                
            if self.ssh_client:
                self.ssh_client.close()
                self.ssh_client = None
                
            self.connected = False
            print(f"Disconnected from {self.name}")
            
        except Exception as e:
            print(f"Error disconnecting from {self.name}: {e}")
    
    def is_connected(self):
        """Check if connection is active"""
        if not self.connected or not self.ssh_client:
            return False
            
        try:
            # Test connection with a simple command
            transport = self.ssh_client.get_transport()
            return transport and transport.is_active()
        except:
            return False


class EnhancedSSHDashboard:
    """Enhanced SSH Dashboard with terminal fix"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Rover SSH Dashboard V3.5 - Terminal Fix Edition")
        self.root.geometry("1400x900")
        
        # SSH connections
        self.nav_connection = SSHConnection("Navigation Pi", "192.168.254.65")
        self.comp_connection = SSHConnection("Companion Pi", "192.168.254.70")
        self.current_connection = None  # Currently selected connection
        
        # GUI state
        self.dark_mode = False
        self.password = None
        
        # Setup GUI
        self.setup_styles()
        self.setup_menu()
        self.setup_main_interface()
        
    def setup_styles(self):
        """Setup ttk styles"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # Configure custom styles
        style.configure('Title.TLabel', font=('Arial', 12, 'bold'))
        style.configure('Status.TLabel', font=('Arial', 9))
        
    def setup_menu(self):
        """Setup menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        # Connection menu
        conn_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Connection", menu=conn_menu)
        conn_menu.add_command(label="Connect to Navigation Pi", command=self.connect_nav_pi)
        conn_menu.add_command(label="Connect to Companion Pi", command=self.connect_comp_pi)
        conn_menu.add_separator()
        conn_menu.add_command(label="Disconnect All", command=self.disconnect_all)
        conn_menu.add_separator()
        conn_menu.add_command(label="Exit", command=self.root.quit)
        
        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)
    
    def setup_main_interface(self):
        """Setup the main interface"""
        # Connection frame
        self.setup_connection_frame()
        
        # Main notebook
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Setup tabs
        self.setup_file_management_tab()
        self.setup_terminal_tab()
        self.setup_system_monitoring_tab()
        
        # Status bar
        self.setup_status_bar()
    
    def setup_connection_frame(self):
        """Setup connection management frame"""
        conn_frame = ttk.LabelFrame(self.root, text="Connection Management", padding="5")
        conn_frame.pack(fill='x', padx=5, pady=5)
        
        # Pi selection
        ttk.Label(conn_frame, text="Select Pi:").grid(row=0, column=0, sticky='w', padx=(0, 5))
        
        self.pi_selection = ttk.Combobox(conn_frame, values=["Navigation Pi (192.168.254.65)", "Companion Pi (192.168.254.70)"], 
                                        state="readonly", width=30)
        self.pi_selection.grid(row=0, column=1, padx=(0, 10))
        self.pi_selection.current(0)
        self.pi_selection.bind('<<ComboboxSelected>>', self.on_pi_selection_changed)
        
        # Connect button
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.connect_selected_pi)
        self.connect_btn.grid(row=0, column=2, padx=(0, 10))
        
        # Disconnect button
        self.disconnect_btn = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect_selected_pi, state='disabled')
        self.disconnect_btn.grid(row=0, column=3, padx=(0, 10))
        
        # Status
        self.connection_status = ttk.Label(conn_frame, text="Not connected", foreground='red')
        self.connection_status.grid(row=0, column=4, padx=(10, 0))
        
        # Update current connection
        self.update_current_connection()
    
    def setup_file_management_tab(self):
        """Setup file management tab"""
        file_frame = ttk.Frame(self.notebook)
        self.notebook.add(file_frame, text="📁 File Management")
        
        # Create main paned window for dual-pane layout
        main_paned = ttk.PanedWindow(file_frame, orient='horizontal')
        main_paned.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Left pane - Local files
        self.setup_local_file_pane(main_paned)
        
        # Right pane - Remote files  
        self.setup_remote_file_pane(main_paned)
        
        # Bottom pane - File operations
        self.setup_file_operations_pane(file_frame)
    
    def setup_local_file_pane(self, parent):
        """Setup local file browser pane"""
        local_frame = ttk.LabelFrame(parent, text="🖥️ Local Files", padding="5")
        parent.add(local_frame, weight=1)
        
        # Path navigation
        nav_frame = ttk.Frame(local_frame)
        nav_frame.pack(fill='x', pady=(0, 5))
        
        ttk.Label(nav_frame, text="Path:").pack(side='left')
        self.local_path_var = tk.StringVar(value=str(Path.home()))
        self.local_path_entry = ttk.Entry(nav_frame, textvariable=self.local_path_var)
        self.local_path_entry.pack(side='left', fill='x', expand=True, padx=(5, 5))
        
        ttk.Button(nav_frame, text="Go", command=self.navigate_local).pack(side='right')
        ttk.Button(nav_frame, text="Home", command=self.go_local_home).pack(side='right', padx=(0, 5))
        
        # File tree
        tree_frame = ttk.Frame(local_frame)
        tree_frame.pack(fill='both', expand=True)
        
        self.local_tree = ttk.Treeview(tree_frame, columns=('is_dir', 'size', 'path'), show='tree')
        self.local_tree.heading('#0', text='File Name', anchor='w')
        self.local_tree.column('#0', width=300, minwidth=200)
        self.local_tree.column('is_dir', width=0, minwidth=0, stretch=False)  # Hidden
        self.local_tree.column('size', width=120, minwidth=80)
        self.local_tree.column('path', width=0, minwidth=0, stretch=False)  # Hidden
        
        # Scrollbars
        local_v_scroll = ttk.Scrollbar(tree_frame, orient='vertical', command=self.local_tree.yview)
        local_h_scroll = ttk.Scrollbar(tree_frame, orient='horizontal', command=self.local_tree.xview)
        self.local_tree.configure(yscrollcommand=local_v_scroll.set, xscrollcommand=local_h_scroll.set)
        
        self.local_tree.pack(side='left', fill='both', expand=True)
        local_v_scroll.pack(side='right', fill='y')
        local_h_scroll.pack(side='bottom', fill='x')
        
        # Bind events
        self.local_tree.bind('<Double-1>', self.on_local_double_click)
        
        # Initialize local tree
        self.local_file_tree = LocalFileTree(self.local_tree)
        self.local_file_tree.refresh_tree()
    
    def setup_remote_file_pane(self, parent):
        """Setup remote file browser pane"""
        remote_frame = ttk.LabelFrame(parent, text="🤖 Remote Files", padding="5")
        parent.add(remote_frame, weight=1)
        
        # Path navigation
        nav_frame = ttk.Frame(remote_frame)
        nav_frame.pack(fill='x', pady=(0, 5))
        
        ttk.Label(nav_frame, text="Path:").pack(side='left')
        self.remote_path_var = tk.StringVar(value="/home/jay")
        self.remote_path_entry = ttk.Entry(nav_frame, textvariable=self.remote_path_var)
        self.remote_path_entry.pack(side='left', fill='x', expand=True, padx=(5, 5))
        
        ttk.Button(nav_frame, text="Refresh", command=self.refresh_remote_tree).pack(side='right')
        
        # File tree
        tree_frame = ttk.Frame(remote_frame)
        tree_frame.pack(fill='both', expand=True)
        
        self.remote_tree_widget = ttk.Treeview(tree_frame, columns=('is_dir', 'size', 'path'), show='tree')
        self.remote_tree_widget.heading('#0', text='File Name', anchor='w')
        self.remote_tree_widget.column('#0', width=300, minwidth=200)
        self.remote_tree_widget.column('is_dir', width=0, minwidth=0, stretch=False)  # Hidden
        self.remote_tree_widget.column('size', width=120, minwidth=80)
        self.remote_tree_widget.column('path', width=0, minwidth=0, stretch=False)  # Hidden
        
        # Scrollbars
        remote_v_scroll = ttk.Scrollbar(tree_frame, orient='vertical', command=self.remote_tree_widget.yview)
        remote_h_scroll = ttk.Scrollbar(tree_frame, orient='horizontal', command=self.remote_tree_widget.xview)
        self.remote_tree_widget.configure(yscrollcommand=remote_v_scroll.set, xscrollcommand=remote_h_scroll.set)
        
        self.remote_tree_widget.pack(side='left', fill='both', expand=True)
        remote_v_scroll.pack(side='right', fill='y')
        remote_h_scroll.pack(side='bottom', fill='x')
        
        # Bind events
        self.remote_tree_widget.bind('<Double-1>', self.on_remote_double_click)
    
    def setup_file_operations_pane(self, parent):
        """Setup file operations panel"""
        ops_frame = ttk.LabelFrame(parent, text="🚀 File Operations", padding="5")
        ops_frame.pack(fill='x', padx=5, pady=(0, 5))
        
        # File operation buttons
        button_frame = ttk.Frame(ops_frame)
        button_frame.pack(fill='x', pady=5)
        
        ttk.Button(button_frame, text="📤 Upload Selected", command=self.upload_selected_files).pack(side='left', padx=2)
        ttk.Button(button_frame, text="📥 Download Selected", command=self.download_selected_files).pack(side='left', padx=2)
        ttk.Button(button_frame, text="🗑️ Delete Selected", command=self.delete_selected_files).pack(side='left', padx=2)
        ttk.Button(button_frame, text="▶️ Run Script", command=self.run_selected_script).pack(side='left', padx=2)
    
    def setup_terminal_tab(self):
        """Setup terminal tab with REAL-TIME output"""
        terminal_frame = ttk.Frame(self.notebook)
        self.notebook.add(terminal_frame, text="💻 Terminal")
        
        # Terminal output with fixed colors for better readability
        self.terminal_output = scrolledtext.ScrolledText(
            terminal_frame, height=25, width=80,
            bg='black', fg='green', insertbackground='green',
            font=('Courier', 10)
        )
        self.terminal_output.pack(fill='both', expand=True, padx=5, pady=(5, 5))
        
        # Command entry frame
        cmd_frame = ttk.Frame(terminal_frame)
        cmd_frame.pack(fill='x', padx=5, pady=(0, 5))
        
        ttk.Label(cmd_frame, text="Command:").pack(side='left')
        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(cmd_frame, textvariable=self.command_var)
        self.command_entry.pack(side='left', fill='x', expand=True, padx=(5, 5))
        self.command_entry.bind('<Return>', self.execute_command)
        
        ttk.Button(cmd_frame, text="Execute", command=self.execute_command).pack(side='right', padx=(0, 5))
        ttk.Button(cmd_frame, text="Stop", command=self.stop_remote_program).pack(side='right', padx=(0, 5))
        ttk.Button(cmd_frame, text="Clear", command=self.clear_terminal).pack(side='right', padx=(0, 5))
    
    def setup_system_monitoring_tab(self):
        """Setup system monitoring tab"""
        monitoring_frame = ttk.Frame(self.notebook)
        self.notebook.add(monitoring_frame, text="📊 System Monitoring")
        
        # System info display
        self.system_info_text = scrolledtext.ScrolledText(monitoring_frame, height=30, width=80)
        self.system_info_text.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Control buttons
        control_frame = ttk.Frame(monitoring_frame)
        control_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Button(control_frame, text="🔄 Refresh System Status", command=self.refresh_system_status).pack(side='left', padx=2)
        ttk.Button(control_frame, text="📊 Health Check", command=self.system_health_check).pack(side='left', padx=2)
    
    def setup_status_bar(self):
        """Setup status bar"""
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill='x', side='bottom', padx=5, pady=2)
        
        self.status_label = ttk.Label(status_frame, text="Ready", style='Status.TLabel')
        self.status_label.pack(side='left')
    
    # Connection management methods
    
    def on_pi_selection_changed(self, event=None):
        """Handle Pi selection change"""
        self.update_current_connection()
        self.update_connection_status()
    
    def update_current_connection(self):
        """Update current connection based on selection"""
        selection = self.pi_selection.current()
        if selection == 0:
            self.current_connection = self.nav_connection
        else:
            self.current_connection = self.comp_connection
    
    def update_connection_status(self):
        """Update connection status display"""
        if self.current_connection and self.current_connection.is_connected():
            self.connection_status.config(text=f"Connected to {self.current_connection.name}", foreground='green')
            self.connect_btn.config(state='disabled')
            self.disconnect_btn.config(state='normal')
        else:
            self.connection_status.config(text="Not connected", foreground='red')
            self.connect_btn.config(state='normal')
            self.disconnect_btn.config(state='disabled')
    
    def connect_nav_pi(self):
        """Connect to Navigation Pi"""
        self.pi_selection.current(0)
        self.update_current_connection()
        self.connect_selected_pi()
    
    def connect_comp_pi(self):
        """Connect to Companion Pi"""
        self.pi_selection.current(1)
        self.update_current_connection()
        self.connect_selected_pi()
    
    def connect_selected_pi(self):
        """Connect to currently selected Pi"""
        if not self.current_connection:
            return
            
        if self.current_connection.is_connected():
            messagebox.showinfo("Info", f"{self.current_connection.name} is already connected")
            return
        
        # Get password if not cached
        if not self.password:
            password = self.get_password_dialog(self.current_connection.name)
            if not password:
                return
            self.password = password
        
        def connect_thread():
            success = self.current_connection.connect(self.password)
            self.root.after(100, lambda: self.on_connection_result(success))
        
        self.status_label.config(text=f"Connecting to {self.current_connection.name}...")
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def on_connection_result(self, success):
        """Handle connection result"""
        if success:
            self.status_label.config(text=f"Connected to {self.current_connection.name}")
            self.terminal_output.insert(tk.END, f"✅ Connected to {self.current_connection.name}\n")
            self.terminal_output.see(tk.END)
            
            # Initialize remote tree
            if hasattr(self, 'remote_tree_widget'):
                self.remote_tree = RemoteFileTree(self.current_connection.ssh_client, self.remote_tree_widget)
                self.refresh_remote_tree()
        else:
            self.status_label.config(text="Connection failed")
            messagebox.showerror("Error", f"Failed to connect to {self.current_connection.name}")
        
        self.update_connection_status()
    
    def disconnect_selected_pi(self):
        """Disconnect from currently selected Pi"""
        if self.current_connection and self.current_connection.is_connected():
            self.current_connection.disconnect()
            self.status_label.config(text=f"Disconnected from {self.current_connection.name}")
            self.terminal_output.insert(tk.END, f"❌ Disconnected from {self.current_connection.name}\n")
            self.terminal_output.see(tk.END)
        
        self.update_connection_status()
    
    def disconnect_all(self):
        """Disconnect from all Pis"""
        self.nav_connection.disconnect()
        self.comp_connection.disconnect()
        self.status_label.config(text="All connections closed")
        self.update_connection_status()
    
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

    # TERMINAL EXECUTION WITH REAL-TIME FIX
    def execute_command(self, event=None):
        """Execute command with REAL-TIME output - THE FIX!"""
        command = self.command_var.get().strip()
        if not command:
            return
            
        if not self.current_connection or not self.current_connection.is_connected():
            self.terminal_output.insert(tk.END, "❌ No active connection\n")
            self.terminal_output.see(tk.END)
            return
        
        # Clear command entry
        self.command_var.set("")
        
        def execute_thread():
            try:
                # Insert command prompt
                self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"$ {command}\n"))
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
                
                # Execute command with PTY (CRITICAL FOR REAL-TIME OUTPUT)
                stdin, stdout, stderr = self.current_connection.ssh_client.exec_command(command, get_pty=True)
                self.current_connection.current_channel = stdout.channel
                
                # Set non-blocking mode for REAL-TIME output (THE KEY FIX!)
                stdout.channel.settimeout(0.1)
                stderr.channel.settimeout(0.1)
                
                # Real-time output reading loop
                while not stdout.channel.exit_status_ready():
                    try:
                        # Check for stdout data
                        if stdout.channel.recv_ready():
                            data = stdout.channel.recv(1024).decode('utf-8', errors='replace')
                            if data:
                                # Immediate GUI update using root.after(0, ...)
                                self.root.after(0, lambda d=data: self.terminal_output.insert(tk.END, d))
                                self.root.after(0, lambda: self.terminal_output.see(tk.END))
                        
                        # Check for stderr data
                        if stderr.channel.recv_stderr_ready():
                            error_data = stderr.channel.recv_stderr(1024).decode('utf-8', errors='replace')
                            if error_data:
                                self.root.after(0, lambda d=error_data: self.terminal_output.insert(tk.END, f"[ERROR] {d}"))
                                self.root.after(0, lambda: self.terminal_output.see(tk.END))
                        
                        # Small delay to prevent excessive CPU usage
                        time.sleep(0.01)
                        
                    except Exception as inner_e:
                        # Handle any inner loop exceptions gracefully
                        print(f"Inner loop exception: {inner_e}")
                        break
                
                # Read any remaining output
                try:
                    remaining_out = stdout.read().decode('utf-8', errors='replace')
                    if remaining_out:
                        self.root.after(0, lambda d=remaining_out: self.terminal_output.insert(tk.END, d))
                
                    remaining_err = stderr.read().decode('utf-8', errors='replace')
                    if remaining_err:
                        self.root.after(0, lambda d=remaining_err: self.terminal_output.insert(tk.END, f"[ERROR] {d}"))
                        
                    self.root.after(0, lambda: self.terminal_output.see(tk.END))
                except:
                    pass  # Ignore remaining output errors
                
            except Exception as e:
                error_msg = str(e)
                self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"Command failed: {error_msg}\n"))
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
        
        # Execute in background thread
        threading.Thread(target=execute_thread, daemon=True).start()
    
    def stop_remote_program(self):
        """Stop running program"""
        if not self.current_connection or not self.current_connection.is_connected():
            self.terminal_output.insert(tk.END, "❌ No active connection\n")
            self.terminal_output.see(tk.END)
            return
        
        def stop_thread():
            try:
                # Strategy 1: Send Ctrl+C to current channel
                if self.current_connection.current_channel and not self.current_connection.current_channel.closed:
                    try:
                        self.current_connection.current_channel.send('\x03')  # Ctrl+C character
                        self.root.after(0, lambda: self.terminal_output.insert(tk.END, "\n^C (Interrupt signal sent)\n"))
                    except:
                        pass
                
                # Strategy 2: Kill Python processes
                try:
                    stdin, stdout, stderr = self.current_connection.ssh_client.exec_command("pkill -SIGTERM -f 'python.*\\.py'")
                    self.root.after(0, lambda: self.terminal_output.insert(tk.END, "SIGTERM sent to Python processes\n"))
                except:
                    pass
                
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
                
            except Exception as e:
                self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"Stop failed: {str(e)}\n"))
        
        self.terminal_output.insert(tk.END, f"\n🛑 Stopping programs on {self.current_connection.name}...\n")
        self.terminal_output.see(tk.END)
        threading.Thread(target=stop_thread, daemon=True).start()
    
    def clear_terminal(self):
        """Clear terminal output"""
        self.terminal_output.delete(1.0, tk.END)
    
    # File management methods
    
    def navigate_local(self):
        """Navigate to local path"""
        path = self.local_path_var.get()
        if os.path.exists(path):
            self.local_file_tree.refresh_tree(path)
        else:
            messagebox.showerror("Error", f"Path does not exist: {path}")
    
    def go_local_home(self):
        """Navigate to local home directory"""
        self.local_path_var.set(str(Path.home()))
        self.navigate_local()
    
    def refresh_remote_tree(self):
        """Refresh remote file tree"""
        if not self.current_connection or not self.current_connection.is_connected():
            messagebox.showwarning("Warning", "No active connection")
            return
        
        if hasattr(self, 'remote_tree'):
            self.remote_tree.refresh_tree(self.remote_path_var.get())
    
    def on_local_double_click(self, event):
        """Handle local tree double-click"""
        item = self.local_tree.selection()[0] if self.local_tree.selection() else None
        
        if item:
            values = self.local_tree.item(item, 'values')
            is_dir = values[0] == 'True' if values else False
            path = values[2] if len(values) > 2 else None
            
            if is_dir and path:
                self.local_path_var.set(path)
                self.navigate_local()
    
    def on_remote_double_click(self, event):
        """Handle remote tree double-click"""
        item = self.remote_tree_widget.selection()[0] if self.remote_tree_widget.selection() else None
        
        if item:
            values = self.remote_tree_widget.item(item, 'values')
            is_dir = values[0] == 'True' if values else False
            path = values[2] if len(values) > 2 else None
            
            if is_dir and path:
                self.remote_path_var.set(path)
                self.refresh_remote_tree()
    
    def upload_selected_files(self):
        """Upload selected local files"""
        if not self.current_connection or not self.current_connection.is_connected():
            messagebox.showwarning("Warning", "No active connection")
            return
        
        selection = self.local_tree.selection()
        if not selection:
            messagebox.showinfo("Info", "Please select files to upload")
            return
        
        remote_path = self.remote_path_var.get()
        
        def upload_thread():
            try:
                with SCPClient(self.current_connection.ssh_client.get_transport()) as scp:
                    for item in selection:
                        values = self.local_tree.item(item, 'values')
                        local_file_path = values[2] if len(values) > 2 else None
                        
                        if local_file_path and os.path.exists(local_file_path):
                            filename = os.path.basename(local_file_path)
                            remote_file_path = os.path.join(remote_path, filename)
                            scp.put(local_file_path, remote_file_path)
                            
                            self.root.after(0, lambda f=filename: self.terminal_output.insert(tk.END, f"✅ Uploaded {f}\n"))
                            self.root.after(0, lambda: self.terminal_output.see(tk.END))
                
                # Refresh remote tree
                self.root.after(500, self.refresh_remote_tree)
                
            except Exception as e:
                self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"❌ Upload failed: {e}\n"))
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
        
        threading.Thread(target=upload_thread, daemon=True).start()
    
    def download_selected_files(self):
        """Download selected remote files"""
        if not self.current_connection or not self.current_connection.is_connected():
            messagebox.showwarning("Warning", "No active connection")
            return
        
        selection = self.remote_tree_widget.selection()
        if not selection:
            messagebox.showinfo("Info", "Please select files to download")
            return
        
        local_path = self.local_path_var.get()
        
        def download_thread():
            try:
                with SCPClient(self.current_connection.ssh_client.get_transport()) as scp:
                    for item in selection:
                        values = self.remote_tree_widget.item(item, 'values')
                        remote_file_path = values[2] if len(values) > 2 else None
                        
                        if remote_file_path:
                            filename = os.path.basename(remote_file_path)
                            local_file_path = os.path.join(local_path, filename)
                            scp.get(remote_file_path, local_file_path)
                            
                            self.root.after(0, lambda f=filename: self.terminal_output.insert(tk.END, f"✅ Downloaded {f}\n"))
                            self.root.after(0, lambda: self.terminal_output.see(tk.END))
                
                # Refresh local tree
                self.root.after(500, self.navigate_local)
                
            except Exception as e:
                self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"❌ Download failed: {e}\n"))
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
        
        threading.Thread(target=download_thread, daemon=True).start()
    
    def delete_selected_files(self):
        """Delete selected remote files"""
        if not self.current_connection or not self.current_connection.is_connected():
            messagebox.showwarning("Warning", "No active connection")
            return
        
        selection = self.remote_tree_widget.selection()
        if not selection:
            messagebox.showinfo("Info", "Please select files to delete")
            return
        
        if not messagebox.askyesno("Confirm Delete", f"Delete {len(selection)} file(s)?"):
            return
        
        def delete_thread():
            try:
                for item in selection:
                    values = self.remote_tree_widget.item(item, 'values')
                    remote_file_path = values[2] if len(values) > 2 else None
                    
                    if remote_file_path:
                        stdin, stdout, stderr = self.current_connection.ssh_client.exec_command(f"rm -rf '{remote_file_path}'")
                        filename = os.path.basename(remote_file_path)
                        self.root.after(0, lambda f=filename: self.terminal_output.insert(tk.END, f"🗑️ Deleted {f}\n"))
                        self.root.after(0, lambda: self.terminal_output.see(tk.END))
                
                # Refresh remote tree
                self.root.after(500, self.refresh_remote_tree)
                
            except Exception as e:
                self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"❌ Delete failed: {e}\n"))
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
        
        threading.Thread(target=delete_thread, daemon=True).start()
    
    def run_selected_script(self):
        """Run selected Python script"""
        if not self.current_connection or not self.current_connection.is_connected():
            messagebox.showwarning("Warning", "No active connection")
            return
        
        selection = self.remote_tree_widget.selection()
        if not selection:
            messagebox.showinfo("Info", "Please select a Python script to run")
            return
        
        item = selection[0]
        values = self.remote_tree_widget.item(item, 'values')
        is_dir = values[0] == 'True' if values else False
        file_path = values[2] if len(values) > 2 else None
        
        if is_dir or not file_path or not file_path.endswith('.py'):
            messagebox.showinfo("Info", "Please select a Python (.py) file")
            return
        
        # Run the script with real-time output
        command = f"cd ~ && python3 -u '{file_path}'"
        self.command_var.set(command)
        self.execute_command()
    
    # System monitoring methods
    
    def refresh_system_status(self):
        """Refresh system status information"""
        if not self.current_connection or not self.current_connection.is_connected():
            messagebox.showwarning("Warning", "No active connection")
            return
        
        def status_thread():
            try:
                commands = [
                    ("System Info", "uname -a"),
                    ("Disk Usage", "df -h"),
                    ("Memory Usage", "free -h"),
                    ("CPU Info", "top -bn1 | head -n 15"),
                    ("Network", "ip addr show")
                ]
                
                status_text = f"=== {self.current_connection.name} Status - {datetime.now().strftime('%H:%M:%S')} ===\n\n"
                
                for title, command in commands:
                    stdin, stdout, stderr = self.current_connection.ssh_client.exec_command(command)
                    output = stdout.read().decode()
                    status_text += f"--- {title} ---\n{output}\n\n"
                
                self.system_info_text.delete(1.0, 'end')
                self.system_info_text.insert(1.0, status_text)
                
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("Error", f"Status refresh failed: {e}"))
        
        threading.Thread(target=status_thread, daemon=True).start()
    
    def system_health_check(self):
        """Perform system health check"""
        if not self.current_connection or not self.current_connection.is_connected():
            messagebox.showwarning("Warning", "No active connection")
            return
        
        def health_thread():
            try:
                checks = [
                    ("Temperature", "vcgencmd measure_temp"),
                    ("Voltage", "vcgencmd measure_volts core"),
                    ("WiFi", "iwconfig"),
                    ("USB Devices", "lsusb")
                ]
                
                health_text = f"=== {self.current_connection.name} Health Check ===\n\n"
                
                for check_name, command in checks:
                    stdin, stdout, stderr = self.current_connection.ssh_client.exec_command(command)
                    output = stdout.read().decode().strip()
                    error = stderr.read().decode().strip()
                    
                    status = "✅ OK" if not error else "⚠️ Warning"
                    health_text += f"{check_name}: {status}\n"
                    if output:
                        health_text += f"  {output}\n"
                    if error:
                        health_text += f"  Error: {error}\n"
                    health_text += "\n"
                
                self.system_info_text.delete(1.0, 'end')
                self.system_info_text.insert(1.0, health_text)
                
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("Error", f"Health check failed: {e}"))
        
        threading.Thread(target=health_thread, daemon=True).start()
    
    def show_about(self):
        """Show about dialog"""
        about_text = """
Rover SSH Dashboard V3.5 - Terminal Fix Edition

Key Features:
• Dual Pi connection management
• Real-time terminal with channel-based reading
• File management with upload/download
• System monitoring and health checks
• Python script execution with live output

Terminal Fix Applied:
• PTY allocation with get_pty=True
• Non-blocking channel reading
• Real-time output polling
• Immediate GUI updates

Perfect for rover development with live debugging!

Version: 3.5 - Terminal Fix Edition
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