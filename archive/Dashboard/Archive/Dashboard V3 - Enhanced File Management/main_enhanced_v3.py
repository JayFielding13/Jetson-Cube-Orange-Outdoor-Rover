#!/usr/bin/env python3
"""
Raspberry Pi SSH Dashboard V3 - Enhanced File Management Edition
===============================================================

Advanced SSH dashboard with comprehensive file management features:
- Dual-pane file browser (local/remote)
- File tree navigation with context menus
- Drag & drop uploads
- Progress tracking and batch operations
- Smart deployment buttons
- Enhanced terminal integration

Features:
- 🌳 File tree browser for remote rover files
- 📁 Local file browser for easy selection
- 🎯 Smart upload buttons for modular directories
- 📋 Enhanced terminal with click-to-navigate
- 🚀 Drag & drop file operations
- 📊 Progress bars and operation feedback

Author: Claude Code Assistant
Version: 3.0 - Enhanced File Management
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


class RemoteFileTree:
    """Remote file tree browser for rover files"""
    
    def __init__(self, ssh_client, tree_widget):
        self.ssh = ssh_client
        self.tree = tree_widget
        self.file_cache = {}  # Cache for file listings
        
    def refresh_tree(self, path="/home/jay"):
        """Refresh the remote file tree"""
        try:
            # Clear existing tree
            for item in self.tree.get_children():
                self.tree.delete(item)
            
            # Build tree starting from root path
            self._build_tree_node("", path)
            
        except Exception as e:
            print(f"Error refreshing tree: {e}")
    
    def _build_tree_node(self, parent, path):
        """Build a tree node with its children"""
        try:
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
                        name = ' '.join(parts[8:])
                        
                        if name in ['.', '..']:
                            continue
                            
                        is_dir = permissions.startswith('d')
                        item_path = os.path.join(path, name)
                        
                        items.append({
                            'name': name,
                            'path': item_path,
                            'is_dir': is_dir,
                            'size': size,
                            'permissions': permissions
                        })
            
            # Sort: directories first, then files
            items.sort(key=lambda x: (not x['is_dir'], x['name'].lower()))
            
            # Add items to tree
            for item in items:
                icon = "📁" if item['is_dir'] else "📄"
                display_name = f"{icon} {item['name']}"
                
                if item['is_dir']:
                    display_name += "/"
                else:
                    display_name += f" ({item['size']})"
                
                node = self.tree.insert(parent, "end", text=display_name, 
                                      values=(item['path'], item['is_dir'], item['size']))
                
                # Add placeholder for directories to make them expandable
                if item['is_dir']:
                    self.tree.insert(node, "end", text="Loading...")
                    
        except Exception as e:
            print(f"Error building tree node {path}: {e}")
    
    def expand_node(self, event):
        """Handle tree node expansion"""
        item = self.tree.selection()[0]
        children = self.tree.get_children(item)
        
        # Remove placeholder and load actual contents
        if len(children) == 1 and self.tree.item(children[0])['text'] == "Loading...":
            self.tree.delete(children[0])
            path = self.tree.item(item)['values'][0]
            self._build_tree_node(item, path)


class LocalFileTree:
    """Local file browser for selecting files to upload"""
    
    def __init__(self, tree_widget):
        self.tree = tree_widget
        self.current_path = Path.home()
        
    def refresh_tree(self, path=None):
        """Refresh local file tree"""
        if path:
            self.current_path = Path(path)
            
        try:
            # Clear existing tree
            for item in self.tree.get_children():
                self.tree.delete(item)
            
            # Add parent directory option (if not at root)
            if self.current_path.parent != self.current_path:
                self.tree.insert("", "end", text="📁 ..", 
                               values=(str(self.current_path.parent), True, ""))
            
            # Get directory contents
            items = []
            try:
                for item in self.current_path.iterdir():
                    if item.name.startswith('.'):
                        continue  # Skip hidden files
                        
                    size = ""
                    if item.is_file():
                        try:
                            size = f"{item.stat().st_size:,} bytes"
                        except:
                            size = "Unknown"
                    
                    items.append({
                        'name': item.name,
                        'path': str(item),
                        'is_dir': item.is_dir(),
                        'size': size
                    })
            except PermissionError:
                self.tree.insert("", "end", text="❌ Permission Denied", values=("", False, ""))
                return
            
            # Sort: directories first, then files
            items.sort(key=lambda x: (not x['is_dir'], x['name'].lower()))
            
            # Add items to tree
            for item in items:
                icon = "📁" if item['is_dir'] else "📄"
                display_name = f"{icon} {item['name']}"
                
                if item['is_dir']:
                    display_name += "/"
                else:
                    display_name += f" ({item['size']})"
                
                self.tree.insert("", "end", text=display_name,
                                values=(item['path'], item['is_dir'], item['size']))
                
        except Exception as e:
            print(f"Error refreshing local tree: {e}")
    
    def navigate_to(self, path):
        """Navigate to a specific path"""
        try:
            self.current_path = Path(path)
            self.refresh_tree()
        except Exception as e:
            print(f"Error navigating to {path}: {e}")


class ProgressDialog:
    """Progress dialog for file operations"""
    
    def __init__(self, parent, title="File Operation"):
        self.window = tk.Toplevel(parent)
        self.window.title(title)
        self.window.geometry("400x150")
        self.window.resizable(False, False)
        
        # Center the window
        self.window.transient(parent)
        self.window.grab_set()
        
        # Progress label
        self.label = ttk.Label(self.window, text="Preparing...")
        self.label.pack(pady=10)
        
        # Progress bar
        self.progress = ttk.Progressbar(self.window, mode='determinate', length=350)
        self.progress.pack(pady=10)
        
        # Status text
        self.status = ttk.Label(self.window, text="")
        self.status.pack(pady=5)
        
        # Cancel button
        self.cancel_btn = ttk.Button(self.window, text="Cancel", command=self.cancel)
        self.cancel_btn.pack(pady=10)
        
        self.cancelled = False
    
    def update(self, current, total, filename=""):
        """Update progress"""
        if not self.cancelled:
            progress_pct = (current / total) * 100 if total > 0 else 0
            self.progress['value'] = progress_pct
            self.label.config(text=f"Processing: {filename}")
            self.status.config(text=f"{current}/{total} ({progress_pct:.1f}%)")
            self.window.update()
    
    def set_status(self, text):
        """Set status text"""
        self.status.config(text=text)
        self.window.update()
    
    def cancel(self):
        """Cancel operation"""
        self.cancelled = True
        self.window.destroy()
    
    def close(self):
        """Close dialog"""
        if self.window.winfo_exists():
            self.window.destroy()


class EnhancedSSHDashboard:
    """Enhanced SSH Dashboard with advanced file management"""
    
    def __init__(self, root):
        try:
            print("Setting up root window...")
            self.root = root
            self.root.title("Raspberry Pi SSH Dashboard V3 - Enhanced File Management")
            self.root.geometry("1400x900")
            print("Root window configured")
            
            # SSH connection variables
            print("Initializing SSH variables...")
            self.ssh_client = None
            self.scp_client = None
            self.connected = False
            self.current_channel = None
            
            # File management
            self.remote_tree = None
            self.local_tree = None
            print("SSH variables initialized")
            
            # Theme variables
            print("Setting up themes...")
            self.dark_mode = False
            self.setup_themes()
            print("Themes configured")
            
            # Setup GUI
            print("Setting up GUI...")
            self.setup_gui()
            print("GUI setup complete")
            
            # Handle window close event
            self.root.protocol("WM_DELETE_WINDOW", self.exit_application)
            print("Window close handler set")
            
            # Force window to appear and be brought to front
            print("Making window visible...")
            self.root.update()
            self.root.deiconify()  # Make sure window is not minimized
            self.root.lift()       # Bring window to front
            self.root.focus_force() # Force focus
            self.root.attributes('-topmost', True)  # Temporarily on top
            self.root.after(100, lambda: self.root.attributes('-topmost', False))  # Remove topmost after delay
            print("Window should now be visible")
            
        except Exception as e:
            print(f"Error in __init__: {e}")
            import traceback
            traceback.print_exc()
            raise
        
    def setup_themes(self):
        """Setup light and dark themes"""
        # Light theme colors
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
        
        # Dark theme colors
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
        
        # Apply initial theme
        self.apply_theme()
    
    def apply_theme(self):
        """Apply current theme"""
        theme = self.dark_theme if self.dark_mode else self.light_theme
        
        # Configure root window
        self.root.configure(bg=theme['bg'])
        
        # Configure ttk styles
        style = ttk.Style()
        
        # Configure frame styles
        style.configure('TFrame', background=theme['bg'])
        style.configure('TLabelFrame', background=theme['bg'], foreground=theme['fg'])
        style.configure('TLabelFrame.Label', background=theme['bg'], foreground=theme['fg'])
        
        # Configure label styles
        style.configure('TLabel', background=theme['bg'], foreground=theme['fg'])
        
        # Configure entry styles
        style.configure('TEntry', fieldbackground=theme['entry_bg'], foreground=theme['entry_fg'])
        
        # Configure button styles
        style.configure('TButton', background=theme['button_bg'], foreground=theme['button_fg'])
        
        # Configure treeview styles
        style.configure('Treeview', background=theme['bg'], foreground=theme['fg'],
                       fieldbackground=theme['bg'])
        style.configure('Treeview.Heading', background=theme['button_bg'], foreground=theme['button_fg'])
    
    def setup_gui(self):
        """Setup the main GUI"""
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Connection & Terminal tab (combined)
        self.setup_connection_terminal_tab()
        
        # File management tab
        self.setup_file_management_tab()
        
        # Settings tab
        self.setup_settings_tab()
    
    def setup_connection_terminal_tab(self):
        """Setup combined connection settings and terminal tab"""
        connection_frame = ttk.Frame(self.notebook)
        self.notebook.add(connection_frame, text="🔗 Connection & Terminal")
        
        # Connection settings in a frame
        settings_frame = ttk.LabelFrame(connection_frame, text="Connection Settings", padding="10")
        settings_frame.pack(fill='x', padx=10, pady=10)
        
        # SSH connection details
        ttk.Label(settings_frame, text="SSH Host:").grid(row=0, column=0, sticky='w', pady=2)
        self.host_var = tk.StringVar(value="192.168.254.65")
        self.host_entry = ttk.Entry(settings_frame, textvariable=self.host_var, width=20)
        self.host_entry.grid(row=0, column=1, sticky='ew', padx=(5, 10), pady=2)
        
        ttk.Label(settings_frame, text="Port:").grid(row=0, column=2, sticky='w', pady=2)
        self.port_var = tk.StringVar(value="22")
        self.port_entry = ttk.Entry(settings_frame, textvariable=self.port_var, width=8)
        self.port_entry.grid(row=0, column=3, sticky='ew', padx=(5, 0), pady=2)
        
        ttk.Label(settings_frame, text="Username:").grid(row=1, column=0, sticky='w', pady=2)
        self.username_var = tk.StringVar(value="jay")
        self.username_entry = ttk.Entry(settings_frame, textvariable=self.username_var, width=20)
        self.username_entry.grid(row=1, column=1, sticky='ew', padx=(5, 10), pady=2)
        
        ttk.Label(settings_frame, text="Password:").grid(row=1, column=2, sticky='w', pady=2)
        self.password_var = tk.StringVar()
        self.password_entry = ttk.Entry(settings_frame, textvariable=self.password_var, show="*", width=20)
        self.password_entry.grid(row=1, column=3, sticky='ew', padx=(5, 0), pady=2)
        
        # Configure grid weights
        settings_frame.columnconfigure(1, weight=1)
        settings_frame.columnconfigure(3, weight=1)
        
        # Connection buttons
        button_frame = ttk.Frame(settings_frame)
        button_frame.grid(row=2, column=0, columnspan=4, pady=10)
        
        self.connect_btn = ttk.Button(button_frame, text="Connect", command=self.connect_ssh)
        self.connect_btn.pack(side='left', padx=5)
        
        self.disconnect_btn = ttk.Button(button_frame, text="Disconnect", 
                                        command=self.disconnect_ssh, state='disabled')
        self.disconnect_btn.pack(side='left', padx=5)
        
        # Status
        self.status_var = tk.StringVar(value="Disconnected")
        self.status_label = ttk.Label(settings_frame, textvariable=self.status_var)
        self.status_label.grid(row=3, column=0, columnspan=4, pady=5)
        
        # Create horizontal layout for terminal and program launcher
        terminal_container = ttk.Frame(connection_frame)
        terminal_container.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Left side - Terminal & Connection Info
        terminal_frame = ttk.LabelFrame(terminal_container, text="Terminal & Connection Info", padding="5")
        terminal_frame.pack(side='left', fill='both', expand=True, padx=(0, 5))
        
        # Right side - Program Launcher
        self.setup_program_launcher(terminal_container)
        
        # Terminal output (this serves as both terminal and connection info)
        theme = self.dark_theme if self.dark_mode else self.light_theme
        self.terminal_output = scrolledtext.ScrolledText(
            terminal_frame, height=15, width=80,
            bg=theme['terminal_bg'], fg=theme['terminal_fg'],
            selectbackground=theme['select_bg'], selectforeground=theme['select_fg'],
            insertbackground=theme['fg']
        )
        self.terminal_output.pack(fill='both', expand=True, pady=(0, 5))
        
        # Add copy support for terminal output
        self.terminal_output.bind('<Control-c>', self.copy_terminal_selection)
        self.terminal_output.bind('<Control-a>', self.select_all_terminal)
        
        # Command input frame
        command_frame = ttk.Frame(terminal_frame)
        command_frame.pack(fill='x', pady=5)
        
        # Command entry
        ttk.Label(command_frame, text="Command:").pack(side='left')
        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(command_frame, textvariable=self.command_var)
        self.command_entry.pack(side='left', fill='x', expand=True, padx=(5, 5))
        self.command_entry.bind('<Return>', self.execute_command)
        
        # Add copy/paste keyboard shortcuts for command entry
        self.command_entry.bind('<Control-c>', self.copy_command)
        self.command_entry.bind('<Control-v>', self.paste_command)
        self.command_entry.bind('<Control-a>', self.select_all_command)
        
        # Terminal control buttons
        control_frame = ttk.Frame(command_frame)
        control_frame.pack(side='right')
        
        ttk.Button(control_frame, text="Execute", command=self.execute_command).pack(side='left', padx=2)
        ttk.Button(control_frame, text="Clear", command=self.clear_terminal).pack(side='left', padx=2)
        ttk.Button(control_frame, text="Save Logs", command=self.save_terminal_logs).pack(side='left', padx=2)
        
        # Quick command buttons
        quick_frame = ttk.LabelFrame(connection_frame, text="Quick Commands", padding="5")
        quick_frame.pack(fill='x', padx=10, pady=(0, 10))
        
        ttk.Button(quick_frame, text="📁 cd rover_project", 
                  command=lambda: self.quick_command("cd /home/jay/rover_project && pwd")).pack(side='left', padx=2)
        ttk.Button(quick_frame, text="🐍 Activate venv", 
                  command=lambda: self.quick_command("cd /home/jay/rover_project && source venv/bin/activate && echo 'Virtual environment activated'")).pack(side='left', padx=2)
        ttk.Button(quick_frame, text="🚀 Start Rover", 
                  command=lambda: self.quick_command("cd /home/jay/rover_project && source venv/bin/activate && python3 rover_smart_startup.py")).pack(side='left', padx=2)
        ttk.Button(quick_frame, text="📊 System Status", 
                  command=lambda: self.quick_command("df -h && free -h && uptime")).pack(side='left', padx=2)
        
        # For backward compatibility, keep a reference to terminal output as info_text
        self.info_text = self.terminal_output
    
    def setup_program_launcher(self, parent):
        """Setup program launcher panel"""
        launcher_frame = ttk.LabelFrame(parent, text="🚀 Program Launcher", padding="5")
        launcher_frame.pack(side='right', fill='y', padx=(5, 0))
        launcher_frame.config(width=300)
        
        # Program list
        list_frame = ttk.Frame(launcher_frame)
        list_frame.pack(fill='both', expand=True, pady=(0, 10))
        
        ttk.Label(list_frame, text="Available Programs:").pack(anchor='w')
        
        # Listbox with scrollbar
        list_container = ttk.Frame(list_frame)
        list_container.pack(fill='both', expand=True, pady=5)
        
        self.program_listbox = tk.Listbox(list_container, height=15, selectmode='single')
        program_scrollbar = ttk.Scrollbar(list_container, orient='vertical', command=self.program_listbox.yview)
        self.program_listbox.configure(yscrollcommand=program_scrollbar.set)
        
        self.program_listbox.pack(side='left', fill='both', expand=True)
        program_scrollbar.pack(side='right', fill='y')
        
        # Bind double-click to run program
        self.program_listbox.bind('<Double-1>', self.run_selected_program)
        
        # Control buttons
        button_frame = ttk.Frame(launcher_frame)
        button_frame.pack(fill='x', pady=5)
        
        ttk.Button(button_frame, text="🔄 Refresh List", 
                  command=self.refresh_program_list).pack(fill='x', pady=2)
        ttk.Button(button_frame, text="🚀 Run Program", 
                  command=self.run_selected_program).pack(fill='x', pady=2)
        ttk.Button(button_frame, text="🛑 Stop Program", 
                  command=self.stop_current_program).pack(fill='x', pady=2)
        
        # Program info
        info_frame = ttk.LabelFrame(launcher_frame, text="Program Info", padding="5")
        info_frame.pack(fill='x', pady=5)
        
        self.program_info = tk.Text(info_frame, height=4, wrap='word', state='disabled')
        self.program_info.pack(fill='x')
        
        # Running program status
        status_frame = ttk.Frame(launcher_frame)
        status_frame.pack(fill='x', pady=5)
        
        ttk.Label(status_frame, text="Status:").pack(anchor='w')
        self.program_status_var = tk.StringVar(value="No program running")
        self.program_status_label = ttk.Label(status_frame, textvariable=self.program_status_var, 
                                            foreground='gray')
        self.program_status_label.pack(anchor='w')
        
        # Initialize with empty list
        self.current_programs = []
        self.running_process = None
    
    def setup_file_management_tab(self):
        """Setup enhanced file management tab"""
        file_frame = ttk.Frame(self.notebook)
        self.notebook.add(file_frame, text="📁 File Management")
        
        # Create main paned window for dual-pane layout
        main_paned = ttk.PanedWindow(file_frame, orient='horizontal')
        main_paned.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Left pane - Local files
        self.setup_local_file_pane(main_paned)
        
        # Right pane - Remote files  
        self.setup_remote_file_pane(main_paned)
        
        # Bottom pane - File operations and progress
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
        
        self.local_tree = ttk.Treeview(tree_frame, columns=('path', 'is_dir', 'size'), show='tree')
        self.local_tree.heading('#0', text='Name', anchor='w')
        
        # Scrollbars
        local_v_scroll = ttk.Scrollbar(tree_frame, orient='vertical', command=self.local_tree.yview)
        local_h_scroll = ttk.Scrollbar(tree_frame, orient='horizontal', command=self.local_tree.xview)
        self.local_tree.configure(yscrollcommand=local_v_scroll.set, xscrollcommand=local_h_scroll.set)
        
        self.local_tree.pack(side='left', fill='both', expand=True)
        local_v_scroll.pack(side='right', fill='y')
        local_h_scroll.pack(side='bottom', fill='x')
        
        # Bind events
        self.local_tree.bind('<Double-1>', self.on_local_double_click)
        self.local_tree.bind('<Button-3>', self.on_local_right_click)
        
        # Initialize local tree
        self.local_file_tree = LocalFileTree(self.local_tree)
        self.local_file_tree.refresh_tree()
    
    def setup_remote_file_pane(self, parent):
        """Setup remote file browser pane"""
        remote_frame = ttk.LabelFrame(parent, text="🤖 Remote Files (Rover)", padding="5")
        parent.add(remote_frame, weight=1)
        
        # Path navigation
        nav_frame = ttk.Frame(remote_frame)
        nav_frame.pack(fill='x', pady=(0, 5))
        
        ttk.Label(nav_frame, text="Path:").pack(side='left')
        self.remote_path_var = tk.StringVar(value="/home/jay/rover_project")
        self.remote_path_entry = ttk.Entry(nav_frame, textvariable=self.remote_path_var)
        self.remote_path_entry.pack(side='left', fill='x', expand=True, padx=(5, 5))
        
        ttk.Button(nav_frame, text="Refresh", command=self.refresh_remote_tree).pack(side='right')
        
        # File tree
        tree_frame = ttk.Frame(remote_frame)
        tree_frame.pack(fill='both', expand=True)
        
        self.remote_tree_widget = ttk.Treeview(tree_frame, columns=('path', 'is_dir', 'size'), show='tree')
        self.remote_tree_widget.heading('#0', text='Name', anchor='w')
        
        # Scrollbars
        remote_v_scroll = ttk.Scrollbar(tree_frame, orient='vertical', command=self.remote_tree_widget.yview)
        remote_h_scroll = ttk.Scrollbar(tree_frame, orient='horizontal', command=self.remote_tree_widget.xview)
        self.remote_tree_widget.configure(yscrollcommand=remote_v_scroll.set, xscrollcommand=remote_h_scroll.set)
        
        self.remote_tree_widget.pack(side='left', fill='both', expand=True)
        remote_v_scroll.pack(side='right', fill='y')
        remote_h_scroll.pack(side='bottom', fill='x')
        
        # Bind events
        self.remote_tree_widget.bind('<Double-1>', self.on_remote_double_click)
        self.remote_tree_widget.bind('<Button-3>', self.on_remote_right_click)
        self.remote_tree_widget.bind('<<TreeviewOpen>>', self.on_remote_tree_expand)
    
    def setup_file_operations_pane(self, parent):
        """Setup file operations panel"""
        ops_frame = ttk.LabelFrame(parent, text="🚀 Quick Operations", padding="5")
        ops_frame.pack(fill='x', padx=5, pady=(0, 5))
        
        # Smart deployment buttons
        deploy_frame = ttk.Frame(ops_frame)
        deploy_frame.pack(fill='x', pady=5)
        
        ttk.Label(deploy_frame, text="Smart Deploy:").pack(side='left')
        
        ttk.Button(deploy_frame, text="📡 Upload to Sensors/", 
                  command=lambda: self.smart_upload('Sensors')).pack(side='left', padx=2)
        ttk.Button(deploy_frame, text="🧭 Upload to Navigation/", 
                  command=lambda: self.smart_upload('Navigation')).pack(side='left', padx=2)
        ttk.Button(deploy_frame, text="🤖 Upload to Main/", 
                  command=lambda: self.smart_upload('Main')).pack(side='left', padx=2)
        ttk.Button(deploy_frame, text="🎯 Upload to Behaviors/", 
                  command=lambda: self.smart_upload('Autonomous Behaviors')).pack(side='left', padx=2)
        
        # File operation buttons
        button_frame = ttk.Frame(ops_frame)
        button_frame.pack(fill='x', pady=5)
        
        ttk.Button(button_frame, text="📤 Upload Selected", 
                  command=self.upload_selected_files).pack(side='left', padx=2)
        ttk.Button(button_frame, text="📥 Download Selected", 
                  command=self.download_selected_files).pack(side='left', padx=2)
        ttk.Button(button_frame, text="🗑️ Delete Selected", 
                  command=self.delete_selected_files).pack(side='left', padx=2)
        ttk.Button(button_frame, text="📋 Compare Files", 
                  command=self.compare_files).pack(side='left', padx=2)
        
        # Program management buttons
        program_frame = ttk.Frame(ops_frame)
        program_frame.pack(fill='x', pady=5)
        
        ttk.Label(program_frame, text="Program Management:").pack(side='left')
        ttk.Button(program_frame, text="🗑️ Delete Programs", 
                  command=self.delete_programs_dialog).pack(side='left', padx=2)
        ttk.Button(program_frame, text="🔄 Refresh Programs", 
                  command=self.refresh_program_list).pack(side='left', padx=2)
        ttk.Button(program_frame, text="📋 List Programs", 
                  command=self.show_programs_list).pack(side='left', padx=2)
        
    
    def setup_settings_tab(self):
        """Setup settings and preferences tab"""
        settings_frame = ttk.Frame(self.notebook)
        self.notebook.add(settings_frame, text="⚙️ Settings")
        
        # Theme settings
        theme_frame = ttk.LabelFrame(settings_frame, text="Theme", padding="10")
        theme_frame.pack(fill='x', padx=10, pady=10)
        
        self.theme_var = tk.BooleanVar(value=self.dark_mode)
        theme_check = ttk.Checkbutton(theme_frame, text="Dark Mode", 
                                     variable=self.theme_var, command=self.toggle_theme)
        theme_check.pack(anchor='w')
        
        # File operation settings
        file_ops_frame = ttk.LabelFrame(settings_frame, text="File Operations", padding="10")
        file_ops_frame.pack(fill='x', padx=10, pady=10)
        
        self.backup_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(file_ops_frame, text="Create backup before overwriting files", 
                       variable=self.backup_var).pack(anchor='w')
        
        self.confirm_delete_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(file_ops_frame, text="Confirm before deleting files", 
                       variable=self.confirm_delete_var).pack(anchor='w')
        
        # About section
        about_frame = ttk.LabelFrame(settings_frame, text="About", padding="10")
        about_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        about_text = """
Dashboard V3 - Enhanced File Management Edition

Features:
🌳 Dual-pane file browser (local/remote)
📁 File tree navigation with context menus  
🎯 Smart deployment buttons for modular directories
📋 Enhanced terminal with click-to-navigate
🚀 Drag & drop file operations (coming soon)
📊 Progress tracking and batch operations
💾 Backup creation and file comparison
🔗 Improved SSH connection management

Version: 3.0
Author: Claude Code Assistant
        """
        
        about_label = ttk.Label(about_frame, text=about_text, justify='left')
        about_label.pack(anchor='w')

    # Event handlers and functionality methods will continue...
    
    def connect_ssh(self):
        """Connect to SSH server"""
        def connect_thread():
            try:
                host = self.host_var.get()
                port = int(self.port_var.get())
                username = self.username_var.get()
                password = self.password_var.get()
                
                self.root.after(0, lambda: self.status_var.set("Connecting..."))
                
                # Create SSH client
                self.ssh_client = paramiko.SSHClient()
                self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                
                # Connect
                self.ssh_client.connect(host, port=port, username=username, password=password)
                
                # Create SCP client
                self.scp_client = SCPClient(self.ssh_client.get_transport())
                
                self.connected = True
                
                # Update UI
                self.root.after(0, self.on_connection_success)
                
            except Exception as e:
                error_msg = str(e)
                self.root.after(0, lambda: self.on_connection_error(error_msg))
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def on_connection_success(self):
        """Handle successful connection"""
        self.status_var.set("Connected")
        self.connect_btn.config(state='disabled')
        self.disconnect_btn.config(state='normal')
        
        # Initialize remote file tree
        self.remote_tree = RemoteFileTree(self.ssh_client, self.remote_tree_widget)
        self.refresh_remote_tree()
        
        # Refresh program list
        self.refresh_program_list()
        
        # Add connection info to both Connection tab and Terminal
        host = self.host_var.get()
        port = self.port_var.get()
        username = self.username_var.get()
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        connection_info = f"Connected to Raspberry Pi (Enhanced Dashboard V3)\n"
        connection_info += f"Host: {host}:{port}\n"
        connection_info += f"Username: {username}\n"
        connection_info += f"Connection established: {timestamp}\n"
        connection_info += "=" * 50 + "\n\n"
        
        # Add to Connection tab info
        self.info_text.insert(tk.END, connection_info)
        
        # Add to Terminal output
        self.terminal_output.insert(tk.END, connection_info)
        self.terminal_output.see(tk.END)
        
        # Switch to file management tab
        self.notebook.select(1)  # File management is tab 1 (after connection)
    
    def on_connection_error(self, error):
        """Handle connection error"""
        self.status_var.set("Connection Failed")
        messagebox.showerror("Connection Error", f"Failed to connect: {error}")
    
    def disconnect_ssh(self):
        """Disconnect from SSH server"""
        if self.scp_client:
            self.scp_client.close()
        if self.ssh_client:
            self.ssh_client.close()
        
        self.connected = False
        self.status_var.set("Disconnected")
        self.connect_btn.config(state='normal')
        self.disconnect_btn.config(state='disabled')
        
        disconnect_info = f"Disconnected at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
        
        # Add to both Connection tab and Terminal
        self.info_text.insert(tk.END, disconnect_info)
        self.terminal_output.insert(tk.END, disconnect_info)
        self.terminal_output.see(tk.END)
    
    def toggle_theme(self):
        """Toggle between light and dark theme"""
        self.dark_mode = self.theme_var.get()
        self.apply_theme()
        
        # Update terminal colors if it exists
        if hasattr(self, 'terminal_output'):
            theme = self.dark_theme if self.dark_mode else self.light_theme
            self.terminal_output.configure(
                bg=theme['terminal_bg'],
                fg=theme['terminal_fg'],
                selectbackground=theme['select_bg'],
                selectforeground=theme['select_fg']
            )
    
    # Continue with remaining methods...
    def navigate_local(self):
        """Navigate to local path"""
        path = self.local_path_var.get()
        self.local_file_tree.navigate_to(path)
    
    def go_local_home(self):
        """Go to local home directory"""
        home_path = str(Path.home())
        self.local_path_var.set(home_path)
        self.navigate_local()
    
    def refresh_remote_tree(self):
        """Refresh remote file tree"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to SSH server first")
            return
        
        path = self.remote_path_var.get()
        self.remote_tree.refresh_tree(path)
    
    def on_local_double_click(self, event):
        """Handle local file tree double click"""
        item = self.local_tree.selection()[0]
        values = self.local_tree.item(item)['values']
        if len(values) >= 2:
            path, is_dir = values[0], values[1]
            if is_dir:
                self.local_path_var.set(path)
                self.navigate_local()
    
    def on_remote_double_click(self, event):
        """Handle remote file tree double click"""
        if not self.connected:
            return
            
        item = self.remote_tree_widget.selection()[0]
        values = self.remote_tree_widget.item(item)['values']
        if len(values) >= 2:
            path, is_dir = values[0], values[1]
            if is_dir:
                self.remote_path_var.set(path)
                self.refresh_remote_tree()
    
    def on_remote_tree_expand(self, event):
        """Handle remote tree expansion"""
        if self.remote_tree:
            self.remote_tree.expand_node(event)
    
    def on_local_right_click(self, event):
        """Handle local file tree right click"""
        # Create context menu for local files
        menu = tk.Menu(self.root, tearoff=0)
        menu.add_command(label="📤 Upload to Remote", command=self.upload_selected_files)
        menu.add_command(label="📋 Copy Path", command=self.copy_local_path)
        menu.add_separator()
        menu.add_command(label="🔄 Refresh", command=lambda: self.local_file_tree.refresh_tree())
        
        try:
            menu.tk_popup(event.x_root, event.y_root)
        finally:
            menu.grab_release()
    
    def on_remote_right_click(self, event):
        """Handle remote file tree right click"""
        # Create context menu for remote files
        menu = tk.Menu(self.root, tearoff=0)
        menu.add_command(label="📥 Download to Local", command=self.download_selected_files)
        menu.add_command(label="🗑️ Delete", command=self.delete_selected_files)
        menu.add_command(label="📋 Copy Path", command=self.copy_remote_path)
        menu.add_separator()
        menu.add_command(label="💻 Open in Terminal", command=self.open_remote_in_terminal)
        menu.add_command(label="🔄 Refresh", command=self.refresh_remote_tree)
        
        try:
            menu.tk_popup(event.x_root, event.y_root)
        finally:
            menu.grab_release()
    
    def smart_upload(self, target_dir):
        """Smart upload to specific modular directory"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to SSH server first")
            return
        
        # Open file dialog for selection
        files = filedialog.askopenfilenames(
            title=f"Select files to upload to {target_dir}/",
            filetypes=[("Python files", "*.py"), ("All files", "*.*")]
        )
        
        if not files:
            return
        
        # Determine target path
        base_path = self.remote_path_var.get()
        target_path = f"{base_path}/{target_dir}/"
        
        # Upload files
        self.upload_files_to_path(files, target_path, f"Upload to {target_dir}")
    
    def upload_selected_files(self):
        """Upload selected local files"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to SSH server first")
            return
        
        selection = self.local_tree.selection()
        if not selection:
            messagebox.showinfo("No Selection", "Please select files to upload")
            return
        
        files = []
        for item in selection:
            values = self.local_tree.item(item)['values']
            if len(values) >= 2:
                path, is_dir = values[0], values[1]
                if not is_dir:  # Only upload files, not directories for now
                    files.append(path)
        
        if not files:
            messagebox.showinfo("No Files", "No files selected for upload")
            return
        
        # Upload to current remote path
        target_path = self.remote_path_var.get() + "/"
        self.upload_files_to_path(files, target_path, "Upload Files")
    
    def upload_files_to_path(self, files, target_path, operation_title):
        """Upload files to specific remote path with progress"""
        def upload_thread():
            progress = ProgressDialog(self.root, operation_title)
            
            try:
                for i, file_path in enumerate(files):
                    if progress.cancelled:
                        break
                    
                    filename = os.path.basename(file_path)
                    progress.update(i, len(files), filename)
                    
                    # Create backup if file exists and backup is enabled
                    remote_file_path = target_path + filename
                    if self.backup_var.get():
                        try:
                            # Check if file exists
                            stdin, stdout, stderr = self.ssh_client.exec_command(f"test -f '{remote_file_path}' && echo 'exists'")
                            if stdout.read().decode().strip() == 'exists':
                                backup_path = f"{remote_file_path}.backup.{int(time.time())}"
                                self.ssh_client.exec_command(f"cp '{remote_file_path}' '{backup_path}'")
                                progress.set_status(f"Created backup: {backup_path}")
                        except:
                            pass  # Backup failed, continue anyway
                    
                    # Upload file
                    self.scp_client.put(file_path, remote_file_path)
                    progress.set_status(f"Uploaded: {filename}")
                
                progress.update(len(files), len(files), "Complete")
                time.sleep(1)  # Show completion briefly
                
                # Refresh remote tree
                self.root.after(0, self.refresh_remote_tree)
                
                # Show success message
                if not progress.cancelled:
                    self.root.after(0, lambda: messagebox.showinfo("Upload Complete", 
                                                                  f"Successfully uploaded {len(files)} files"))
                
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("Upload Error", f"Upload failed: {e}"))
            
            finally:
                progress.close()
        
        threading.Thread(target=upload_thread, daemon=True).start()
    
    def download_selected_files(self):
        """Download selected remote files"""
        # Implementation for downloading files
        messagebox.showinfo("Coming Soon", "Download functionality will be implemented next")
    
    def delete_selected_files(self):
        """Delete selected remote files"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to SSH server first")
            return
        
        selection = self.remote_tree_widget.selection()
        if not selection:
            messagebox.showinfo("No Selection", "Please select files to delete")
            return
        
        files = []
        for item in selection:
            values = self.remote_tree_widget.item(item)['values']
            if len(values) >= 1:
                path = values[0]
                files.append(path)
        
        if not files:
            return
        
        # Confirm deletion
        if self.confirm_delete_var.get():
            result = messagebox.askyesno("Confirm Delete", 
                                       f"Are you sure you want to delete {len(files)} file(s)?")
            if not result:
                return
        
        # Delete files
        def delete_thread():
            try:
                for file_path in files:
                    self.ssh_client.exec_command(f"rm '{file_path}'")
                
                # Refresh remote tree
                self.root.after(0, self.refresh_remote_tree)
                self.root.after(0, lambda: messagebox.showinfo("Delete Complete", 
                                                              f"Successfully deleted {len(files)} files"))
                
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("Delete Error", f"Delete failed: {e}"))
        
        threading.Thread(target=delete_thread, daemon=True).start()
    
    def compare_files(self):
        """Compare selected local and remote files"""
        messagebox.showinfo("Coming Soon", "File comparison functionality will be implemented next")
    
    def copy_local_path(self):
        """Copy local file path to clipboard"""
        selection = self.local_tree.selection()
        if selection:
            values = self.local_tree.item(selection[0])['values']
            if values:
                self.root.clipboard_clear()
                self.root.clipboard_append(values[0])
    
    def copy_remote_path(self):
        """Copy remote file path to clipboard"""
        selection = self.remote_tree_widget.selection()
        if selection:
            values = self.remote_tree_widget.item(selection[0])['values']
            if values:
                self.root.clipboard_clear()
                self.root.clipboard_append(values[0])
    
    def open_remote_in_terminal(self):
        """Open remote path in terminal"""
        selection = self.remote_tree_widget.selection()
        if selection:
            values = self.remote_tree_widget.item(selection[0])['values']
            if len(values) >= 2:
                path, is_dir = values[0], values[1]
                if is_dir:
                    # Switch to connection tab and cd to directory
                    self.notebook.select(0)  # Connection & Terminal is tab 0
                    self.command_var.set(f"cd '{path}' && pwd")
                    self.execute_command()
    
    # Terminal methods (copy from V2 with enhancements)
    def execute_command(self, event=None):
        """Execute command in terminal"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to SSH server first")
            return
        
        command = self.command_var.get().strip()
        if not command:
            return
        
        self.command_var.set("")  # Clear command entry
        
        def execute_thread():
            try:
                self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"$ {command}\n"))
                
                # Execute command
                stdin, stdout, stderr = self.ssh_client.exec_command(command)
                
                # Read output
                output = stdout.read().decode('utf-8', errors='replace')
                error = stderr.read().decode('utf-8', errors='replace')
                
                if output:
                    self.root.after(0, lambda: self.terminal_output.insert(tk.END, output))
                if error:
                    self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"[ERROR] {error}"))
                
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
                
            except Exception as e:
                self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"Command failed: {e}\n"))
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
        
        threading.Thread(target=execute_thread, daemon=True).start()
    
    def quick_command(self, command):
        """Execute a quick command"""
        self.command_var.set(command)
        self.execute_command()
    
    def clear_terminal(self):
        """Clear terminal output"""
        self.terminal_output.delete(1.0, tk.END)
    
    def save_terminal_logs(self):
        """Save terminal logs to file"""
        try:
            filename = filedialog.asksaveasfilename(
                defaultextension=".txt",
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
                title="Save Terminal Logs"
            )
            
            if filename:
                content = self.terminal_output.get(1.0, tk.END)
                with open(filename, 'w') as f:
                    f.write(f"SSH Dashboard V3 Terminal Log\n")
                    f.write(f"Session Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write(f"Connected to: {self.host_var.get()}:{self.port_var.get()}\n")
                    f.write(f"Username: {self.username_var.get()}\n")
                    f.write("=" * 50 + "\n\n")
                    f.write(content)
                
                messagebox.showinfo("Success", f"Terminal logs saved to {filename}")
        
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save logs: {e}")
    
    # Copy/paste methods (from V2)
    def copy_command(self, event=None):
        """Copy selected text from command entry"""
        try:
            if self.command_entry.selection_present():
                selected_text = self.command_entry.selection_get()
                self.root.clipboard_clear()
                self.root.clipboard_append(selected_text)
                self.root.update()
        except tk.TclError:
            try:
                all_text = self.command_entry.get()
                if all_text:
                    self.root.clipboard_clear()
                    self.root.clipboard_append(all_text)
                    self.root.update()
            except Exception:
                pass
        return "break"
    
    def paste_command(self, event=None):
        """Paste text from clipboard to command entry"""
        try:
            clipboard_text = self.root.clipboard_get()
            cursor_pos = self.command_entry.index(tk.INSERT)
            current_text = self.command_entry.get()
            new_text = current_text[:cursor_pos] + clipboard_text + current_text[cursor_pos:]
            self.command_var.set(new_text)
            self.command_entry.icursor(cursor_pos + len(clipboard_text))
        except tk.TclError:
            pass
        return "break"
    
    def select_all_command(self, event=None):
        """Select all text in command entry"""
        self.command_entry.select_range(0, tk.END)
        self.command_entry.icursor(tk.END)
        return "break"
    
    def copy_terminal_selection(self, event=None):
        """Copy selected text from terminal output"""
        try:
            if self.terminal_output.tag_ranges("sel"):
                selected_text = self.terminal_output.selection_get()
                self.root.clipboard_clear()
                self.root.clipboard_append(selected_text)
                self.root.update()
        except tk.TclError:
            pass
        return "break"
    
    def select_all_terminal(self, event=None):
        """Select all text in terminal output"""
        self.terminal_output.tag_add("sel", "1.0", tk.END)
        return "break"
    
    def refresh_program_list(self):
        """Refresh the list of available programs on the Pi"""
        if not self.connected:
            return
        
        def refresh_thread():
            try:
                # Get Python files in rover_project directory
                command = "find /home/jay/rover_project -maxdepth 1 -name '*.py' -type f"
                stdin, stdout, stderr = self.ssh_client.exec_command(command)
                files = stdout.read().decode().strip().split('\n')
                
                programs = []
                for file_path in files:
                    if file_path.strip():
                        filename = os.path.basename(file_path)
                        # Skip __pycache__ and other non-executable files
                        if not filename.startswith('__') and filename.endswith('.py'):
                            programs.append({
                                'name': filename,
                                'path': file_path,
                                'description': self.get_program_description(file_path)
                            })
                
                # Update UI
                self.root.after(0, lambda: self.update_program_list(programs))
                
            except Exception as e:
                self.root.after(0, lambda: print(f"Error refreshing program list: {e}"))
        
        threading.Thread(target=refresh_thread, daemon=True).start()
    
    def get_program_description(self, file_path):
        """Get program description from file docstring"""
        try:
            # Read first few lines to get description
            command = f"head -20 '{file_path}' | grep -E '^(#|\"\"\")' | head -5"
            stdin, stdout, stderr = self.ssh_client.exec_command(command)
            lines = stdout.read().decode().strip().split('\n')
            
            description = ""
            for line in lines:
                line = line.strip()
                if line.startswith('"""') and 'description' not in line.lower():
                    continue
                if line.startswith('#') or line.startswith('"""'):
                    desc_part = line.lstrip('#').lstrip('"').strip()
                    if desc_part and not desc_part.startswith('!'):
                        description = desc_part[:100]  # Limit length
                        break
            
            return description or "Python script"
            
        except Exception:
            return "Python script"
    
    def update_program_list(self, programs):
        """Update the program listbox with new programs"""
        self.current_programs = programs
        self.program_listbox.delete(0, tk.END)
        
        for program in programs:
            display_name = program['name']
            self.program_listbox.insert(tk.END, display_name)
        
        # Update info if a program is selected
        self.on_program_select()
    
    def on_program_select(self, event=None):
        """Handle program selection"""
        selection = self.program_listbox.curselection()
        if selection and self.current_programs:
            index = selection[0]
            program = self.current_programs[index]
            
            # Update program info
            self.program_info.config(state='normal')
            self.program_info.delete(1.0, tk.END)
            info_text = f"Name: {program['name']}\n"
            info_text += f"Path: {program['path']}\n"
            info_text += f"Description: {program['description']}"
            self.program_info.insert(1.0, info_text)
            self.program_info.config(state='disabled')
        
        # Bind selection event
        self.program_listbox.bind('<<ListboxSelect>>', self.on_program_select)
    
    def run_selected_program(self, event=None):
        """Run the selected program"""
        selection = self.program_listbox.curselection()
        if not selection or not self.current_programs:
            messagebox.showinfo("No Selection", "Please select a program to run")
            return
        
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to SSH server first")
            return
        
        index = selection[0]
        program = self.current_programs[index]
        
        # Confirm execution
        result = messagebox.askyesno("Run Program", 
                                   f"Run {program['name']}?\n\nThis will execute the program in the rover environment.")
        if not result:
            return
        
        # Build command to run program
        if 'rover' in program['name'].lower():
            # Rover programs need virtual environment
            command = f"cd /home/jay/rover_project && source venv/bin/activate && python3 {program['name']}"
        else:
            # Other Python scripts
            command = f"cd /home/jay/rover_project && python3 {program['name']}"
        
        # Execute in terminal
        self.program_status_var.set(f"Running: {program['name']}")
        self.program_status_label.config(foreground='green')
        
        # Add to terminal and execute
        self.terminal_output.insert(tk.END, f"\n🚀 Starting program: {program['name']}\n")
        self.terminal_output.insert(tk.END, f"Command: {command}\n")
        self.terminal_output.see(tk.END)
        
        self.command_var.set(command)
        self.execute_command()
    
    def stop_current_program(self):
        """Stop the currently running program"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to SSH server first")
            return
        
        # Send interrupt signal to stop running programs
        try:
            # Try to kill Python processes
            command = "pkill -f 'python.*\\.py'"
            stdin, stdout, stderr = self.ssh_client.exec_command(command)
            
            self.program_status_var.set("Stopped running programs")
            self.program_status_label.config(foreground='red')
            
            self.terminal_output.insert(tk.END, "\n🛑 Stop signal sent to running programs\n")
            self.terminal_output.see(tk.END)
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to stop programs: {e}")
    
    def delete_programs_dialog(self):
        """Show dialog to select and delete programs"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to SSH server first")
            return
        
        # Create dialog window
        dialog = tk.Toplevel(self.root)
        dialog.title("Delete Programs")
        dialog.geometry("500x400")
        dialog.resizable(False, False)
        dialog.transient(self.root)
        dialog.grab_set()
        
        # Center the dialog
        dialog.geometry("+%d+%d" % (self.root.winfo_rootx() + 50, self.root.winfo_rooty() + 50))
        
        # Main frame
        main_frame = ttk.Frame(dialog, padding="10")
        main_frame.pack(fill='both', expand=True)
        
        # Instructions
        ttk.Label(main_frame, text="Select programs to delete from the rover:", 
                 font=('TkDefaultFont', 10, 'bold')).pack(pady=(0, 10))
        
        # Program list with checkboxes
        list_frame = ttk.Frame(main_frame)
        list_frame.pack(fill='both', expand=True, pady=(0, 10))
        
        # Create scrollable listbox with checkboxes
        canvas = tk.Canvas(list_frame, height=250)
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Get programs and create checkboxes
        self.program_vars = []
        self.program_paths = []
        
        def load_programs():
            try:
                # Get Python files in rover_project directory
                command = "find /home/jay/rover_project -name '*.py' -type f | sort"
                stdin, stdout, stderr = self.ssh_client.exec_command(command)
                files = stdout.read().decode().strip().split('\n')
                
                for i, file_path in enumerate(files):
                    if file_path.strip() and not '__pycache__' in file_path:
                        filename = os.path.basename(file_path)
                        # Create checkbox for each program
                        var = tk.BooleanVar()
                        self.program_vars.append(var)
                        self.program_paths.append(file_path)
                        
                        frame = ttk.Frame(scrollable_frame)
                        frame.pack(fill='x', pady=1)
                        
                        checkbox = ttk.Checkbutton(frame, text=filename, variable=var)
                        checkbox.pack(side='left')
                        
                        # Add file path as tooltip
                        ttk.Label(frame, text=f"({file_path})", foreground='gray').pack(side='left', padx=(5, 0))
                
            except Exception as e:
                ttk.Label(scrollable_frame, text=f"Error loading programs: {e}").pack()
        
        # Load programs in thread
        threading.Thread(target=load_programs, daemon=True).start()
        
        # Buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill='x', pady=(10, 0))
        
        def select_all():
            for var in self.program_vars:
                var.set(True)
        
        def select_none():
            for var in self.program_vars:
                var.set(False)
        
        def delete_selected():
            selected_paths = []
            for i, var in enumerate(self.program_vars):
                if var.get():
                    selected_paths.append(self.program_paths[i])
            
            if not selected_paths:
                messagebox.showinfo("No Selection", "Please select programs to delete")
                return
            
            # Confirm deletion
            result = messagebox.askyesno(
                "Confirm Delete", 
                f"Are you sure you want to delete {len(selected_paths)} program(s)?\n\n"
                f"This action cannot be undone!"
            )
            if not result:
                return
            
            # Delete programs
            def delete_thread():
                try:
                    for file_path in selected_paths:
                        self.ssh_client.exec_command(f"rm '{file_path}'")
                    
                    # Refresh program list and remote tree
                    self.root.after(0, self.refresh_program_list)
                    self.root.after(0, self.refresh_remote_tree)
                    self.root.after(0, lambda: messagebox.showinfo(
                        "Delete Complete", 
                        f"Successfully deleted {len(selected_paths)} programs"
                    ))
                    self.root.after(0, dialog.destroy)
                    
                except Exception as e:
                    self.root.after(0, lambda: messagebox.showerror(
                        "Delete Error", f"Delete failed: {e}"
                    ))
            
            threading.Thread(target=delete_thread, daemon=True).start()
        
        ttk.Button(button_frame, text="Select All", command=select_all).pack(side='left', padx=2)
        ttk.Button(button_frame, text="Select None", command=select_none).pack(side='left', padx=2)
        ttk.Button(button_frame, text="🗑️ Delete Selected", command=delete_selected).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Cancel", command=dialog.destroy).pack(side='right', padx=2)
    
    def show_programs_list(self):
        """Show a list of all programs on the rover"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to SSH server first")
            return
        
        def get_programs_info():
            try:
                # Get detailed program information
                command = "find /home/jay/rover_project -name '*.py' -type f -exec ls -la {} + | sort"
                stdin, stdout, stderr = self.ssh_client.exec_command(command)
                output = stdout.read().decode().strip()
                
                # Create info dialog
                info_dialog = tk.Toplevel(self.root)
                info_dialog.title("Programs on Rover")
                info_dialog.geometry("700x500")
                info_dialog.transient(self.root)
                
                # Text widget with scrollbar
                text_frame = ttk.Frame(info_dialog)
                text_frame.pack(fill='both', expand=True, padx=10, pady=10)
                
                text_widget = scrolledtext.ScrolledText(text_frame, wrap='none', 
                                                       font=('Courier', 10))
                text_widget.pack(fill='both', expand=True)
                
                # Add header
                header = f"Python Programs in /home/jay/rover_project\n"
                header += f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                header += "=" * 60 + "\n\n"
                text_widget.insert('1.0', header)
                
                # Add program listing
                if output:
                    text_widget.insert(tk.END, output)
                else:
                    text_widget.insert(tk.END, "No Python programs found.")
                
                text_widget.config(state='disabled')
                
                # Close button
                ttk.Button(info_dialog, text="Close", 
                          command=info_dialog.destroy).pack(pady=10)
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to get programs list: {e}")
        
        threading.Thread(target=get_programs_info, daemon=True).start()
    
    def exit_application(self):
        """Clean exit of application"""
        if self.connected:
            self.disconnect_ssh()
        self.root.quit()
        self.root.destroy()


def main():
    """Main entry point"""
    try:
        # Check display environment
        print("Checking display environment...")
        display = os.environ.get('DISPLAY', 'Not set')
        print(f"DISPLAY environment variable: {display}")
        
        # Check if we're in a virtual environment without display
        if not display or display == 'Not set':
            print("Warning: No DISPLAY environment variable set")
            print("You may need to run: export DISPLAY=:0")
        
        print("Initializing Tkinter...")
        root = tk.Tk()
        print("Tkinter root window created successfully")
        
        # Try to get screen dimensions
        try:
            screen_width = root.winfo_screenwidth()
            screen_height = root.winfo_screenheight()
            print(f"Screen dimensions: {screen_width}x{screen_height}")
        except Exception as e:
            print(f"Could not get screen dimensions: {e}")
        
        print("Creating Enhanced SSH Dashboard...")
        app = EnhancedSSHDashboard(root)
        print("Dashboard created successfully")
        
        print("Starting main event loop...")
        print("The GUI window should appear on your screen now...")
        root.mainloop()
        print("Main loop ended")
        
    except tk.TclError as e:
        print(f"Tkinter/Display error: {e}")
        print("This usually means:")
        print("1. No X11 display server is running")
        print("2. DISPLAY environment variable is not set")
        print("3. X11 forwarding is not enabled")
        print("Try running: export DISPLAY=:0")
    except Exception as e:
        print(f"Error in main(): {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    print("Starting SSH Dashboard V3...")
    main()
    print("Dashboard closed.")