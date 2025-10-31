import tkinter as tk
from tkinter import ttk, messagebox, filedialog, scrolledtext
import paramiko
from scp import SCPClient
import threading
import os
import io
import time
import socket
from datetime import datetime


class SSHDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("Raspberry Pi SSH Dashboard - Linux Edition (Enhanced)")
        self.root.geometry("1100x750")
        
        # SSH connection variables
        self.ssh_client = None
        self.scp_client = None
        self.connected = False
        self.current_channel = None
        
        # Theme variables
        self.dark_mode = False
        self.setup_themes()
        
        # Setup GUI
        self.setup_gui()
        
        # Handle window close event
        self.root.protocol("WM_DELETE_WINDOW", self.exit_application)
        
        # Last uploaded file/folder tracking
        self.last_uploaded_file = None
        self.last_uploaded_folder = None
        
    def setup_themes(self):
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
        style.configure('TEntry', fieldbackground=theme['entry_bg'], foreground=theme['entry_fg'],
                       bordercolor=theme['select_bg'], lightcolor=theme['select_bg'],
                       darkcolor=theme['select_bg'])
        
        # Configure button styles
        style.configure('TButton', background=theme['button_bg'], foreground=theme['button_fg'],
                       bordercolor=theme['select_bg'], lightcolor=theme['button_bg'],
                       darkcolor=theme['button_bg'])
        style.map('TButton', 
                 background=[('active', theme['select_bg'])],
                 foreground=[('active', theme['select_fg'])])
        
    def toggle_theme(self):
        self.dark_mode = not self.dark_mode
        self.apply_theme()
        
        # Update terminal colors if it exists
        if hasattr(self, 'terminal_output'):
            theme = self.dark_theme if self.dark_mode else self.light_theme
            self.terminal_output.configure(
                bg=theme['terminal_bg'],
                fg=theme['terminal_fg'],
                selectbackground=theme['select_bg'],
                selectforeground=theme['select_fg'],
                insertbackground=theme['fg']
            )
        
        # Update theme button text
        if hasattr(self, 'theme_btn'):
            self.theme_btn.configure(text="🌙 Dark" if not self.dark_mode else "☀️ Light")
    
    def get_local_ip(self):
        """Get the local IP address of this Linux machine for X11 forwarding"""
        try:
            # Connect to a remote address to determine local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            return local_ip
        except Exception:
            # Fallback to localhost
            return "127.0.0.1"
        
    def setup_gui(self):
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)
        
        # Connection frame
        conn_frame = ttk.LabelFrame(main_frame, text="Connection Settings", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Connection fields
        ttk.Label(conn_frame, text="Host:").grid(row=0, column=0, sticky=tk.W, padx=(0, 5))
        self.host_var = tk.StringVar(value="192.168.254.65")
        self.host_entry = ttk.Entry(conn_frame, textvariable=self.host_var, width=20)
        self.host_entry.grid(row=0, column=1, padx=(0, 10))
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=2, sticky=tk.W, padx=(0, 5))
        self.port_var = tk.StringVar(value="22")
        self.port_entry = ttk.Entry(conn_frame, textvariable=self.port_var, width=10)
        self.port_entry.grid(row=0, column=3, padx=(0, 10))
        
        ttk.Label(conn_frame, text="Username:").grid(row=1, column=0, sticky=tk.W, padx=(0, 5))
        self.username_var = tk.StringVar(value="jay")
        self.username_entry = ttk.Entry(conn_frame, textvariable=self.username_var, width=20)
        self.username_entry.grid(row=1, column=1, padx=(0, 10))
        
        ttk.Label(conn_frame, text="Password:").grid(row=1, column=2, sticky=tk.W, padx=(0, 5))
        self.password_var = tk.StringVar()
        self.password_entry = ttk.Entry(conn_frame, textvariable=self.password_var, width=20, show="*")
        self.password_entry.grid(row=1, column=3, padx=(0, 10))
        
        # Connect button
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.connect_ssh)
        self.connect_btn.grid(row=0, column=4, rowspan=2, padx=(10, 0))
        
        # Theme toggle button
        self.theme_btn = ttk.Button(conn_frame, text="🌙 Dark", command=self.toggle_theme)
        self.theme_btn.grid(row=0, column=5, rowspan=2, padx=(10, 0))
        
        # Stop program button
        self.stop_btn = ttk.Button(conn_frame, text="⏹️ Stop", command=self.stop_remote_program, state="disabled")
        self.stop_btn.grid(row=0, column=6, rowspan=2, padx=(10, 0))
        
        # Status label
        self.status_var = tk.StringVar(value="Disconnected")
        self.status_label = ttk.Label(conn_frame, textvariable=self.status_var)
        self.status_label.grid(row=2, column=0, columnspan=7, pady=(5, 0))
        
        # File operations frame
        file_frame = ttk.LabelFrame(main_frame, text="File Operations", padding="5")
        file_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        file_frame.columnconfigure(0, weight=1)
        
        # Upload file button
        self.upload_file_btn = ttk.Button(file_frame, text="📄 Upload Python File", 
                                         command=self.upload_file, state="disabled")
        self.upload_file_btn.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # NEW: Upload folder button
        self.upload_folder_btn = ttk.Button(file_frame, text="📁 Upload Folder", 
                                           command=self.upload_folder, state="disabled")
        self.upload_folder_btn.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # NEW: Upload Modular Pi Code button (quick shortcut)
        self.upload_modular_btn = ttk.Button(file_frame, text="🤖 Upload Modular Pi Code", 
                                            command=self.upload_modular_code, state="disabled")
        self.upload_modular_btn.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # Remote path entry
        ttk.Label(file_frame, text="Remote Path:").grid(row=3, column=0, sticky=tk.W)
        self.remote_path_var = tk.StringVar(value="/home/jay/rover_project/")
        self.remote_path_entry = ttk.Entry(file_frame, textvariable=self.remote_path_var)
        self.remote_path_entry.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # Quick run button
        self.run_btn = ttk.Button(file_frame, text="▶️ Run Last Uploaded File", 
                                command=self.run_python_file, state="disabled")
        self.run_btn.grid(row=5, column=0, sticky=(tk.W, tk.E))
        
        # Activate environment button
        self.activate_btn = ttk.Button(file_frame, text="🔧 Activate Rover Environment", 
                                     command=self.activate_environment, state="disabled")
        self.activate_btn.grid(row=6, column=0, sticky=(tk.W, tk.E), pady=(5, 0))
        
        # Test X11 button
        self.test_x11_btn = ttk.Button(file_frame, text="🖥️ Test X11 Display", 
                                     command=self.test_x11_display, state="disabled")
        self.test_x11_btn.grid(row=7, column=0, sticky=(tk.W, tk.E), pady=(5, 0))
        
        # Terminal frame
        terminal_frame = ttk.LabelFrame(main_frame, text="Terminal", padding="5")
        terminal_frame.grid(row=1, column=1, rowspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(10, 0))
        terminal_frame.columnconfigure(0, weight=1)
        terminal_frame.rowconfigure(0, weight=1)
        
        # Terminal output
        theme = self.dark_theme if self.dark_mode else self.light_theme
        self.terminal_output = scrolledtext.ScrolledText(
            terminal_frame, height=20, width=60,
            bg=theme['terminal_bg'], fg=theme['terminal_fg'],
            selectbackground=theme['select_bg'], selectforeground=theme['select_fg'],
            insertbackground=theme['fg']
        )
        self.terminal_output.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 5))
        
        # Add copy support for terminal output (Ctrl+C to copy selected text)
        self.terminal_output.bind('<Control-c>', self.copy_terminal_selection)
        self.terminal_output.bind('<Control-a>', self.select_all_terminal)
        
        # Command entry
        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(terminal_frame, textvariable=self.command_var)
        self.command_entry.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        self.command_entry.bind('<Return>', self.execute_command)
        
        # Add copy/paste keyboard shortcuts for command entry
        self.command_entry.bind('<Control-c>', self.copy_command)
        self.command_entry.bind('<Control-v>', self.paste_command)
        self.command_entry.bind('<Control-a>', self.select_all_command)
        
        # Execute button
        self.execute_btn = ttk.Button(terminal_frame, text="Execute", 
                                    command=self.execute_command, state="disabled")
        self.execute_btn.grid(row=2, column=0, sticky=(tk.W, tk.E))
        
        # Clear terminal button
        self.clear_btn = ttk.Button(terminal_frame, text="Clear Terminal", 
                                  command=self.clear_terminal)
        self.clear_btn.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(5, 0))
        
        # Save logs button
        self.save_logs_btn = ttk.Button(terminal_frame, text="Save Terminal Logs", 
                                      command=self.save_terminal_logs)
        self.save_logs_btn.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=(5, 0))
        
        # Verbose mode checkbox
        self.verbose_var = tk.BooleanVar(value=True)
        self.verbose_check = ttk.Checkbutton(terminal_frame, text="Verbose Output", 
                                           variable=self.verbose_var)
        self.verbose_check.grid(row=5, column=0, sticky=tk.W, pady=(5, 0))
        
        # X11 mode checkbox
        self.x11_var = tk.BooleanVar(value=True)
        self.x11_check = ttk.Checkbutton(terminal_frame, text="Enable X11 Forwarding", 
                                       variable=self.x11_var)
        self.x11_check.grid(row=6, column=0, sticky=tk.W, pady=(5, 0))
        
        # Display local IP for X11 reference
        local_ip = self.get_local_ip()
        self.ip_label = ttk.Label(terminal_frame, text=f"Local IP: {local_ip} (for X11)", 
                                font=('TkDefaultFont', 8))
        self.ip_label.grid(row=7, column=0, sticky=tk.W, pady=(5, 0))
        
    def connect_ssh(self):
        if self.connected:
            self.disconnect_ssh()
            return
            
        host = self.host_var.get()
        port = int(self.port_var.get())
        username = self.username_var.get()
        password = self.password_var.get()
        
        if not all([host, username, password]):
            messagebox.showerror("Error", "Please fill in all connection fields")
            return
            
        def connect_thread():
            try:
                self.ssh_client = paramiko.SSHClient()
                self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                
                # Connect with X11 forwarding enabled
                self.ssh_client.connect(
                    host, port=port, username=username, password=password,
                    allow_agent=False, look_for_keys=False
                )
                
                # Enable X11 forwarding properly
                transport = self.ssh_client.get_transport()
                transport.set_keepalive(30)
                self.scp_client = SCPClient(self.ssh_client.get_transport())
                
                self.root.after(0, self.on_connection_success)
                
            except Exception as e:
                error_msg = str(e)
                self.root.after(0, lambda: self.on_connection_error(error_msg))
        
        self.status_var.set("Connecting...")
        self.connect_btn.config(state="disabled")
        threading.Thread(target=connect_thread, daemon=True).start()
        
    def on_connection_success(self):
        self.connected = True
        self.status_var.set("Connected")
        self.connect_btn.config(text="Disconnect", state="normal")
        self.upload_file_btn.config(state="normal")
        self.upload_folder_btn.config(state="normal")  # NEW
        self.upload_modular_btn.config(state="normal")  # NEW
        self.execute_btn.config(state="normal")
        self.activate_btn.config(state="normal")
        self.stop_btn.config(state="normal")
        self.test_x11_btn.config(state="normal")
        self.terminal_output.insert(tk.END, "Connected to Raspberry Pi (Enhanced Linux Edition)\n")
        self.terminal_output.see(tk.END)
        
    def on_connection_error(self, error_msg):
        self.status_var.set("Connection Failed")
        self.connect_btn.config(state="normal")
        messagebox.showerror("Connection Error", f"Failed to connect: {error_msg}")
        
    def disconnect_ssh(self):
        if self.scp_client:
            self.scp_client.close()
        if self.ssh_client:
            self.ssh_client.close()
            
        self.connected = False
        self.ssh_client = None
        self.scp_client = None
        self.status_var.set("Disconnected")
        self.connect_btn.config(text="Connect", state="normal")
        self.upload_file_btn.config(state="disabled")
        self.upload_folder_btn.config(state="disabled")  # NEW
        self.upload_modular_btn.config(state="disabled")  # NEW
        self.run_btn.config(state="disabled")
        self.execute_btn.config(state="disabled")
        self.activate_btn.config(state="disabled")
        self.stop_btn.config(state="disabled")
        self.test_x11_btn.config(state="disabled")
        self.terminal_output.insert(tk.END, "Disconnected from Raspberry Pi\n")
        self.terminal_output.see(tk.END)
        
    def upload_file(self):
        """Upload a single Python file"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected to Raspberry Pi")
            return
            
        file_path = filedialog.askopenfilename(
            title="Select Python File",
            filetypes=[("Python files", "*.py"), ("All files", "*.*")]
        )
        
        if file_path:
            def upload_thread():
                try:
                    filename = os.path.basename(file_path)
                    remote_path = self.remote_path_var.get() + filename
                    
                    self.scp_client.put(file_path, remote_path)
                    self.last_uploaded_file = remote_path
                    
                    self.root.after(0, lambda: self.on_upload_success(filename, remote_path))
                    
                except Exception as e:
                    error_msg = str(e)
                    self.root.after(0, lambda: self.on_upload_error(error_msg))
            
            self.terminal_output.insert(tk.END, f"Uploading {os.path.basename(file_path)}...\n")
            self.terminal_output.see(tk.END)
            threading.Thread(target=upload_thread, daemon=True).start()
    
    def upload_folder(self):
        """NEW: Upload an entire folder structure"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected to Raspberry Pi")
            return
            
        folder_path = filedialog.askdirectory(
            title="Select Folder to Upload"
        )
        
        if folder_path:
            folder_name = os.path.basename(folder_path)
            remote_base_path = self.remote_path_var.get()
            
            # Ask user for confirmation
            response = messagebox.askyesno(
                "Confirm Folder Upload",
                f"Upload folder '{folder_name}' to:\n{remote_base_path}\n\n"
                f"This will upload all files and subdirectories.\nContinue?"
            )
            
            if response:
                def upload_folder_thread():
                    try:
                        self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"📁 Starting folder upload: {folder_name}\n"))
                        self.root.after(0, lambda: self.terminal_output.see(tk.END))
                        
                        # Upload folder recursively
                        files_uploaded = self._upload_folder_recursive(folder_path, remote_base_path)
                        
                        self.last_uploaded_folder = remote_base_path + folder_name
                        
                        self.root.after(0, lambda: self.on_folder_upload_success(folder_name, files_uploaded))
                        
                    except Exception as e:
                        error_msg = str(e)
                        self.root.after(0, lambda: self.on_upload_error(f"Folder upload failed: {error_msg}"))
                
                threading.Thread(target=upload_folder_thread, daemon=True).start()
    
    def _upload_folder_recursive(self, local_path, remote_base_path):
        """Recursively upload folder contents"""
        files_uploaded = 0
        
        for root, dirs, files in os.walk(local_path):
            # Calculate relative path from the original local_path
            rel_path = os.path.relpath(root, local_path)
            if rel_path == '.':
                remote_dir = remote_base_path + os.path.basename(local_path) + '/'
            else:
                remote_dir = remote_base_path + os.path.basename(local_path) + '/' + rel_path.replace('\\', '/') + '/'
            
            # Create remote directory
            try:
                self.ssh_client.exec_command(f'mkdir -p "{remote_dir}"')
            except Exception as e:
                print(f"Warning: Could not create directory {remote_dir}: {e}")
            
            # Upload files in this directory
            for file in files:
                local_file_path = os.path.join(root, file)
                remote_file_path = remote_dir + file
                
                try:
                    self.scp_client.put(local_file_path, remote_file_path)
                    files_uploaded += 1
                    
                    # Update terminal with progress
                    self.root.after(0, lambda f=file, d=remote_dir: 
                                   self.terminal_output.insert(tk.END, f"  ✅ {f} -> {d}\n"))
                    self.root.after(0, lambda: self.terminal_output.see(tk.END))
                    
                except Exception as e:
                    self.root.after(0, lambda f=file, err=str(e): 
                                   self.terminal_output.insert(tk.END, f"  ❌ Failed to upload {f}: {err}\n"))
                    self.root.after(0, lambda: self.terminal_output.see(tk.END))
        
        return files_uploaded
    
    def upload_modular_code(self):
        """NEW: Quick shortcut to upload the Modular Pi Code folder"""
        if not self.connected:
            messagebox.showerror("Error", "Not connected to Raspberry Pi")
            return
        
        # Try to find the Modular Pi Code folder automatically
        possible_paths = [
            "/home/jay/Desktop/Mini Rover Development/Modular Pi Code",
            os.path.expanduser("~/Desktop/Mini Rover Development/Modular Pi Code"),
            "./Modular Pi Code",
            "../Modular Pi Code"
        ]
        
        modular_code_path = None
        for path in possible_paths:
            if os.path.exists(path):
                modular_code_path = path
                break
        
        if not modular_code_path:
            # If not found, let user select it
            modular_code_path = filedialog.askdirectory(
                title="Select Modular Pi Code Folder",
                initialdir="/home/jay/Desktop"
            )
        
        if modular_code_path:
            # Confirm the upload
            response = messagebox.askyesno(
                "Upload Modular Pi Code",
                f"Upload the complete modular rover system?\n\n"
                f"From: {modular_code_path}\n"
                f"To: {self.remote_path_var.get()}modular_code/\n\n"
                f"This includes:\n"
                f"• Main/ (Arduino interface, motors, config)\n"
                f"• Sensors/ (Ultrasonic sensor)\n"
                f"• Navigation/ (Obstacle avoidance)\n"
                f"• Autonomous Behaviors/ (Exploration)\n"
                f"• Test programs\n\n"
                f"Continue?"
            )
            
            if response:
                def upload_modular_thread():
                    try:
                        self.root.after(0, lambda: self.terminal_output.insert(tk.END, "🤖 Uploading Modular Pi Code...\n"))
                        self.root.after(0, lambda: self.terminal_output.see(tk.END))
                        
                        # Set remote path to include modular_code subdirectory
                        remote_path = self.remote_path_var.get() + "modular_code/"
                        
                        # Upload the entire folder structure
                        files_uploaded = self._upload_folder_recursive(modular_code_path, self.remote_path_var.get())
                        
                        self.last_uploaded_folder = remote_path
                        
                        # Also set the main test file as last_uploaded_file for easy running
                        self.last_uploaded_file = remote_path + "rover_exploration_test.py"
                        
                        self.root.after(0, lambda: self.terminal_output.insert(tk.END, 
                                       f"🎉 Modular Pi Code upload complete!\n"
                                       f"📊 Files uploaded: {files_uploaded}\n"
                                       f"📍 Location: {remote_path}\n"
                                       f"🚀 Ready to run: rover_exploration_test.py\n"))
                        self.root.after(0, lambda: self.terminal_output.see(tk.END))
                        
                        # Enable the run button
                        self.root.after(0, lambda: self.run_btn.config(state="normal"))
                        
                    except Exception as e:
                        error_msg = str(e)
                        self.root.after(0, lambda: self.on_upload_error(f"Modular code upload failed: {error_msg}"))
                
                threading.Thread(target=upload_modular_thread, daemon=True).start()
    
    def on_upload_success(self, filename, remote_path):
        self.terminal_output.insert(tk.END, f"✅ Successfully uploaded {filename} to {remote_path}\n")
        self.terminal_output.see(tk.END)
        self.run_btn.config(state="normal")
        
    def on_folder_upload_success(self, folder_name, files_count):
        self.terminal_output.insert(tk.END, f"🎉 Successfully uploaded folder '{folder_name}'\n")
        self.terminal_output.insert(tk.END, f"📊 Total files uploaded: {files_count}\n")
        self.terminal_output.see(tk.END)
        
    def on_upload_error(self, error_msg):
        self.terminal_output.insert(tk.END, f"❌ Upload failed: {error_msg}\n")
        self.terminal_output.see(tk.END)
        
    def run_python_file(self):
        if not self.connected or not self.last_uploaded_file:
            messagebox.showerror("Error", "No file uploaded or not connected")
            return
            
        if self.verbose_var.get():
            command = f"cd ~/rover_project && source venv/bin/activate && echo 'Starting program...' && python3 -u {self.last_uploaded_file}"
        else:
            command = f"cd ~/rover_project && source venv/bin/activate && python3 {self.last_uploaded_file}"
        self.execute_command_internal(command)
        
    def activate_environment(self):
        if not self.connected:
            messagebox.showerror("Error", "Not connected to Raspberry Pi")
            return
            
        command = "cd ~/rover_project && source venv/bin/activate && echo 'Rover environment activated' && pwd && which python"
        self.execute_command_internal(command)
        
    def test_x11_display(self):
        if not self.connected:
            messagebox.showerror("Error", "Not connected to Raspberry Pi")
            return
            
        # Use the regular command execution with X11 test
        test_command = "echo 'Testing X11...' && echo 'DISPLAY: '$DISPLAY && timeout 5 xclock"
        self.execute_command_internal(test_command)
        
    def stop_remote_program(self):
        if not self.connected:
            messagebox.showerror("Error", "Not connected to Raspberry Pi")
            return
            
        def stop_thread():
            try:
                # Multiple stop strategies for better reliability
                stop_successful = False
                
                # Strategy 1: Send Ctrl+C to current channel
                if self.current_channel and not self.current_channel.closed:
                    try:
                        self.current_channel.send('\x03')  # Ctrl+C character
                        self.root.after(0, lambda: self.terminal_output.insert(tk.END, "\n^C (Interrupt signal sent)\n"))
                        stop_successful = True
                    except Exception as e:
                        self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"\nChannel interrupt failed: {str(e)}\n"))
                
                # Strategy 2: Kill Python processes by name
                try:
                    stdin, stdout, stderr = self.ssh_client.exec_command("pkill -SIGTERM -f 'python.*\\.py'")
                    result = stdout.read().decode()
                    self.root.after(0, lambda: self.terminal_output.insert(tk.END, "SIGTERM sent to Python processes\n"))
                    stop_successful = True
                except Exception as e:
                    self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"SIGTERM failed: {str(e)}\n"))
                
                # Strategy 3: Force kill if others fail
                if not stop_successful:
                    try:
                        stdin, stdout, stderr = self.ssh_client.exec_command("pkill -SIGKILL -f 'python.*\\.py'")
                        self.root.after(0, lambda: self.terminal_output.insert(tk.END, "SIGKILL sent to Python processes (force kill)\n"))
                    except Exception as e:
                        self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"Force kill failed: {str(e)}\n"))
                
                # Strategy 4: Show running processes for verification
                try:
                    stdin, stdout, stderr = self.ssh_client.exec_command("ps aux | grep python | grep -v grep")
                    processes = stdout.read().decode().strip()
                    if processes:
                        self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"Remaining Python processes:\n{processes}\n"))
                    else:
                        self.root.after(0, lambda: self.terminal_output.insert(tk.END, "No Python processes running\n"))
                except Exception as e:
                    self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"Process check failed: {str(e)}\n"))
                
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
                
            except Exception as e:
                self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"Stop operation failed: {str(e)}\n"))
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
        
        # Run stop operation in separate thread
        self.terminal_output.insert(tk.END, "\nAttempting to stop program...\n")
        self.terminal_output.see(tk.END)
        threading.Thread(target=stop_thread, daemon=True).start()
        
    def execute_command(self, event=None):
        command = self.command_var.get().strip()
        if command:
            self.execute_command_internal(command)
            self.command_var.set("")
            
    def execute_command_internal(self, command):
        if not self.connected:
            messagebox.showerror("Error", "Not connected to Raspberry Pi")
            return
            
        def execute_thread():
            try:
                self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"$ {command}\n"))
                
                # Prepare command with X11 setup if enabled
                if self.x11_var.get():
                    # Use auto-detected local IP for X11 forwarding
                    local_ip = self.get_local_ip()
                    x11_setup = f"source ~/.bashrc && export DISPLAY={local_ip}:0"
                    command_with_x11 = f"{x11_setup} && {command}"
                else:
                    command_with_x11 = command
                
                stdin, stdout, stderr = self.ssh_client.exec_command(command_with_x11, get_pty=True)
                self.current_channel = stdout.channel
                
                # Read output in real-time with better buffering
                import select
                import time
                
                # Set non-blocking mode for better real-time output
                stdout.channel.settimeout(0.1)
                stderr.channel.settimeout(0.1)
                
                while not stdout.channel.exit_status_ready():
                    # Check for stdout data
                    if stdout.channel.recv_ready():
                        data = stdout.channel.recv(1024).decode('utf-8', errors='replace')
                        if data:
                            self.root.after(0, lambda d=data: self.terminal_output.insert(tk.END, d))
                            self.root.after(0, lambda: self.terminal_output.see(tk.END))
                    
                    # Check for stderr data
                    if stderr.channel.recv_stderr_ready():
                        error_data = stderr.channel.recv_stderr(1024).decode('utf-8', errors='replace')
                        if error_data:
                            self.root.after(0, lambda d=error_data: self.terminal_output.insert(tk.END, f"[ERROR] {d}"))
                            self.root.after(0, lambda: self.terminal_output.see(tk.END))
                    
                    time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
                # Read any remaining output
                remaining_out = stdout.read().decode('utf-8', errors='replace')
                if remaining_out:
                    self.root.after(0, lambda d=remaining_out: self.terminal_output.insert(tk.END, d))
                
                remaining_err = stderr.read().decode('utf-8', errors='replace')
                if remaining_err:
                    self.root.after(0, lambda d=remaining_err: self.terminal_output.insert(tk.END, f"[ERROR] {d}"))
                    
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
                
            except Exception as e:
                error_msg = str(e)
                self.root.after(0, lambda: self.terminal_output.insert(tk.END, f"Command failed: {error_msg}\n"))
                self.root.after(0, lambda: self.terminal_output.see(tk.END))
        
        threading.Thread(target=execute_thread, daemon=True).start()
        
    def copy_command(self, event=None):
        """Copy selected text from command entry"""
        try:
            if self.command_entry.selection_present():
                selected_text = self.command_entry.selection_get()
                self.root.clipboard_clear()
                self.root.clipboard_append(selected_text)
                self.root.update()  # Update clipboard
        except tk.TclError:
            # No selection, copy all text
            try:
                all_text = self.command_entry.get()
                if all_text:
                    self.root.clipboard_clear()
                    self.root.clipboard_append(all_text)
                    self.root.update()
            except Exception:
                pass
        return "break"  # Prevent default behavior
    
    def paste_command(self, event=None):
        """Paste text from clipboard to command entry"""
        try:
            clipboard_text = self.root.clipboard_get()
            # Get current cursor position
            cursor_pos = self.command_entry.index(tk.INSERT)
            # Insert clipboard text at cursor position
            current_text = self.command_entry.get()
            new_text = current_text[:cursor_pos] + clipboard_text + current_text[cursor_pos:]
            self.command_var.set(new_text)
            # Move cursor to end of pasted text
            self.command_entry.icursor(cursor_pos + len(clipboard_text))
        except tk.TclError:
            # No clipboard content
            pass
        return "break"  # Prevent default behavior
    
    def select_all_command(self, event=None):
        """Select all text in command entry"""
        self.command_entry.select_range(0, tk.END)
        self.command_entry.icursor(tk.END)
        return "break"  # Prevent default behavior
    
    def copy_terminal_selection(self, event=None):
        """Copy selected text from terminal output"""
        try:
            if self.terminal_output.tag_ranges("sel"):
                selected_text = self.terminal_output.selection_get()
                self.root.clipboard_clear()
                self.root.clipboard_append(selected_text)
                self.root.update()
        except tk.TclError:
            # No selection
            pass
        return "break"
    
    def select_all_terminal(self, event=None):
        """Select all text in terminal output"""
        self.terminal_output.tag_add("sel", "1.0", tk.END)
        return "break"
        
    def clear_terminal(self):
        self.terminal_output.delete(1.0, tk.END)
        
    def save_terminal_logs(self):
        """Save terminal contents to a text file"""
        try:
            # Get current date and time for filename
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            default_filename = f"SSH_Terminal_Log_{timestamp}.txt"
            
            # Ask user where to save the file
            file_path = filedialog.asksaveasfilename(
                title="Save Terminal Logs",
                defaultextension=".txt",
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
                initialfile=default_filename
            )
            
            if file_path:
                # Get all terminal content
                terminal_content = self.terminal_output.get(1.0, tk.END)
                
                # Add header with session info
                header = f"SSH Dashboard Terminal Log - Enhanced Linux Edition\n"
                header += f"Session Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                header += f"Connected to: {self.host_var.get()}:{self.port_var.get()}\n"
                header += f"Username: {self.username_var.get()}\n"
                header += f"Local IP: {self.get_local_ip()}\n"
                header += "=" * 50 + "\n\n"
                
                # Write to file
                with open(file_path, 'w', encoding='utf-8') as f:
                    f.write(header + terminal_content)
                
                messagebox.showinfo("Success", f"Terminal logs saved to:\n{file_path}")
                return True
            return False
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save terminal logs:\n{str(e)}")
            return False
    
    def exit_application(self):
        """Exit application with option to save terminal logs"""
        # Check if there's content in the terminal
        terminal_content = self.terminal_output.get(1.0, tk.END).strip()
        
        if terminal_content:
            # Ask if user wants to save terminal logs
            response = messagebox.askyesnocancel(
                "Save Terminal Logs?",
                "Do you want to save the terminal logs before exiting?\n\n"
                "Yes - Save logs and exit\n"
                "No - Exit without saving\n"
                "Cancel - Return to application"
            )
            
            if response is None:  # Cancel
                return
            elif response:  # Yes - save logs
                if not self.save_terminal_logs():
                    # If saving failed, ask if they still want to exit
                    if not messagebox.askyesno("Exit Anyway?", "Failed to save logs. Exit anyway?"):
                        return
        
        # Disconnect SSH if connected
        if self.connected:
            try:
                self.disconnect_ssh()
            except:
                pass
        
        # Close the application
        self.root.quit()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = SSHDashboard(root)
    root.mainloop()

if __name__ == "__main__":
    main()