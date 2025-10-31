# Rover Control Dashboard

**Comprehensive GUI for rover monitoring and remote control**

## Features

### 🖥️ Real-Time Monitoring
- **Live rover status** - mode, RC signals, motor speeds
- **Sensor visualization** - ultrasonic distance with graphical display
- **System information** - battery, signal strength, uptime
- **Autonomous state tracking** - roaming status and collision detection

### 🔧 SSH Remote Control
- **One-click SSH connection** to `jay@192.168.254.62`
- **Program launcher** - select and run rover programs remotely
- **Live output monitoring** - see program output in real-time
- **Process management** - start, stop, and monitor running programs

### ⚡ Quick Commands
- **🔄 Reboot Pi** - safely restart the Raspberry Pi
- **📥 Git Pull** - update code from repository
- **📋 List Processes** - see running Python programs
- **📤 Upload Python File** - transfer local Python files to Pi
- **💾 Save Debug Log** - manually save comprehensive debugging log

### 💻 Pi Terminal
- **Command Line Interface** - execute any command on Pi via SSH
- **Virtual Environment Auto-activation** - pip/python commands automatically use venv
- **Command History** - use Up/Down arrows to navigate previous commands
- **Quick Command Buttons** - common operations like `pip list`, `ls -la`, etc.
- **Real-time Output** - see command results immediately

### 📊 Debug Logging System
- **Comprehensive Logging** - captures all rover communications and system events
- **Smart Buffer Management** - keeps last ~10 minutes of data (2000 entries max)
- **Automatic Saves** - logs saved every 3 minutes and on session end
- **Manual Save** - click "💾 Save Debug Log" button anytime
- **Structured Format** - organized logs with timestamps and categories
- **Debug Categories**: CONNECTION, SERIAL_RX, SERIAL_TX, SSH_TX, SSH_RX, COMMAND, TELEMETRY, MODE_CHANGE, NAV_CHANGE, STATE, ERROR, PROGRAM, SYSTEM

**Log File Location**: `/Mini Rover Development/Data Logs/rover_debug_YYYYMMDD_HHMMSS.txt`

**What Gets Logged**:
- All serial communications with Arduino
- SSH commands and responses
- Program launches and stops
- Connection events and errors
- Robot mode and navigation state changes
- Telemetry data updates
- System status snapshots

## Installation

### Windows Setup
```bash
# Install Python dependencies
pip install -r requirements.txt

# Run the dashboard
python rover_control_dashboard.py
```

### Required Dependencies
- `paramiko` - SSH remote control
- `pyserial` - Serial communication with rover
- `tkinter` - GUI framework (built-in with Python)

## Usage

### 1. Serial Connection
The dashboard automatically attempts to connect to the rover via USB serial:
- Tries `/dev/ttyUSB0`, `/dev/ttyACM0`, `/dev/ttyUSB1`
- Shows connection status in **SERIAL CONNECTION** panel
- Displays real-time rover data when connected

### 2. SSH Remote Control
1. **Click "Connect SSH"** - prompts for password and connects to `jay@192.168.254.62`
   - Password dialog appears for secure authentication
   - Shows connection status: CONNECTING → AUTHENTICATING → CONNECTED
   - Clear error messages for authentication failures
2. **Select Program** - choose from dropdown menu:
   - `autonomous_always_thinking.py` (recommended)
   - `autonomous_controller_smooth.py`
   - `autonomous_controller_lidar.py`
   - `autonomous_test_simple.py`
   - And more...
3. **Click "🚀 Launch Program"** - automatically executes:
   ```bash
   ssh jay@192.168.254.62
   cd ~/rover_project
   source venv/bin/activate
   python3 [selected_program]
   ```
4. **Monitor output** - program messages appear in log
5. **Click "🛑 Stop Program"** - cleanly terminate when done

### 3. Pi Terminal Interface
1. **SSH Required** - ensure SSH connection is established first
2. **Type Commands** - enter any command in the terminal input field
3. **Press Enter** - execute command (or click Execute button)
4. **View Output** - command results appear in terminal output area
5. **Use Quick Commands** - click buttons for common operations:
   - **📦 pip list** - show installed packages
   - **📥 pip install [pkg]** - template for installing packages
   - **📁 ls -la** - detailed file listing
   - **📂 pwd** - show current directory
   - **🔍 ps aux** - show running Python processes
6. **Command History** - use ↑/↓ arrow keys to navigate previous commands

**Virtual Environment Auto-Activation:**
- Commands with `pip`, `python`, or `python3` automatically activate the virtual environment
- No need to manually run `source venv/bin/activate`

**Example Commands:**
```bash
pip install psutil           # Install a package
pip list                     # List installed packages
ls -la *.py                  # List Python files
python3 --version            # Check Python version
top                          # Show system processes
df -h                        # Show disk usage
```

### 4. File Upload
1. **Click "📤 Upload Python File"** - opens file browser
2. **Select Python File** - choose .py file from your computer
3. **Confirm Overwrite** - if file exists, choose whether to replace
4. **Auto-Integration** - uploaded files appear in program dropdown
5. **Ready to Launch** - immediately available for execution

### 5. Dashboard Panels

#### Status Row
- **CONTROL MODE** - Current rover mode (FAILSAFE/MANUAL/AUTONOMOUS)
- **SERIAL CONNECTION** - USB connection status
- **RC SIGNAL** - RC transmitter signal validity

#### SSH Control Row
- **SSH CONNECTION** - Remote connection status and controls
- **PROGRAM LAUNCHER** - Program selection and execution
- **QUICK COMMANDS** - System administration tools and file upload

#### Pi Terminal Row
- **COMMAND INPUT** - Terminal command line interface
- **QUICK COMMAND BUTTONS** - Common operations (pip, ls, ps, etc.)
- **TERMINAL OUTPUT** - Real-time command execution results

#### Motor & RC Row
- **MOTOR SPEEDS** - Visual gauges showing left/right motor speeds
- **RC CHANNELS** - Live RC transmitter values (CH1, CH2, CH9)

#### Sensor Row
- **ULTRASONIC SENSOR** - Distance visualization with beam display
- **AUTONOMOUS ROAMING** - Navigation state and collision status
- **SYSTEM INFO** - Battery voltage, signal strength, uptime

#### Pi Telemetry Row
- **LIDAR DATA** - LiDAR points, minimum distance, quality metrics
- **NAVIGATION STATE** - Current navigation state with sector visualization
- **PI SYSTEM** - CPU usage, memory usage, temperature monitoring

#### Message Log
- **Real-time logging** of all rover communications
- **Program output** streaming
- **System messages** and error reporting

## Architecture

### Communication Flow
```
Windows PC → Dashboard → SSH → Raspberry Pi → USB Serial → Arduino → Motors
            ↑                                              ↓
            ← Serial USB ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ←
```

### SSH Workflow Automation
The dashboard replaces multiple manual workflows:

**Program Launching** (via Program Launcher):
```bash
# Manual process (replaced by dashboard)
ssh jay@192.168.254.62
cd ~/rover_project  
source venv/bin/activate
python3 autonomous_always_thinking.py
```

**Command Execution** (via Pi Terminal):
```bash
# Manual process (replaced by terminal interface)
ssh jay@192.168.254.62
cd ~/rover_project && source venv/bin/activate
pip install psutil
```

**File Transfer** (via Upload function):
```bash
# Manual process (replaced by file upload)
scp autonomous_lidar_telemetry.py jay@192.168.254.62:~/rover_project/
```

### Safety Features
- **Confirmation dialogs** for destructive actions (reboot)
- **Connection status monitoring** - clear visual indicators
- **Process isolation** - SSH operations in background threads
- **Clean shutdown** - proper cleanup of connections

## Troubleshooting

### SSH Connection Issues
- Verify Pi is powered on and connected to network
- Check IP address is `192.168.254.62`
- Ensure SSH is enabled on Pi
- Verify correct username/password credentials
- Try manual SSH connection first: `ssh jay@192.168.254.62`

### Serial Connection Issues  
- Check USB cable connection
- Verify Arduino is powered and programmed
- Try different USB ports
- Check Windows Device Manager for COM ports

### Program Launch Issues
- Ensure SSH connection is established first
- Check that virtual environment exists on Pi
- Verify program files exist in `~/rover_project`
- Check log for detailed error messages

## Development

### File Structure
```
/Mini Rover Development/Dashboard/
├── rover_control_dashboard.py  # Main application
├── requirements.txt            # Python dependencies  
└── README.md                  # This documentation
```

### Adding New Programs
Edit the program dropdown in `setup_ssh_control_row()`:
```python
self.program_combo['values'] = [
    'autonomous_always_thinking.py',
    'your_new_program.py',  # Add here
    # ... existing programs
]
```

### Configuration
Update SSH settings in `__init__()`:
```python
self.ssh_config = {
    'host': '192.168.254.62',      # Pi IP address
    'username': 'jay',             # Pi username  
    'work_dir': '~/rover_project', # Work directory
    'venv_activate': 'source venv/bin/activate'  # Virtual env
}
```

---

**Transforms your testing workflow from manual command-line steps to one-click operations!**