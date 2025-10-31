# SSH Dashboard - Linux Edition

**Cross-platform SSH dashboard for Raspberry Pi development - optimized for Ubuntu/Linux**

## 🐧 Linux-Specific Improvements

This Linux version includes the following enhancements over the Windows version:

### ✅ **Key Changes Made**
- **Auto-detects local IP** for X11 forwarding (replaces hardcoded Windows IP)
- **Linux file paths** and conventions  
- **Native Linux GUI** integration
- **Bash launcher script** instead of .bat file
- **Updated window title** - "Linux Edition"
- **Local IP display** in GUI for reference

### 🚀 **Quick Start**

```bash
cd "Dashboard V2 - Linux"
./run.sh
```

Or manually:
```bash
python3 -m venv venv
source venv/bin/activate  
pip install -r requirements.txt
python3 main.py
```

### 📋 **Prerequisites**

**Required:**
- Python 3.7+ (usually pre-installed on Ubuntu)
- tkinter: `sudo apt install python3-tk`
- X11 for GUI apps (usually pre-installed)

**Optional for X11 forwarding:**
- X11 server running locally
- `xhost` command available

### 🔧 **Installation**

1. **Install system dependencies:**
```bash
sudo apt update
sudo apt install python3-tk python3-venv python3-pip
```

2. **Clone/navigate to folder and run:**
```bash
./run.sh
```

### 🎯 **Features**

**All original Windows features plus:**
- ✅ **Automatic IP detection** - No manual IP configuration needed
- ✅ **Native Linux file dialogs** - Uses GTK file chooser
- ✅ **Linux process management** - Better SSH channel handling
- ✅ **X11 forwarding setup** - Automated permission setup
- ✅ **Linux paths** - Uses proper Unix path separators

### 🌐 **X11 Forwarding Setup**

**For GUI apps on Raspberry Pi to display on your Linux desktop:**

1. **Enable X11 on your Linux machine:**
```bash
# Allow X11 connections (run once per session)
xhost +local:
```

2. **The dashboard automatically:**
   - Detects your local IP address
   - Sets up DISPLAY environment variable on Pi
   - Shows your IP in the GUI for reference

3. **Test X11 with the "Test X11 Display" button**

### 🔍 **Troubleshooting**

**Can't connect to Raspberry Pi:**
- Check IP address and SSH credentials
- Ensure Pi has SSH enabled: `sudo systemctl enable ssh`

**GUI doesn't appear:**
```bash
# Install tkinter if missing
sudo apt install python3-tk

# Check if X11 is running
echo $DISPLAY
```

**X11 forwarding not working:**
```bash
# On your Linux machine
xhost +local:

# Check if X11 server is running
ps aux | grep Xorg
```

**Permission denied errors:**
```bash
# Make script executable
chmod +x run.sh

# Install pip packages globally if venv fails
pip3 install --user paramiko scp
```

### 📁 **File Structure**

```
Dashboard V2 - Linux/
├── main.py           # Main dashboard application (Linux optimized)
├── requirements.txt  # Python dependencies  
├── run.sh           # Linux launcher script (executable)
└── README.md        # This file
```

### 🆚 **Differences from Windows Version**

| Feature | Windows Version | Linux Version |
|---------|----------------|---------------|
| **X11 IP** | Hardcoded `192.168.254.14:0` | Auto-detected local IP |
| **Launcher** | `run.bat` | `run.sh` (executable) |
| **File Dialogs** | Windows native | GTK/Qt native |
| **Paths** | Windows paths | Unix paths |
| **Process Control** | Windows signals | Linux signals |

### 🔧 **Configuration**

**Default settings work for most setups:**
- Host: `192.168.254.65` (update for your Pi)
- Port: `22` (standard SSH)
- Username: `jay` (update for your user)
- Remote Path: `/home/jay/rover_project/`

**Customize in the GUI or edit the code defaults.**

### 📊 **Features Overview**

- 🔗 **SSH Connection** - Secure connection to Raspberry Pi
- 📁 **File Upload** - Upload Python files via SCP
- ⚡ **Quick Run** - Execute uploaded files instantly  
- 🖥️ **Terminal** - Interactive SSH terminal
- 🎨 **Themes** - Light/Dark theme toggle
- 📋 **Logging** - Save terminal sessions to files
- ⏹️ **Process Control** - Stop running programs remotely
- 🖼️ **X11 Support** - Forward GUI apps to your Linux desktop

### 🤖 **Perfect for Rover Development**

This dashboard is ideal for:
- Uploading rover control scripts
- Running autonomous navigation code
- Monitoring rover telemetry in real-time
- Testing sensor interfaces remotely
- Debugging robot behavior over SSH

---

**Ready to control your rover from Linux! 🚀🤖**