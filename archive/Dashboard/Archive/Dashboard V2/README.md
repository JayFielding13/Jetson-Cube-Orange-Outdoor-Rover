# Raspberry Pi SSH Dashboard

A desktop Python application for managing your Raspberry Pi robotics project remotely with X11 forwarding support for GUI applications.

## Features
- SSH connection to Raspberry Pi
- File upload and management
- Remote terminal access
- Quick-run Python scripts
- X11 forwarding for GUI applications (xclock, matplotlib plots, etc.)
- Dark/Light mode themes
- Terminal log saving
- Program stop functionality

## Installation

### Windows Setup
1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Install X11 server for Windows (choose one):
   - **VcXsrv** (recommended, free): https://sourceforge.net/projects/vcxsrv/
   - **Xming** (free): https://sourceforge.net/projects/xming/
   - **X410** (paid, Microsoft Store)

3. Configure X11 server:
   - Set display number to 0
   - Enable "Disable access control"
   - Use "Multiple windows" mode

### Raspberry Pi Setup
For X11 forwarding to work properly, add this line to your Pi's ~/.bashrc:

```bash
echo 'export DISPLAY=<YOUR_WINDOWS_IP>:0' >> ~/.bashrc
```

Replace `<YOUR_WINDOWS_IP>` with your Windows machine's IP address on the local network.

**Example:**
```bash
echo 'export DISPLAY=192.168.1.100:0' >> ~/.bashrc
```

**To find your Windows IP:**
- Open Command Prompt and run: `ipconfig`
- Look for "IPv4 Address" under your active network adapter

## Usage
```bash
python main.py
```

Or use the provided batch file:
```bash
run.bat
```

## Configuration
- Update connection settings in the GUI (default: your Pi's IP, username: jay)
- Default upload path: `/home/jay/rover_project/`
- Enable/disable X11 forwarding with the checkbox
- Toggle dark/light themes
- Enable verbose output for detailed program feedback

## X11 Forwarding Test
Use the "Test X11 Display" button to verify X11 forwarding is working. You should see an xclock window appear on your Windows desktop for 5 seconds.

## Troubleshooting
- If X11 forwarding fails, ensure your Windows X11 server is running and configured correctly
- Verify the DISPLAY variable is set correctly on your Pi
- Check that your Windows firewall allows the X11 server
- For connection issues, verify your Pi's IP address and SSH credentials