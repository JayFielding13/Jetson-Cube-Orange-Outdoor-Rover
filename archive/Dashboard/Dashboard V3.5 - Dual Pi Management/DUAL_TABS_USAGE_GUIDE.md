# Dashboard V3.6 - Dual Pi Dedicated Tabs Usage Guide

## 🚀 Quick Start

Launch the enhanced dashboard with separate Pi tabs:

```bash
cd "/home/jay/Desktop/Mini Rover Development/Dashboard/Dashboard V3.5 - Dual Pi Management"
./run_v3_dual_tabs.sh
```

## 🎯 Key Improvements

### Separate Pi Tabs
- **📡 Navigation Pi Tab**: Dedicated file management for 192.168.254.65
- **🎨 Companion Pi Tab**: Dedicated file management for 192.168.254.70  
- **📊 System Monitoring**: Both Pi system monitoring
- **⚙️ Settings**: Configuration and preferences

### Enhanced File Management
- **Independent connections** - Each Pi has its own SSH connection
- **Parallel operations** - Upload to both Pis simultaneously
- **Pi-specific shortcuts** - Quick navigation to common directories
- **Clear organization** - No confusion about which Pi you're working with

## 📋 Dashboard Interface

### Navigation Pi Tab (📡)
**Purpose**: Upload navigation scripts, Arduino code, sensor drivers
**Quick Navigation Buttons:**
- 📡 Pi Code → `/home/jay/rover_project/Pi Code`
- 🤖 Arduino Code → `/home/jay/rover_project/Arduino Code`
- 🏠 Home → `/home/jay`

**Perfect for uploading:**
- `intelligent_rover_wanderer.py`
- `navigation_pi_dual_sensor_controller.py`
- Arduino sensor validation scripts
- Navigation algorithms

### Companion Pi Tab (🎨)
**Purpose**: Upload visualization scripts, data processing, logging
**Quick Navigation Buttons:**
- 🎨 Visualization → `/home/jay/rover_project/Visualization`
- 📊 Data Logs → `/home/jay/rover_project/Data Logs`
- 🏠 Home → `/home/jay`

**Perfect for uploading:**
- `companion_pi_sensor_visualizer.py`
- `dev_pc_visualization_receiver.py`
- Data processing scripts
- Logging utilities

## 🔧 File Operations Per Pi

### Upload Process
1. **Select Pi tab** (Navigation or Companion)
2. **Navigate local files** to your script location
3. **Navigate remote path** to destination folder  
4. **Select local files** you want to upload
5. **Click "📤 Upload Selected"**

### Quick Upload Example
```bash
# Upload wanderer script to Navigation Pi:
1. Click "📡 Navigation Pi" tab
2. Navigate local files to: "Pi Code/Navigation V7 - Dual Pi/"
3. Select "intelligent_rover_wanderer.py"
4. Set remote path to: "/home/jay/rover_project/Pi Code/"
5. Click "📤 Upload Selected"

# Upload visualizer to Companion Pi:
1. Click "🎨 Companion Pi" tab  
2. Navigate local files to: "Pi Code/Navigation V7 - Dual Pi/"
3. Select "companion_pi_sensor_visualizer.py"
4. Set remote path to: "/home/jay/rover_project/Pi Code/"
5. Click "📤 Upload Selected"
```

## 🌐 Connection Management

### Initial Connection
- Dashboard will automatically try to connect to both Pis on startup
- Enter password when prompted (same for both Pis)
- Status bar shows connection status: ✅ Connected / ❌ Disconnected

### Manual Connection
- Use **"Connect"** button in each Pi tab
- Or use **File → Connect to Navigation/Companion Pi**
- Connection status updates in real-time

### Connection Troubleshooting
- Check Pi IP addresses in Settings tab
- Verify network connectivity: `ping 192.168.254.65`
- Ensure SSH is enabled on both Pis
- Check firewall settings if connections fail

## 📊 System Monitoring

### Real-time Status
- **Status bar** shows connection status for both Pis
- **System Monitoring tab** provides detailed system info
- **Health Check** button runs comprehensive diagnostics

### Monitoring Features
- **CPU usage** and temperature monitoring
- **Memory and disk usage** tracking
- **Network connectivity** status
- **Process monitoring** and service health
- **Arduino connection** detection (Navigation Pi)

## 🎮 Deploying Your Rover System

### Step 1: Upload Navigation Scripts
1. Go to **📡 Navigation Pi tab**
2. Upload `intelligent_rover_wanderer.py`
3. Upload any Arduino test scripts
4. Set executable permissions if needed

### Step 2: Upload Visualization Scripts  
1. Go to **🎨 Companion Pi tab**
2. Upload `companion_pi_sensor_visualizer.py`
3. Upload development PC receiver script
4. Install Python dependencies if needed

### Step 3: Test System
1. SSH to Navigation Pi and run: `python3 intelligent_rover_wanderer.py`
2. SSH to Companion Pi and run: `python3 companion_pi_sensor_visualizer.py`
3. Run on Dev PC: `python3 dev_pc_visualization_receiver.py`

## 🔧 Advanced Features

### File Operations
- **📤 Upload Selected**: Transfer files to Pi
- **📥 Download Selected**: Get files from Pi  
- **🗑️ Delete Selected**: Remove files from Pi
- **Batch operations**: Select multiple files

### Quick Navigation
- **Pi-specific shortcuts** for common directories
- **Breadcrumb navigation** in path bars
- **Double-click folders** to navigate
- **Right-click menus** for advanced operations

### Connection Settings
- **Settings tab** allows IP address customization
- **Connection timeout** configuration
- **Automatic reconnection** options
- **Password caching** for convenience

## 💡 Best Practices

### File Organization
- Keep scripts organized by function (navigation vs visualization)
- Use clear file names that indicate purpose
- Backup important scripts before overwriting
- Test scripts locally before uploading

### Development Workflow
1. **Develop locally** with your favorite IDE
2. **Test scripts** in local environment
3. **Upload to appropriate Pi** using dashboard
4. **Test on Pi** via SSH terminal
5. **Monitor performance** using system monitoring

### Troubleshooting
- **Check connections** first if operations fail
- **Verify file paths** are correct on both sides
- **Monitor system resources** during operations
- **Use system monitoring** to diagnose issues

## 📞 Support

### Common Issues
- **Connection timeouts**: Check network connectivity
- **Upload failures**: Verify remote path exists
- **Permission errors**: Check file permissions on Pi
- **Performance issues**: Monitor system resources

### Getting Help
- Check system monitoring tab for diagnostics
- Use health check feature for comprehensive analysis
- Verify network settings in Settings tab
- Restart dashboard if persistent issues occur

---

**Dashboard V3.6 - Dual Pi Dedicated Tabs Edition**  
**Perfect for professional dual Pi rover development!** 🤖🚀