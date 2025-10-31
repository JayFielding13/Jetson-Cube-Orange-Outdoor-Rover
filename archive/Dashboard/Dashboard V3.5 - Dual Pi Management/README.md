# SSH Dashboard V3.5 - Dual Pi Management Edition

🤖 **Advanced dual Pi management dashboard for comprehensive rover system control!**

## ✨ New Features in V3.5

### 🤖 Dual Pi Architecture Support
- **Navigation Pi (192.168.254.65)** - Arduino communication, navigation, safety systems
- **Companion Pi (192.168.254.70)** - Visualization, data relay, logging, LiDAR processing
- **Simultaneous connections** - Connect to both Pi systems at once
- **Active Pi selection** - Choose which Pi to use for file operations
- **Inter-Pi communication testing** - Verify connectivity between Pi systems

### 📊 Enhanced System Monitoring
- **Dedicated monitoring tab** - Separate tab with dual side-by-side terminals
- **Individual Pi terminals** - Navigation Pi and Companion Pi terminals
- **Real-time monitoring** - Live output from both Pi systems simultaneously  
- **Individual logging** - Save separate logs for each Pi system
- **Connection health indicators** - Visual status indicators for each Pi

### 🔬 Phase 1 Testing Framework
- **Arduino connection testing** - Verify Arduino USB device detection
- **Motor system diagnostics** - Test motor controller functionality
- **LiDAR system verification** - Check LiDAR system status
- **Inter-Pi communication** - Test network connectivity between Pis
- **System health checks** - Comprehensive system diagnostics

### 🌳 Enhanced File Management
- **Multi-Pi file browsing** - Browse files on both Pi systems
- **Cross-Pi file operations** - Copy files between Pi systems
- **Program launcher** - Run programs on selected Pi
- **Synchronized operations** - Coordinate actions across both systems

## 🏗️ Architecture Overview

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Arduino       │◄──►│  Navigation Pi  │◄──►│  Companion Pi   │
│   (Safety)      │    │  (Brain)        │    │  (Eyes)         │
└─────────────────┘    └─────────────────┘    └─────────────────┘
        ▲                       ▲                       ▲
        │                       │                       │
   Motors/Sensors         Navigation Logic      Visualization/Relay
                                │                       │
                                └─────────┬─────────────┘
                                          ▼
                                ┌─────────────────┐
                                │ Development PC  │
                                │   (Dashboard)   │
                                └─────────────────┘
```

### Data Flow
1. **LiDAR → Navigation Pi** - Direct sensor connection for real-time navigation
2. **Navigation Pi → Companion Pi** - Processed navigation data and visualization requests
3. **Companion Pi → Dev PC** - Visualization data, system logs, diagnostic information
4. **Arduino ↔ Navigation Pi** - Motor commands and sensor data (safety gatekeeper)

## 🚀 Quick Start

1. **Launch the dual Pi dashboard:**
   ```bash
   cd "Dashboard V3.5 - Dual Pi Management"
   ./run_v3.sh
   ```

2. **Connect to both Pi systems:**
   - Enter password for both Pi systems
   - Click "🔗 Connect Both" for simultaneous connection
   - Or connect individually using Pi-specific buttons

3. **Test system integration:**
   - Click "🔄 Test Inter-Pi Comm" to verify connectivity
   - Use "📡 Check Arduino" on Navigation Pi
   - Use "👁️ Check LiDAR" on Companion Pi
   - Click "📊 Show Monitoring" to open dual terminal monitoring

4. **Use the enhanced interface:**
   - **Dual Pi Management** - Connection setup and Navigation Pi program control
   - **System Monitoring** - Side-by-side terminals for both Pi systems
   - **File Management** - Cross-Pi file operations and program management

## 📋 Key Improvements Over V3

| Feature | V3 (Single Pi) | V3.5 (Dual Pi) |
|---------|----------------|-----------------|
| Connection Management | Single SSH connection | Dual Pi simultaneous connections |
| Architecture Support | Monolithic | Distributed Pi architecture |
| System Monitoring | Basic connection status | Real-time dual Pi monitoring |
| Testing Framework | None | Phase 1 Arduino/Pi integration tests |
| Data Flow | Single Pi operations | Multi-Pi coordinated operations |
| Diagnostics | Limited | Comprehensive inter-Pi diagnostics |

## 🔧 Phase 1 Testing Tools

### Navigation Pi Tests
- **🧭 Arduino Connection** - Verify Arduino USB device presence
- **🚗 Motor Systems** - Test motor controller connectivity
- **📡 System Health** - Monitor navigation system resources
- **🔗 Network Connectivity** - Test connection to companion Pi

### Companion Pi Tests  
- **👁️ LiDAR System** - Verify LiDAR sensor connectivity
- **📊 System Resources** - Monitor visualization system performance
- **🔗 Network Connectivity** - Test connection to navigation Pi
- **📈 Data Relay** - Test data forwarding capabilities

### Inter-Pi Communication
- **🔄 Ping Tests** - Verify network connectivity between Pis
- **📡 Data Transfer** - Test file transfer capabilities
- **🤝 Coordination** - Test synchronized operations
- **⚡ Latency Testing** - Measure communication delays

## 🎯 Testing Strategy

### Phase 1: Foundation Layer
✅ **Arduino/Pi Communication Testing**
- Verify Arduino safety gatekeeper functions
- Test emergency stop and override capabilities
- Validate serial communication protocols

✅ **Inter-Pi Network Testing** 
- Test network connectivity between Pi systems
- Verify data transfer capabilities
- Measure communication latency and reliability

### Phase 2: System Integration (Coming Next)
- Motor control and movement testing
- Sensor data integration and validation
- Basic navigation behavior testing
- LiDAR data processing and relay

## 🛠️ Advanced Features

### Active Pi Management
- **Pi Selection** - Choose active Pi for file operations and program execution
- **Context Switching** - Seamlessly switch between Pi systems
- **Legacy Compatibility** - Maintains compatibility with single-Pi operations

### System Status Dashboard
- **Real-time Monitoring** - Live status updates from both Pi systems
- **Error Tracking** - Comprehensive error logging and reporting
- **Performance Metrics** - System resource usage and performance monitoring

### Testing Automation
- **Automated Test Suites** - Run comprehensive system tests
- **Diagnostic Reports** - Generate detailed system health reports
- **Integration Validation** - Verify multi-system integration

## 🔮 Coming in Future Versions

- **🎥 Real-time LiDAR Visualization** - Live LiDAR data streaming and display
- **📊 Performance Dashboards** - Advanced system monitoring and analytics
- **🤖 Automated System Recovery** - Automatic error detection and recovery
- **🔄 Load Balancing** - Dynamic task distribution between Pi systems
- **📱 Mobile Dashboard** - Remote monitoring and control capabilities

## 📝 Usage Tips

1. **Always test inter-Pi communication first** - Ensures both systems can coordinate
2. **Use Navigation Pi for critical operations** - It handles safety-critical functions
3. **Monitor system status regularly** - Watch for connection issues or resource constraints
4. **Test Arduino connection before rover operations** - Verify safety systems are functional

---
**Dashboard V3.5 - Dual Pi Management Edition**  
*Building robust, scalable robotics architectures one Pi at a time!* 🤖✨

## 🐛 Troubleshooting

### Connection Issues
- Verify both Pi systems are powered and accessible
- Check network connectivity between Pi systems
- Ensure SSH is enabled on both Pi systems

### Inter-Pi Communication Failures
- Test individual Pi connections first
- Verify network configuration and routing
- Check firewall settings on both systems

### Performance Issues
- Monitor system resources on both Pi systems
- Consider load distribution between systems
- Check network bandwidth and latency