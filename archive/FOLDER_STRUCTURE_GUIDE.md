# Platform-Based Folder Structure Guide
**Detailed Directory Organization for 5 Rover Platforms**

**Purpose**: Provide specific folder structure for organizing all rover development work by platform  
**Implementation**: Progressive migration from current structure to platform-based organization

---

## 📁 **COMPLETE FOLDER STRUCTURE**

### **Root Directory: `/Mini Rover Development/`**

```
Mini Rover Development/
├── 01-Outdoor-Rover-Platform/
│   ├── Hardware/
│   │   ├── Cube-Orange-Config/
│   │   │   ├── Parameter-Files/
│   │   │   │   ├── ardurover_obstacle_avoidance_params.txt
│   │   │   │   ├── rtk_gps_params.txt
│   │   │   │   └── mission_planner_full_params.param
│   │   │   ├── Firmware/
│   │   │   │   ├── ArduRover-4.6.2/
│   │   │   │   └── Update-Logs/
│   │   │   ├── Wiring-Diagrams/
│   │   │   └── Calibration-Data/
│   │   │       ├── Compass-Calibration/
│   │   │       ├── EKF-Configuration/
│   │   │       └── Motor-Output-Mapping/
│   │   ├── LIDAR-Setup/
│   │   │   ├── RPLidar-A1-Config/
│   │   │   │   ├── Device-Specs/
│   │   │   │   ├── Mounting-Hardware/
│   │   │   │   └── USB-Connection-Guide/
│   │   │   ├── Calibration/
│   │   │   │   ├── Distance-Accuracy-Tests/
│   │   │   │   ├── Angular-Resolution-Tests/
│   │   │   │   └── Performance-Benchmarks/
│   │   │   └── Integration/
│   │   │       ├── MAVLink-Protocol/
│   │   │       ├── ArduRover-Parameters/
│   │   │       └── Mission-Planner-Setup/
│   │   ├── RTK-GPS-Integration/
│   │   │   ├── Hardware-Setup/
│   │   │   │   ├── ZED-F9P-Configuration/
│   │   │   │   ├── Antenna-Placement/
│   │   │   │   └── Cable-Management/
│   │   │   ├── Base-Station-Connection/
│   │   │   │   ├── RTCM3-Corrections/
│   │   │   │   ├── Network-Setup/
│   │   │   │   └── Range-Testing/
│   │   │   └── Integration-Testing/
│   │   │       ├── Mission-Planner-RTK/
│   │   │       ├── Waypoint-Navigation/
│   │   │       └── Precision-Validation/
│   │   ├── Camera-Systems/
│   │   │   ├── Logitech-C920-Setup/
│   │   │   │   ├── Driver-Installation/
│   │   │   │   ├── Resolution-Settings/
│   │   │   │   └── Mount-Design/
│   │   │   ├── Computer-Vision/
│   │   │   │   ├── OpenCV-Installation/
│   │   │   │   ├── Color-Calibration/
│   │   │   │   └── Object-Detection-Tuning/
│   │   │   └── Integration/
│   │   │       ├── MAVLink-Commands/
│   │   │       ├── LIDAR-Coordination/
│   │   │       └── Real-Time-Performance/
│   │   └── Power-Systems/
│   │       ├── Battery-Management/
│   │       ├── Power-Distribution/
│   │       └── Consumption-Analysis/
│   ├── Software/
│   │   ├── LIDAR-Bridge/
│   │   │   ├── Production/
│   │   │   │   ├── lidar_bridge_v462.py ⭐
│   │   │   │   ├── auto-start-service/
│   │   │   │   │   ├── lidar-bridge.service
│   │   │   │   │   ├── install-service.sh
│   │   │   │   │   └── service-management.md
│   │   │   │   └── monitoring/
│   │   │   │       ├── health-check.py
│   │   │   │       ├── performance-monitor.py
│   │   │   │       └── log-analysis.py
│   │   │   ├── Development/
│   │   │   │   ├── lidar_bridge_v2.py
│   │   │   │   ├── lidar_bridge_v3.py
│   │   │   │   └── experimental/
│   │   │   ├── Testing/
│   │   │   │   ├── lidar-health-test.py
│   │   │   │   ├── obstacle-detection-test.py
│   │   │   │   └── mavlink-integration-test.py
│   │   │   └── Documentation/
│   │   │       ├── Setup-Guide.md
│   │   │       ├── Troubleshooting.md
│   │   │       └── API-Reference.md
│   │   ├── Vision-Tracking/
│   │   │   ├── Production/
│   │   │   │   ├── tennis_ball_enhanced.py ⭐
│   │   │   │   ├── object-tracking-base.py
│   │   │   │   └── vision-mavlink-interface.py
│   │   │   ├── Development/
│   │   │   │   ├── color-calibration-tools/
│   │   │   │   ├── detection-algorithms/
│   │   │   │   └── performance-optimization/
│   │   │   ├── Testing/
│   │   │   │   ├── camera-diagnostic.py
│   │   │   │   ├── vision_diagnostic.py
│   │   │   │   ├── find_camera.py
│   │   │   │   └── tracking-accuracy-tests/
│   │   │   └── Configuration/
│   │   │       ├── camera-settings.json
│   │   │       ├── detection-parameters.json
│   │   │       └── control-gains.json
│   │   ├── GPS-Navigation/
│   │   │   ├── RTK-Integration/
│   │   │   │   ├── rtk-status-monitor.py
│   │   │   │   ├── correction-data-handler.py
│   │   │   │   └── precision-navigation.py
│   │   │   ├── Waypoint-Navigation/
│   │   │   │   ├── mission-planner-interface.py
│   │   │   │   ├── autonomous-navigation.py
│   │   │   │   └── path-planning-algorithms/
│   │   │   └── Testing/
│   │   │       ├── gps-accuracy-tests/
│   │   │       ├── rtk-performance-validation/
│   │   │       └── waypoint-mission-tests/
│   │   ├── Integration-Scripts/
│   │   │   ├── Multi-System-Coordination/
│   │   │   │   ├── sensor-fusion.py
│   │   │   │   ├── priority-arbitration.py
│   │   │   │   └── conflict-resolution.py
│   │   │   ├── System-Health/
│   │   │   │   ├── all-systems-status.py
│   │   │   │   ├── diagnostic-suite.py
│   │   │   │   └── performance-monitoring.py
│   │   │   └── Deployment/
│   │   │       ├── full-system-startup.py
│   │   │       ├── service-orchestration.py
│   │   │       └── configuration-management.py
│   │   └── Utilities/
│   │       ├── Network-Tools/
│   │       ├── Configuration-Management/
│   │       └── Development-Helpers/
│   ├── Testing/
│   │   ├── Field-Tests/
│   │   │   ├── Autonomous-Navigation/
│   │   │   │   ├── obstacle-avoidance-tests/
│   │   │   │   ├── waypoint-navigation-tests/
│   │   │   │   └── extended-mission-tests/
│   │   │   ├── Sensor-Integration/
│   │   │   │   ├── lidar-gps-fusion-tests/
│   │   │   │   ├── vision-lidar-coordination/
│   │   │   │   └── multi-sensor-validation/
│   │   │   └── Performance-Validation/
│   │   │       ├── accuracy-measurements/
│   │   │       ├── reliability-tests/
│   │   │       └── endurance-testing/
│   │   ├── Integration-Tests/
│   │   │   ├── System-Startup/
│   │   │   ├── Multi-System-Compatibility/
│   │   │   └── Failure-Recovery/
│   │   └── Performance-Data/
│   │       ├── LIDAR-Performance/
│   │       ├── GPS-Accuracy/
│   │       ├── Vision-Tracking/
│   │       └── Integration-Metrics/
│   └── Documentation/
│       ├── Setup-Guides/
│       │   ├── Initial-Setup.md
│       │   ├── Hardware-Assembly.md
│       │   ├── Software-Installation.md
│       │   └── System-Calibration.md
│       ├── Operation-Manuals/
│       │   ├── Mission-Planner-Guide.md
│       │   ├── Field-Operation-Procedures.md
│       │   └── Troubleshooting-Guide.md
│       ├── Development-Log/
│       │   ├── LIDAR-Integration-Log.md
│       │   ├── RTK-GPS-Development.md
│       │   ├── Vision-System-Development.md
│       │   └── Integration-Progress.md
│       └── Technical-References/
│           ├── Hardware-Specifications/
│           ├── Software-Architecture/
│           └── Performance-Specifications/

├── 02-Indoor-Rover-Platform/
│   ├── Hardware/
│   │   ├── Arduino-Gatekeeper/
│   │   │   ├── Firmware/
│   │   │   │   ├── rover_arduino_gatekeeper_dual_sensor.ino ⭐
│   │   │   │   ├── stable-versions/
│   │   │   │   └── development-versions/
│   │   │   ├── Configuration/
│   │   │   │   ├── Pin-Assignments/
│   │   │   │   ├── Sensor-Calibration/
│   │   │   │   └── Communication-Protocol/
│   │   │   └── Hardware-Setup/
│   │   │       ├── Wiring-Diagrams/
│   │   │       ├── Component-Lists/
│   │   │       └── Assembly-Instructions/
│   │   ├── Pi-Configuration/
│   │   │   ├── NavigationPi-Setup/
│   │   │   │   ├── OS-Configuration/
│   │   │   │   ├── Python-Environment/
│   │   │   │   └── Network-Configuration/
│   │   │   ├── CompanionPi-Setup/
│   │   │   │   ├── Visualization-Server/
│   │   │   │   ├── Telemetry-Handler/
│   │   │   │   └── Web-Interface/
│   │   │   └── Dual-Pi-Communication/
│   │   │       ├── UDP-Streaming/
│   │   │       ├── Data-Synchronization/
│   │   │       └── Network-Architecture/
│   │   └── Sensor-Setup/
│   │       ├── HC-SR04-Ultrasonics/
│   │       │   ├── Mounting-Solutions/
│   │       │   ├── Calibration-Procedures/
│   │       │   └── Performance-Testing/
│   │       ├── IMU-Integration/
│   │       │   ├── BerryGPS-IMU-Setup/
│   │       │   ├── Calibration-Data/
│   │       │   └── Integration-Scripts/
│   │       └── Optional-Camera/
│   │           ├── Indoor-Vision-Setup/
│   │           └── Lighting-Considerations/
│   ├── Software/
│   │   ├── Navigation-System/
│   │   │   ├── Production/
│   │   │   │   ├── run_enhanced_exploration.py ⭐
│   │   │   │   ├── autonomous_explorer.py
│   │   │   │   └── main.py
│   │   │   ├── Modules/
│   │   │   │   ├── sensors/
│   │   │   │   │   ├── ultrasonic.py
│   │   │   │   │   ├── berry_imu.py
│   │   │   │   │   └── __init__.py
│   │   │   │   ├── actuators/
│   │   │   │   │   ├── arduino_interface.py
│   │   │   │   │   └── __init__.py
│   │   │   │   ├── navigation/
│   │   │   │   │   ├── obstacle_avoidance.py
│   │   │   │   │   ├── path_planning.py
│   │   │   │   │   └── __init__.py
│   │   │   │   ├── config/
│   │   │   │   │   ├── settings.py
│   │   │   │   │   └── __init__.py
│   │   │   │   └── utils/
│   │   │   │       ├── data_logger.py
│   │   │   │       ├── math_helpers.py
│   │   │   │       └── __init__.py
│   │   │   ├── Development/
│   │   │   │   ├── enhanced_nav_with_reverse.py
│   │   │   │   ├── adaptive_imu_navigator.py
│   │   │   │   └── experimental-algorithms/
│   │   │   └── Configuration/
│   │   │       ├── navigation-parameters.json
│   │   │       ├── sensor-calibration.json
│   │   │       └── motor-control-settings.json
│   │   ├── Visualization/
│   │   │   ├── Web-Interface/
│   │   │   │   ├── minimal_viz_8090.py ⭐
│   │   │   │   ├── static/
│   │   │   │   │   ├── css/
│   │   │   │   │   ├── js/
│   │   │   │   │   └── images/
│   │   │   │   └── templates/
│   │   │   ├── Real-Time-Data/
│   │   │   │   ├── udp-streaming.py
│   │   │   │   ├── data-processing.py
│   │   │   │   └── visualization-backend.py
│   │   │   └── Dashboard/
│   │   │       ├── sensor-dashboard.py
│   │   │       ├── performance-metrics.py
│   │   │       └── system-status.py
│   │   ├── Test-Scripts/
│   │   │   ├── Hardware-Tests/
│   │   │   │   ├── test_arduino_connection.py
│   │   │   │   ├── test_sensor_module.py
│   │   │   │   ├── test_motor_commands.py
│   │   │   │   └── test_autonomous_basic.py
│   │   │   ├── Navigation-Tests/
│   │   │   │   ├── obstacle-avoidance-tests/
│   │   │   │   ├── path-planning-validation/
│   │   │   │   └── performance-benchmarks/
│   │   │   └── Integration-Tests/
│   │   │       ├── dual-pi-communication/
│   │   │       ├── sensor-fusion-tests/
│   │   │       └── system-reliability/
│   │   └── Utilities/
│   │       ├── Data-Collection/
│   │       ├── Calibration-Tools/
│   │       └── Development-Helpers/
│   ├── Testing/
│   │   ├── Indoor-Navigation/
│   │   │   ├── Controlled-Environment/
│   │   │   ├── Complex-Obstacle-Courses/
│   │   │   └── Performance-Validation/
│   │   └── Performance-Metrics/
│   │       ├── Navigation-Success-Rates/
│   │       ├── Sensor-Accuracy/
│   │       └── System-Reliability/
│   └── Documentation/
│       ├── Setup-Guide.md
│       ├── Operation-Manual.md
│       ├── Development-History.md
│       └── Troubleshooting.md

├── 03-RTK-Base-Station/
│   ├── Hardware/
│   │   ├── GPS-Configuration/
│   │   │   ├── ZED-F9P-Base-Setup/
│   │   │   │   ├── Device-Configuration/
│   │   │   │   ├── Firmware-Updates/
│   │   │   │   └── Performance-Validation/
│   │   │   ├── Antenna-Systems/
│   │   │   │   ├── Survey-Grade-Antennas/
│   │   │   │   ├── Ground-Plane-Design/
│   │   │   │   └── Cable-Management/
│   │   │   └── Environmental-Protection/
│   │   │       ├── Weather-Enclosures/
│   │   │       ├── Lightning-Protection/
│   │   │       └── Temperature-Control/
│   │   ├── Mounting-Solutions/
│   │   │   ├── Survey-Tripods/
│   │   │   ├── Permanent-Installations/
│   │   │   │   ├── Concrete-Monuments/
│   │   │   │   ├── Building-Mounts/
│   │   │   │   └── Pole-Installations/
│   │   │   └── Portable-Solutions/
│   │   │       ├── Quick-Deploy-Tripods/
│   │   │       ├── Vehicle-Mounts/
│   │   │       └── Backpack-Systems/
│   │   └── Power-Systems/
│   │       ├── AC-Power-Solutions/
│   │       ├── Battery-Backup-Systems/
│   │       ├── Solar-Power-Integration/
│   │       └── Power-Consumption-Analysis/
│   ├── Software/
│   │   ├── Mission-Planner-Setup/
│   │   │   ├── Base-Station-Configuration/
│   │   │   ├── Survey-In-Procedures/
│   │   │   ├── Correction-Distribution/
│   │   │   └── Status-Monitoring/
│   │   ├── Correction-Distribution/
│   │   │   ├── RTCM3-Message-Handler/
│   │   │   ├── Network-Distribution/
│   │   │   ├── Radio-Link-Integration/
│   │   │   └── Multi-Rover-Support/
│   │   ├── Monitoring-Scripts/
│   │   │   ├── Base-Station-Health/
│   │   │   ├── Correction-Quality-Monitor/
│   │   │   ├── Coverage-Area-Analysis/
│   │   │   └── Performance-Logging/
│   │   └── Automation/
│   │       ├── Auto-Start-Scripts/
│   │       ├── System-Recovery/
│   │       └── Remote-Management/
│   ├── CAD-Models/                    # Future CAD files
│   │   ├── Mounting-Hardware/
│   │   ├── Enclosure-Designs/
│   │   ├── Antenna-Mounts/
│   │   └── Assembly-Drawings/
│   ├── Installation-Data/             # Site survey and setup records
│   │   ├── Site-Surveys/
│   │   │   ├── Location-Analysis/
│   │   │   ├── Multipath-Assessment/
│   │   │   └── Coverage-Planning/
│   │   ├── Installation-Records/
│   │   │   ├── Setup-Procedures/
│   │   │   ├── Calibration-Data/
│   │   │   └── Commissioning-Tests/
│   │   └── Maintenance-Logs/
│   │       ├── Routine-Maintenance/
│   │       ├── Performance-Tracking/
│   │       └── Issue-Resolution/
│   └── Documentation/
│       ├── Setup-Guide.md
│       ├── Operation-Manual.md
│       ├── Maintenance-Procedures.md
│       └── Troubleshooting.md

├── 04-Mobile-RTK-Station/             # Future development
│   ├── Hardware/
│   │   ├── Device-Design/
│   │   │   ├── Form-Factor-Studies/
│   │   │   ├── Component-Selection/
│   │   │   └── Integration-Planning/
│   │   ├── Wearable-Solutions/
│   │   │   ├── Belt-Mount-Design/
│   │   │   ├── Backpack-Integration/
│   │   │   └── Hands-Free-Operation/
│   │   └── Power-Management/
│   │       ├── Battery-Life-Analysis/
│   │       ├── Charging-Solutions/
│   │       └── Power-Optimization/
│   ├── Software/
│   │   ├── Follow-Me-Applications/
│   │   │   ├── Person-Tracking/
│   │   │   ├── Dynamic-Waypoints/
│   │   │   └── Safety-Systems/
│   │   ├── Mobile-Coordination/
│   │   │   ├── Rover-Communication/
│   │   │   ├── Multi-Device-Network/
│   │   │   └── Mesh-Networking/
│   │   └── User-Interface/
│   │       ├── Mobile-App-Design/
│   │       ├── Voice-Control/
│   │       └── Emergency-Controls/
│   ├── Mobile-Apps/
│   │   ├── Android-Development/
│   │   ├── iOS-Development/
│   │   └── Cross-Platform-Solutions/
│   ├── Prototypes/
│   │   ├── Proof-of-Concept/
│   │   ├── Alpha-Testing/
│   │   └── Beta-Development/
│   └── Documentation/
│       ├── Concept-Design.md
│       ├── Development-Roadmap.md
│       └── User-Requirements.md

├── 05-Base-Station-Setup/
│   ├── Network-Configuration/
│   │   ├── SSH-Keys/
│   │   │   ├── Key-Generation-Scripts/
│   │   │   ├── Deployment-Procedures/
│   │   │   ├── rover-access-keys/
│   │   │   │   ├── outdoor-rover-keys/
│   │   │   │   ├── indoor-rover-keys/
│   │   │   │   └── base-station-keys/
│   │   │   └── Key-Management/
│   │   │       ├── Rotation-Procedures/
│   │   │       ├── Backup-Management/
│   │   │       └── Access-Control/
│   │   ├── Router-Configs/
│   │   │   ├── Travel-Router-Setup/
│   │   │   │   ├── GL-iNet-AX1800-Config/
│   │   │   │   ├── Firmware-Updates/
│   │   │   │   └── Performance-Optimization/
│   │   │   ├── Network-Topology/
│   │   │   │   ├── IP-Address-Plans/
│   │   │   │   ├── VLAN-Configuration/
│   │   │   │   └── Routing-Tables/
│   │   │   └── Security-Configuration/
│   │   │       ├── Firewall-Rules/
│   │   │       ├── Access-Control/
│   │   │       └── VPN-Setup/
│   │   ├── Internet-Bridge/
│   │   │   ├── Laptop-WiFi-Bridge/
│   │   │   │   ├── IP-Forwarding-Setup/
│   │   │   │   ├── NAT-Configuration/
│   │   │   │   └── iptables-Rules/
│   │   │   ├── Pi-Routing-Setup/
│   │   │   │   ├── Route-Configuration/
│   │   │   │   ├── DNS-Setup/
│   │   │   │   └── Network-Testing/
│   │   │   └── Automation-Scripts/
│   │   │       ├── Auto-Bridge-Setup/
│   │   │       ├── Connection-Monitoring/
│   │   │       └── Failover-Procedures/
│   │   └── IP-Management/
│   │       ├── Address-Allocation/
│   │       │   ├── Static-IP-Assignments/
│   │       │   ├── DHCP-Reservations/
│   │       │   └── Network-Documentation/
│   │       ├── Network-Monitoring/
│   │       │   ├── Connectivity-Tests/
│   │       │   ├── Performance-Monitoring/
│   │       │   └── Troubleshooting-Tools/
│   │       └── Network-Security/
│   │           ├── Access-Control-Lists/
│   │           ├── Traffic-Analysis/
│   │           └── Intrusion-Detection/
│   ├── Platform-Configurations/
│   │   ├── Mission-Planner-Params/
│   │   │   ├── Outdoor-Rover-Params/
│   │   │   │   ├── LIDAR-Parameters/
│   │   │   │   ├── RTK-GPS-Parameters/
│   │   │   │   ├── Obstacle-Avoidance-Params/
│   │   │   │   └── Complete-Parameter-Sets/
│   │   │   ├── Indoor-Rover-Params/
│   │   │   │   ├── Basic-Navigation-Params/
│   │   │   │   ├── Sensor-Configuration/
│   │   │   │   └── Motor-Control-Params/
│   │   │   └── Base-Station-Params/
│   │   │       ├── RTK-Base-Configuration/
│   │   │       ├── Correction-Distribution/
│   │   │       └── Monitoring-Setup/
│   │   ├── Arduino-Firmware/
│   │   │   ├── Production-Firmware/
│   │   │   │   ├── Indoor-Rover-Gatekeeper/
│   │   │   │   └── Version-History/
│   │   │   ├── Development-Firmware/
│   │   │   │   ├── Experimental-Features/
│   │   │   │   └── Testing-Versions/
│   │   │   └── Firmware-Management/
│   │   │       ├── Upload-Procedures/
│   │   │       ├── Version-Control/
│   │   │       └── Backup-Management/
│   │   ├── Pi-Environments/
│   │   │   ├── Python-Environment-Setup/
│   │   │   │   ├── requirements.txt
│   │   │   │   ├── virtual-environment-setup/
│   │   │   │   └── dependency-management/
│   │   │   ├── System-Configuration/
│   │   │   │   ├── OS-Setup-Scripts/
│   │   │   │   ├── Package-Installation/
│   │   │   │   └── System-Optimization/
│   │   │   └── Development-Tools/
│   │   │       ├── IDE-Configuration/
│   │   │       ├── Debugging-Tools/
│   │   │       └── Code-Quality-Tools/
│   │   └── Service-Configs/
│   │       ├── systemd-Services/
│   │       │   ├── LIDAR-Bridge-Service/
│   │       │   ├── Navigation-Services/
│   │       │   ├── Visualization-Services/
│   │       │   └── Monitoring-Services/
│   │       ├── Service-Management/
│   │       │   ├── Installation-Scripts/
│   │       │   ├── Update-Procedures/
│   │       │   └── Troubleshooting/
│   │       └── Auto-Start-Configuration/
│   │           ├── Boot-Sequence/
│   │           ├── Dependency-Management/
│   │           └── Failure-Recovery/
│   ├── Operational-Data/
│   │   ├── Test-Logs/
│   │   │   ├── Field-Test-Results/
│   │   │   │   ├── Outdoor-Rover-Tests/
│   │   │   │   ├── Indoor-Rover-Tests/
│   │   │   │   └── Integration-Tests/
│   │   │   ├── Performance-Data/
│   │   │   │   ├── System-Benchmarks/
│   │   │   │   ├── Sensor-Accuracy/
│   │   │   │   └── Reliability-Metrics/
│   │   │   └── Issue-Reports/
│   │   │       ├── Bug-Reports/
│   │   │       ├── Resolution-Records/
│   │   │       └── Lessons-Learned/
│   │   ├── Hardware-Inventory/
│   │   │   ├── Component-Lists/
│   │   │   │   ├── Outdoor-Rover-BOM/
│   │   │   │   ├── Indoor-Rover-BOM/
│   │   │   │   ├── Base-Station-Components/
│   │   │   │   └── Spare-Parts-Inventory/
│   │   │   ├── Serial-Numbers/
│   │   │   │   ├── GPS-Devices/
│   │   │   │   ├── LIDAR-Units/
│   │   │   │   ├── Cameras/
│   │   │   │   └── Computing-Devices/
│   │   │   └── Warranty-Information/
│   │   │       ├── Purchase-Records/
│   │   │       ├── Warranty-Status/
│   │   │       └── Support-Contacts/
│   │   └── Software-Versions/
│   │       ├── Current-Deployments/
│   │       │   ├── Production-Versions/
│   │       │   ├── Development-Branches/
│   │       │   └── Configuration-Snapshots/
│   │       ├── Version-History/
│   │       │   ├── Release-Notes/
│   │       │   ├── Deployment-Records/
│   │       │   └── Rollback-Procedures/
│   │       └── Update-Management/
│   │           ├── Update-Procedures/
│   │           ├── Testing-Protocols/
│   │           └── Deployment-Automation/
│   └── Documentation/
│       ├── Setup-Procedures/
│       │   ├── Network-Setup-Guide.md
│       │   ├── SSH-Configuration.md
│       │   ├── Internet-Bridge-Setup.md
│       │   └── Platform-Configuration.md
│       ├── Troubleshooting/
│       │   ├── Network-Issues.md
│       │   ├── SSH-Problems.md
│       │   ├── Service-Failures.md
│       │   └── Hardware-Problems.md
│       ├── Configuration-Guides/
│       │   ├── Parameter-Management.md
│       │   ├── Service-Configuration.md
│       │   ├── Environment-Setup.md
│       │   └── Security-Configuration.md
│       └── Operational-Procedures/
│           ├── Daily-Operations.md
│           ├── Maintenance-Schedule.md
│           ├── Backup-Procedures.md
│           └── Emergency-Procedures.md

└── Archive/                           # Historical/deprecated files
    ├── Previous-Organizations/
    │   ├── Path-Based-Organization/
    │   └── Original-Structure/
    ├── Deprecated-Code/
    │   ├── Old-Navigation-Scripts/
    │   ├── Legacy-Arduino-Firmware/
    │   └── Obsolete-Configuration/
    ├── Historical-Documentation/
    │   ├── Early-Development-Logs/
    │   ├── Superseded-Guides/
    │   └── Old-Test-Results/
    └── Migration-Records/
        ├── Organization-Changes/
        ├── File-Movement-Logs/
        └── Archive-Procedures/
```

---

## 📋 **IMPLEMENTATION CHECKLIST**

### **Phase 1: Core Structure Creation**
- [ ] Create main platform directories (01-05)
- [ ] Set up Hardware/Software/Testing/Documentation subdirectories
- [ ] Create README.md files for each major section
- [ ] Establish .gitignore patterns for each platform

### **Phase 2: File Migration**
- [ ] Map existing files to new structure
- [ ] Move production-ready code to appropriate Platform/Production directories
- [ ] Organize development and experimental code
- [ ] Archive deprecated or outdated files

### **Phase 3: Documentation Creation**
- [ ] Create platform-specific setup guides
- [ ] Document hardware configurations and procedures
- [ ] Establish troubleshooting guides for each platform
- [ ] Create operational procedures documentation

### **Phase 4: Configuration Management**
- [ ] Organize parameter files by platform
- [ ] Set up configuration version control
- [ ] Create deployment and backup procedures
- [ ] Establish change management processes

---

## 🎯 **FOLDER USAGE GUIDELINES**

### **Production vs Development**
- **Production/**: Only tested, stable, field-deployable code
- **Development/**: Active development, experimental features
- **Testing/**: Test scripts, validation procedures, performance data
- **Archive/**: Historical, deprecated, or superseded files

### **Documentation Standards**
- Each major directory should have a README.md explaining its contents
- Setup guides should be comprehensive and step-by-step
- Troubleshooting guides should include common issues and solutions
- All configuration changes should be documented

### **Version Control**
- Use semantic versioning for production software
- Tag stable releases for easy rollback
- Maintain changelog files for major components
- Document breaking changes and migration procedures

---

**🎯 FOLDER STRUCTURE STATUS: COMPREHENSIVE PLATFORM-BASED ORGANIZATION**  
**Ready for Implementation | Supports All 5 Platforms | Scales for Future Growth**

---

*Structure Guide Created: September 13, 2025*  
*Implementation: Progressive migration from current structure*  
*Maintenance: Update as platforms evolve and grow*