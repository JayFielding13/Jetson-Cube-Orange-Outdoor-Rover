# Jetson Cube Orange Outdoor Rover - Manual Joystick Control Integration

This document describes how to add manual joystick control support to the Jetson Cube Orange Outdoor Rover, enabling the Mobile RTK Control Module dashboard to send real-time throttle/steering commands from the ESP32-C3 joystick over HTTP.

## Overview

The Mobile RTK Control Module dashboard sends HTTP POST requests with throttle and steering values. The Jetson's `http_bridge.py` receives these and publishes velocity commands to the ROS2 `/cmd_vel` topic, which controls the rover.

### Architecture Summary

```
Dashboard (Pi)                    Jetson Orin Nano                   Rover
┌─────────────────┐              ┌─────────────────┐              ┌──────────┐
│ ESP32 Joystick  │              │   http_bridge   │              │  Gazebo  │
│       ↓         │   HTTP       │   (Flask:5001)  │   ROS2       │  or Real │
│ joystick_handler│ ──────────▶  │       ↓         │ ─────────▶   │  Motors  │
│       ↓         │  POST        │   /cmd_vel      │  Twist       │          │
│ robot_controller│  /api/velocity│  (geometry_msgs)│              │          │
└─────────────────┘              └─────────────────┘              └──────────┘
```

---

## Existing Project Structure

### Key Files

| Path | Purpose |
|------|---------|
| `ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/http_bridge.py` | Flask HTTP server + ROS2 node |
| `ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/gps_bridge.py` | GPS coordinate conversion |
| `ros2_ws/src/jetson_rover_bridge/launch/ground_control_bridge.launch.py` | Launch HTTP + GPS bridges |
| `xbox_rover.config.yaml` | Xbox controller joystick mapping |
| `run_xbox_controller.sh` | Script to launch Xbox teleop |
| `jetson_rover_server.py` | Direct MAVLink server (hardware, port 5000) |

### Existing HTTP API Endpoints (Port 5001)

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/health` | GET | Health check |
| `/api/status` | GET | Robot status (GPS, odometry, armed state) |
| `/api/arm` | POST | ARM motors |
| `/api/disarm` | POST | DISARM motors |
| `/api/target` | POST | Send GPS waypoint |
| `/api/stop` | POST | Emergency stop |
| `/api/pause` | POST | Pause (stay armed, zero velocity) |
| `/api/cancel` | POST | Cancel mission |

### Missing Endpoint

**`/api/velocity`** - Direct velocity control for joystick integration (needs to be added)

---

## Implementation: Add Velocity Endpoint to http_bridge.py

### File to Modify

`/home/jay/Git Sandbox/Jetson Cube Orange Outdoor Rover/ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/http_bridge.py`

### 1. Add the Velocity Endpoint

Add this endpoint after the existing `/api/pause` endpoint (around line 256):

```python
@app.route('/api/velocity', methods=['POST'])
def send_velocity():
    """
    Direct velocity control for joystick/manual mode

    Request JSON:
    {
        "linear": 0.5,    # -1.0 to +1.0 (forward/backward)
        "angular": -0.3   # -1.0 to +1.0 (turn left/right)
    }
    """
    data = request.json

    if not data:
        return jsonify({
            'success': False,
            'message': 'No data provided'
        }), 400

    # Get values with defaults
    linear = float(data.get('linear', 0.0))
    angular = float(data.get('angular', 0.0))

    # Clamp to valid range
    linear = max(-1.0, min(1.0, linear))
    angular = max(-1.0, min(1.0, angular))

    # Scale to actual velocity limits
    max_linear = 0.7   # m/s (matches xbox_rover.config.yaml)
    max_angular = 0.8  # rad/s

    linear_vel = linear * max_linear
    angular_vel = angular * max_angular

    if ros_node:
        success = ros_node.send_velocity_command(linear_vel, angular_vel)
        return jsonify({
            'success': success,
            'linear': linear,
            'angular': angular,
            'linear_vel': linear_vel,
            'angular_vel': angular_vel,
            'message': 'Velocity command sent' if success else 'Failed - check armed state'
        })

    return jsonify({
        'success': False,
        'message': 'ROS node not initialized'
    }), 503
```

### 2. Optional: Add Manual Mode Tracking

To track manual control state and add timeout safety, add to `HTTPBridgeNode.__init__`:

```python
# Manual control state
self.manual_mode = False
self.last_velocity_time = time.time()
```

Add a timer callback for safety timeout:

```python
def __init__(self):
    super().__init__('http_bridge')
    # ... existing code ...

    # Manual control safety timeout timer (check every 100ms)
    self.safety_timer = self.create_timer(0.1, self.check_velocity_timeout)

def check_velocity_timeout(self):
    """Stop if no velocity command received for 500ms"""
    if self.manual_mode and self.armed:
        if time.time() - self.last_velocity_time > 0.5:
            self.get_logger().warn('Velocity timeout - stopping')
            self.send_velocity_command(0.0, 0.0)
            self.manual_mode = False
```

Update `send_velocity_command()` to track the timestamp:

```python
def send_velocity_command(self, linear_x: float, angular_z: float) -> bool:
    """Send velocity command to rover"""
    if not self.armed:
        self.get_logger().debug('Cannot send velocity: not armed')
        return False

    self.last_velocity_time = time.time()
    self.manual_mode = True

    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    self.cmd_vel_pub.publish(msg)
    return True
```

---

## Alternative: Use Existing Xbox Controller System

The project already has a working Xbox controller system that publishes directly to `/cmd_vel` via ROS2.

### Configuration File: `xbox_rover.config.yaml`

```yaml
teleop_twist_joy_node:
  ros__parameters:
    axis_linear:
      x: 1                    # Left stick vertical
    scale_linear:
      x: 0.7                  # Max 0.7 m/s
    scale_linear_turbo:
      x: 1.5                  # Turbo 1.5 m/s

    axis_angular:
      yaw: 0                  # Left stick horizontal
    scale_angular:
      yaw: 0.8                # Max 0.8 rad/s
    scale_angular_turbo:
      yaw: 1.5                # Turbo 1.5 rad/s

    enable_button: -1         # No deadman switch
    enable_turbo_button: 5    # Right bumper for turbo
```

### Launch Script: `run_xbox_controller.sh`

```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:=xbox_rover.config.yaml
```

This can serve as reference for the velocity limits used in the HTTP endpoint.

---

## API Endpoint Specification

### POST `/api/velocity`

**Request Body (JSON):**
```json
{
    "linear": 0.75,     // -1.0 (reverse) to +1.0 (forward)
    "angular": -0.25    // -1.0 (right) to +1.0 (left)
}
```

**Response (JSON):**
```json
{
    "success": true,
    "linear": 0.75,
    "angular": -0.25,
    "linear_vel": 0.525,
    "angular_vel": -0.2,
    "message": "Velocity command sent"
}
```

**Requirements:**
- Motors must be ARMED before velocity commands are accepted
- Commands should be sent at 20Hz from the dashboard
- If no command received for 500ms, stop motors (safety timeout)
- Linear values map to `/cmd_vel.linear.x`
- Angular values map to `/cmd_vel.angular.z`

---

## Dashboard Integration

### robot_controller.py Updates

The dashboard's `RobotController` class should call the velocity endpoint:

```python
def send_velocity_command(self, linear: float, angular: float) -> bool:
    """
    Send velocity command to robot

    Args:
        linear: -1.0 (reverse) to +1.0 (forward)
        angular: -1.0 (right) to +1.0 (left)

    Returns:
        True if command was accepted
    """
    if not self.connected or not self.armed:
        return False

    try:
        response = requests.post(
            f"{self.base_url}/api/velocity",
            json={'linear': linear, 'angular': angular},
            timeout=0.1  # Short timeout for responsive control
        )
        return response.status_code == 200 and response.json().get('success', False)
    except requests.RequestException:
        return False
```

### Joystick Handler Integration

The dashboard already reads joystick values at 20 FPS. Connect them to the velocity endpoint:

```python
# In dashboard update loop
if self.manual_mode_enabled and self.joystick_connected:
    throttle, steering = self.joystick_handler.get_position()
    self.robot_controller.send_velocity_command(throttle, steering)
```

---

## Testing

### 1. Test with curl

```bash
# ARM the robot first
curl -X POST http://100.91.191.47:5001/api/arm

# Send velocity command
curl -X POST http://100.91.191.47:5001/api/velocity \
  -H "Content-Type: application/json" \
  -d '{"linear": 0.3, "angular": 0.0}'

# Stop
curl -X POST http://100.91.191.47:5001/api/velocity \
  -H "Content-Type: application/json" \
  -d '{"linear": 0.0, "angular": 0.0}'

# DISARM
curl -X POST http://100.91.191.47:5001/api/disarm
```

### 2. Test from Dashboard

1. Launch the dashboard on the Pi
2. Connect to robot (Jetson at 100.91.191.47:5001)
3. ARM motors
4. Enable Manual Mode
5. Move joystick - rover should respond

### 3. Safety Timeout Test

1. ARM and enable manual mode
2. Send velocity command with curl
3. Wait >500ms without sending another command
4. Verify rover stops automatically

---

## ROS2 Topics Reference

### Published Topics (from http_bridge)

| Topic | Type | Purpose |
|-------|------|---------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/rover/armed` | `std_msgs/Bool` | ARM/DISARM state |
| `/waypoint/goto` | `geometry_msgs/PoseStamped` | Waypoint targets |

### Subscribed Topics (by http_bridge)

| Topic | Type | Purpose |
|-------|------|---------|
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS position |
| `/odom` | `nav_msgs/Odometry` | Odometry feedback |

---

## Network Configuration

| Device | Tailscale IP | Port | Service |
|--------|--------------|------|---------|
| Jetson Orin Nano | 100.91.191.47 | 5001 | http_bridge (ROS2) |
| Jetson Orin Nano | 100.91.191.47 | 5000 | jetson_rover_server (MAVLink) |
| Raspberry Pi (beaconpi) | 100.73.233.124 | - | Dashboard |
| RTK Base Station | 100.66.67.11 | - | NTRIP caster |

---

## Comparison: HTTP Bridge vs MAVLink Server

| Feature | http_bridge.py (Port 5001) | jetson_rover_server.py (Port 5000) |
|---------|---------------------------|-----------------------------------|
| Protocol | ROS2 → Gazebo | MAVLink → Cube Orange |
| Use Case | Simulation | Real Hardware |
| Velocity | `/cmd_vel` topic | RC Override |
| ARM/DISARM | ROS2 topic | MAVLink command |
| Status | Odometry + GPS | MAVLink telemetry |

**For simulation/development**: Use http_bridge on port 5001
**For real hardware**: Use jetson_rover_server on port 5000 (requires separate RC Override implementation)

---

## Files Modified

| File | Change |
|------|--------|
| `http_bridge.py` | Add `/api/velocity` endpoint |

## Files Reference (No Changes Needed)

| File | Purpose |
|------|---------|
| `ground_control_bridge.launch.py` | Launch file (already starts http_bridge) |
| `xbox_rover.config.yaml` | Reference for velocity limits |
| `robot_controller.py` | Dashboard HTTP client |
| `joystick_handler_serial.py` | ESP32 joystick reader |

---

## Troubleshooting

### Command not accepted
1. Check motors are ARMED (`/api/status` shows `armed: true`)
2. Verify ROS2 node is running (`ros2 node list` should show `/http_bridge`)
3. Check http_bridge logs for errors

### Rover moves erratically
1. Reduce `max_linear` and `max_angular` values
2. Ensure joystick has 15% deadzone configured
3. Check network latency (<50ms ideal)

### Timeout/connection errors
1. Verify Tailscale VPN is connected
2. Check firewall allows port 5001
3. Test with `curl` to isolate dashboard issues

---

**Project**: Jetson Cube Orange Outdoor Rover
**Last Updated**: November 21, 2025
**Status**: Ready for implementation
