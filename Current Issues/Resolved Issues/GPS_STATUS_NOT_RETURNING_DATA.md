# Issue: GPS Data Not Returning in /api/status Response

**Date Reported:** November 21, 2025
**Reported By:** Mobile RTK Control Module Dashboard
**Severity:** High - Blocks rover GPS display on dashboard

---

## Problem Description

The dashboard is connected to the Jetson rover at `100.91.191.47:5001` but is not receiving GPS data. The `/api/status` endpoint returns `"gps": false` instead of a GPS data object.

---

## Actual Response from Jetson

```bash
curl http://100.91.191.47:5001/api/status
```

```json
{
    "battery": false,
    "camera": false,
    "gps": false,
    "lidar": false,
    "manual_mode": false,
    "state": false,
    "success": true,
    "timestamp": "2025-11-21T16:55:14.077503"
}
```

---

## Expected Response Format

The dashboard expects GPS data in this format:

```json
{
    "success": true,
    "armed": false,
    "gps": {
        "latitude": 37.7749,
        "longitude": -122.4194,
        "altitude": 10.5,
        "satellites": 12,
        "fix_type": 4,
        "hdop": 0.8
    },
    "position": {
        "x": 0.0,
        "y": 0.0,
        "heading": 0.0
    },
    "velocity": {
        "linear": 0.0,
        "angular": 0.0
    }
}
```

### GPS Object Fields Required

| Field | Type | Description |
|-------|------|-------------|
| `latitude` | float | Decimal degrees |
| `longitude` | float | Decimal degrees |
| `altitude` | float | Meters above sea level |
| `satellites` | int | Number of satellites in view |
| `fix_type` | int | 0=No GPS, 1=No Fix, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fix |
| `hdop` | float | Horizontal dilution of precision |

---

## Dashboard Code Reference

The dashboard parses GPS data in `09_dashboard_enhanced.py` at the `update_rover_position()` method (around line 1735):

```python
def update_rover_position(self):
    """Poll rover GPS position from Jetson server"""
    try:
        if self.robot and self.robot_connected:
            status = self.robot.update_status()
            if status and 'gps' in status:
                gps_data = status['gps']
                self.rover_lat = gps_data.get('latitude')
                self.rover_lon = gps_data.get('longitude')
                self.rover_alt = gps_data.get('altitude', 0.0)
                self.rover_satellites = gps_data.get('satellites', 0)
                self.rover_fix_type = gps_data.get('fix_type', 0)
                self.rover_hdop = gps_data.get('hdop', 0.0)
```

When `gps` is `false` instead of a dict, the `.get()` calls fail silently and no GPS data is displayed.

---

## Possible Causes

1. **MAVLink connection not established** - The server may not be connected to the Cube Orange flight controller

2. **No GPS telemetry from Cube Orange** - The flight controller may not be sending GPS messages (GPS_RAW_INT, GLOBAL_POSITION_INT)

3. **GPS hardware not connected** - Physical GPS module may not be connected to the Cube Orange

4. **No satellite fix** - GPS may be indoors or have no clear sky view

5. **Code issue in status endpoint** - The `/api/status` handler may not be populating GPS data correctly

---

## Files to Check

### Primary file (unified bridge):
- `~/rover/experimental/manual_control/ros2_unified_bridge.py` (if this is the active server)

### Or original files:
- `ros2_ws/src/jetson_rover_bridge/jetson_rover_bridge/http_bridge.py`
- `jetson_rover_server.py` (MAVLink direct connection)

### Look for:
1. The `/api/status` endpoint handler
2. GPS data parsing from MAVLink messages
3. Any GPS subscription or telemetry handling

---

## Suggested Fix

In the status endpoint handler, ensure GPS data is populated:

```python
@app.route('/api/status', methods=['GET'])
def get_status():
    gps_data = None

    # Get GPS from MAVLink or ROS2 topic
    if mavlink_connection and last_gps_message:
        gps_data = {
            'latitude': last_gps_message.lat / 1e7,  # MAVLink sends as int * 1e7
            'longitude': last_gps_message.lon / 1e7,
            'altitude': last_gps_message.alt / 1000.0,  # mm to meters
            'satellites': last_gps_message.satellites_visible,
            'fix_type': last_gps_message.fix_type,
            'hdop': last_gps_message.eph / 100.0  # cm to meters
        }

    return jsonify({
        'success': True,
        'gps': gps_data if gps_data else False,
        # ... other fields
    })
```

---

## Testing

After fix, verify with:

```bash
# Check status returns GPS object
curl http://100.91.191.47:5001/api/status | python3 -m json.tool

# Should see:
# "gps": {
#     "latitude": ...,
#     "longitude": ...,
#     ...
# }
```

---

## Related

- Health endpoint works correctly: `/api/health` returns `{"success": true, "status": "online"}`
- Velocity endpoint works: `/api/velocity` accepts commands
- Only GPS data population is affected

---

**Status:** RESOLVED
**Assigned To:** Jetson Cube Orange development instance
**Resolved By:** Service restart
**Resolution Date:** November 21, 2025

---

## Resolution

### Root Cause
The unified bridge service was running with stale ROS2 subscriptions. The service had been started before MAVROS2 fully initialized its topics, causing the GPS subscription to fail silently.

### Investigation Findings
1. MAVROS2 was publishing GPS data correctly to `/mavros/mavros/gps1/raw`
2. The unified bridge code had the correct topic path
3. The service needed `ROS_DOMAIN_ID=42` to see MAVROS topics (was configured correctly)
4. Simply restarting the service allowed it to properly subscribe to the GPS topic

### Fix Applied
```bash
sudo systemctl restart jetson-unified-bridge
```

### Verified Response After Fix
```json
{
    "gps": {
        "altitude": 111.63,
        "fix_type": 3,
        "latitude": 45.4303643,
        "longitude": -122.8409513,
        "satellites": 16,
        "hdop": 0.068,
        ...
    },
    "state": {
        "armed": false,
        "connected": true,
        "mode": "MANUAL"
    },
    "success": true
}
```

### Recommendation
Consider adding a startup delay or retry logic to the unified bridge service to ensure MAVROS2 topics are available before subscribing. Alternatively, implement dynamic topic discovery.
