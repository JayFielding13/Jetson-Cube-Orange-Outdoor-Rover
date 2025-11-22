# Issue: RTK Corrections Status Not Reported in /api/status

**Date Reported:** November 21, 2025
**Reported By:** Mobile RTK Control Module Dashboard development instance
**Severity:** Medium - Feature request for improved RTK monitoring
**Affects:** Dashboard RTK status display for rover

---

## Problem Description

The Mobile RTK Control Module Dashboard has a "RTK CORRECTIONS" section under "ROVER GPS" on the GPS Status tab. This section is designed to show whether the Jetson rover is receiving RTK correction data from the base station.

Previously, the `/api/status` endpoint did not include RTK correction status, so the dashboard could not display this information and showed "Status: NOT AVAILABLE".

---

## Resolution

### Implementation Approach

Used Option 1 from the suggestions: Subscribe to the existing RTCM topic `/mavros/gps_rtk/send_rtcm` that the `rtcm_mqtt_forwarder` publishes to. This tracks:
- Connection status (based on 5-second timeout)
- Total correction messages received
- Total bytes received
- Time since last correction

### Changes Made

**File Modified:** `~/ros2_unified_bridge.py` on Jetson

1. **Added RTCM message import:**
   ```python
   from mavros_msgs.msg import State, Waypoint, GPSRAW, OverrideRCIn, RTCM
   ```

2. **Added RTK tracking variables:**
   ```python
   self.rtk_corrections_count = 0
   self.rtk_bytes_received = 0
   self.last_rtk_time = None
   self.rtk_connected = False
   self.rtk_timeout = 5.0  # Consider disconnected after 5s
   ```

3. **Added RTCM subscription:**
   ```python
   rtcm_qos = QoSProfile(
       reliability=ReliabilityPolicy.RELIABLE,
       durability=DurabilityPolicy.VOLATILE,
       history=HistoryPolicy.KEEP_LAST,
       depth=10
   )

   self.rtcm_subscriber = self.create_subscription(
       RTCM,
       '/mavros/gps_rtk/send_rtcm',
       self.rtcm_callback,
       rtcm_qos
   )
   ```

4. **Added RTK callback and status timer:**
   ```python
   def rtcm_callback(self, msg):
       with self.rtk_lock:
           self.rtk_corrections_count += 1
           self.rtk_bytes_received += len(msg.data)
           self.last_rtk_time = time.time()
           self.rtk_connected = True

   def check_rtk_status(self):
       # Check if RTK corrections are still being received
       # Marks as disconnected after 5 seconds of no data
   ```

5. **Updated `/api/status` to include RTK data:**
   ```python
   status['rtk'] = bridge_node.get_rtk_status()
   ```

6. **Added dedicated `/api/rtk` endpoint**

---

## Verified API Response

```bash
curl http://100.91.191.47:5001/api/status
```

Now includes:
```json
{
    "rtk": {
        "connected": true,
        "source": "MQTT",
        "corrections_received": 48,
        "bytes_received": 44982,
        "last_correction_age_ms": 319
    },
    ...
}
```

Dedicated endpoint:
```bash
curl http://100.91.191.47:5001/api/rtk
```

Returns:
```json
{
    "bytes_received": 44982,
    "connected": true,
    "corrections_received": 48,
    "last_correction_age_ms": 319,
    "source": "MQTT",
    "success": true
}
```

---

## Dashboard Compatibility

The implementation matches what the dashboard expects:
- `rtk.connected` (bool) - Dashboard checks this for status display
- `rtk.corrections_received` (int) - Dashboard shows "Forwarded: X"

Additional fields provided:
- `bytes_received` - Total RTCM bytes for monitoring
- `last_correction_age_ms` - For staleness detection
- `source` - Shows "MQTT" or "NONE"

---

## Network Reference

| Device | Tailscale IP | Purpose |
|--------|--------------|---------|
| Jetson Orin Nano | 100.91.191.47:5001 | Unified Bridge API |
| RTK Base Station | 100.66.67.11 | MQTT RTCM Publisher |
| BeaconPi (Dashboard) | 100.73.233.124 | Mobile RTK Control Module |

---

## Related Files

**Jetson (modified):**
- `~/ros2_unified_bridge.py` - Production unified bridge with RTK tracking
- `~/rover/experimental/manual_control/ros2_unified_bridge.py` - Experimental version (same code)
- `~/ros2_unified_bridge.py.backup` - Backup of previous version

**Dashboard (already implemented):**
- `09_dashboard_enhanced.py` - Lines 1749-1752 (data extraction), 1830-1845 (display)

---

**Status:** RESOLVED
**Resolution Date:** November 21, 2025
**Resolved By:** Added RTCM topic subscription to ros2_unified_bridge.py
