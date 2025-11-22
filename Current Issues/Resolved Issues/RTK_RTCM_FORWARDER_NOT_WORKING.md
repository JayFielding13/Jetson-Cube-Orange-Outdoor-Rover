# Issue: RTK RTCM Forwarder Not Sending Corrections to MAVROS

**Date Reported:** November 21, 2025
**Reported By:** Jay (during Mobile RTK Module integration testing)
**Severity:** High - RTK corrections not reaching GPS

---

## Problem Description

After moving all devices to Tailscale VPN, the RTCM forwarder service was not successfully forwarding RTK corrections from the base station to MAVROS. The GPS remained at Fix Type 3 (3D Fix) instead of converging to RTK Float/Fixed.

---

## Root Causes Found

### Issue 1: Wrong MQTT Broker IP Address

The RTCM forwarder was configured with the old home network IP:

```python
# Old (wrong)
MQTT_BROKER = '192.168.254.165'  # RTKPi base station

# New (correct)
MQTT_BROKER = '100.66.67.11'  # RTKPi Tailscale IP
```

### Issue 2: QoS Mismatch Between Publisher and Subscriber

The forwarder was publishing with `BEST_EFFORT` QoS, but MAVROS's gps_rtk subscriber expects `RELIABLE` QoS:

```
Publisher (rtcm_mqtt_forwarder): BEST_EFFORT
Subscriber (MAVROS gps_rtk):     RELIABLE
```

In ROS2, a BEST_EFFORT publisher cannot send messages to a RELIABLE subscriber.

---

## Files Modified

**File:** `/home/jay/rtcm_mqtt_forwarder.py` on Jetson

### Fix 1: Update MQTT Broker IP (Line 17)

```python
# Before
MQTT_BROKER = '192.168.254.165'  # RTKPi base station

# After
MQTT_BROKER = '100.66.67.11'  # RTKPi base station (Tailscale)
```

### Fix 2: Update QoS Profile (Line 30-31)

```python
# Before
# MAVROS QoS profile (BEST_EFFORT to match MAVROS)
mavros_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    ...
)

# After
# MAVROS QoS profile (RELIABLE to match MAVROS gps_rtk subscriber)
mavros_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    ...
)
```

---

## Resolution Steps

1. SSH to Jetson: `ssh jay@100.91.191.47`

2. Edit the forwarder script:
   ```bash
   nano ~/rtcm_mqtt_forwarder.py
   # Update MQTT_BROKER IP
   # Update QoS to RELIABLE
   ```

3. Restart the service:
   ```bash
   sudo systemctl restart jetson-rtcm-forwarder
   ```

4. Verify RTCM is flowing:
   ```bash
   source /opt/ros/humble/setup.bash
   export ROS_DOMAIN_ID=42
   ros2 topic hz /mavros/gps_rtk/send_rtcm
   # Should show ~3-4 Hz
   ```

---

## Verification

After fixes applied:

```
# Service logs show successful connection
Nov 21 17:11:47 Jetson1 bash: [INFO] Connecting to MQTT broker at 100.66.67.11:1883...
Nov 21 17:11:47 Jetson1 bash: [INFO] Connected to MQTT broker successfully
Nov 21 17:11:47 Jetson1 bash: [INFO] Subscribed to rtk/base/corrections

# RTCM topic publishing at ~3.8 Hz
$ ros2 topic hz /mavros/gps_rtk/send_rtcm
average rate: 3.824
```

---

## Lessons Learned

1. **Always update IP addresses** when migrating to Tailscale VPN
2. **Check QoS compatibility** between ROS2 publishers and subscribers - mismatches cause silent failures
3. **Use `ros2 topic info -v`** to see QoS profiles of publishers and subscribers

---

**Status:** RESOLVED
**Resolution Date:** November 21, 2025
**Resolved By:** Updated MQTT broker IP and fixed QoS mismatch
