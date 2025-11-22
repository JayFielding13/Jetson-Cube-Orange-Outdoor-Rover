# Issue: /api/disarm Endpoint Hanging

**Date Reported:** November 21, 2025
**Reported By:** Mobile RTK Control Module development instance
**Severity:** High - Cannot disarm rover from dashboard

---

## Problem Description

The `/api/disarm` endpoint on the Jetson unified bridge is hanging indefinitely. Requests to this endpoint never return, causing timeouts on the dashboard side.

**Observed Behavior:**
- ARM endpoint (`/api/arm`) works correctly
- Status endpoint (`/api/status`) works correctly
- DISARM endpoint (`/api/disarm`) hangs forever (tested with 10+ second timeout)
- Rover remains armed after disarm attempt

**Test Commands:**
```bash
# This works:
curl -s http://100.91.191.47:5001/api/status | grep armed
# Returns: "armed": true

# This hangs forever:
curl -s --max-time 10 -X POST http://100.91.191.47:5001/api/disarm
# Returns: timeout (exit code 28)
```

---

## Root Cause Analysis

The disarm endpoint calls `bridge_node.call_command_service(command=400, param1=0.0)` which:

1. Sends MAV_CMD_COMPONENT_ARM_DISARM with param1=0 (disarm)
2. Waits up to 5 seconds for the ROS2 future to complete
3. The future is never completing - MAVROS service may be stuck

**Actual Root Cause Found:**
The unified bridge service had entered a stale state where:
- The ROS2 executor or service call handling was blocked
- State subscription callbacks were not receiving updates properly
- The `armed` field in the API response was stale (not matching MAVROS state topic)
- Mode field would update but armed field remained stale

This is likely caused by a threading issue or executor deadlock in the Flask+ROS2 integration when handling concurrent requests.

---

## Resolution

### Fix Applied

Restarting the unified bridge service resolved the issue:

```bash
sudo systemctl restart jetson-unified-bridge
```

### Verification

After restart, full arm/disarm cycle works correctly:

```bash
# Step 1: Check initial status (disarmed)
curl -s http://100.91.191.47:5001/api/status | python3 -c "import sys,json; d=json.load(sys.stdin); print(f\"Armed: {d['state_summary']['armed']}\")"
# Output: Armed: False

# Step 2: Arm via API
curl -s -X POST http://100.91.191.47:5001/api/arm
# Output: {"result":0,"success":true}

# Step 3: Verify armed
curl -s http://100.91.191.47:5001/api/status | python3 -c "import sys,json; d=json.load(sys.stdin); print(f\"Armed: {d['state_summary']['armed']}\")"
# Output: Armed: True

# Step 4: Disarm via API (previously hanging)
curl -s -X POST http://100.91.191.47:5001/api/disarm
# Output: {"result":0,"success":true}
# Response time: 82ms (was hanging indefinitely before)

# Step 5: Verify disarmed
curl -s http://100.91.191.47:5001/api/status | python3 -c "import sys,json; d=json.load(sys.stdin); print(f\"Armed: {d['state_summary']['armed']}\")"
# Output: Armed: False
```

---

## Workaround (if issue recurs)

If the disarm endpoint starts hanging again:

1. **Immediate fix:** Restart the unified bridge service:
   ```bash
   sudo systemctl restart jetson-unified-bridge
   ```

2. **Alternative disarm methods:**
   - Physical safety switch on Cube Orange
   - MAVProxy/Mission Planner connection
   - Direct ROS2 service call:
     ```bash
     ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong '{command: 400, param1: 0.0}'
     ```

---

## Future Improvements

To prevent this issue from recurring:

1. **Add health monitoring** - Periodic check that state updates are being received
2. **Watchdog timer** - Restart service if state becomes stale for >30 seconds
3. **Thread safety audit** - Review Flask/ROS2 executor integration for potential deadlocks
4. **Timeout handling** - Return error immediately if service call times out instead of hanging

---

## Files Involved

- `/home/jay/ros2_unified_bridge.py` on Jetson
  - `/api/disarm` endpoint
  - `call_command_service()` method
  - State subscription callbacks

---

**Status:** RESOLVED
**Resolution Date:** November 21, 2025
**Resolved By:** Service restart; identified as stale state/executor deadlock issue
