# Xbox 360 Controller Configuration Update
**Date:** November 12, 2025
**Updated By:** Jay

---

## Changes Made

### Issue 1: Deadman Switch Removed
**Problem:** User had to hold button to enable movement (deadman switch)
**Solution:** Disabled enable button requirement

### Issue 2: No Rotation Control
**Problem:** Rover could move forward/backward but couldn't turn left/right
**Solution:**
- Verified axis mapping (axis 0 = left stick horizontal is correct)
- Increased angular scale from 0.4 to 0.8 rad/s for better responsiveness
- Added turbo angular scale of 1.2 rad/s

---

## Configuration File

**Created:** `xbox_rover.config.yaml`

```yaml
teleop_twist_joy_node:
  ros__parameters:
    # Left stick vertical (up/down) - Forward/Backward
    axis_linear:
      x: 1
    scale_linear:
      x: 0.7
    scale_linear_turbo:
      x: 1.5

    # Left stick horizontal (left/right) - Rotation
    axis_angular:
      yaw: 0
    scale_angular:
      yaw: 0.8  # Increased from 0.4 for better turning response
    scale_angular_turbo:
      yaw: 1.2

    # No deadman switch - always enabled
    enable_button: -1  # -1 means no button required
    enable_turbo_button: 5  # Right bumper for turbo (optional)

    # Disable deadman requirement
    require_enable_button: false
```

---

## Updated Controller Mapping

**Left Stick:**
- **Up/Down** → Forward/Backward (0.7 m/s, turbo: 1.5 m/s)
- **Left/Right** → Turn left/right (0.8 rad/s, turbo: 1.2 rad/s)

**Right Bumper (RB):**
- Hold for turbo speed boost (optional)

**No Deadman Switch:**
- Just move the stick to control the rover
- No button press required

---

## Files Modified

### 1. Created Custom Config
**File:** `xbox_rover.config.yaml`
- Custom teleop configuration
- Deadman switch disabled
- Increased angular scale for better turning

### 2. Updated Launch Script
**File:** `run_xbox_controller.sh`
**Changes:**
- Updated to use custom config file
- Updated help text (removed deadman switch instructions)
- Added config file path resolution

**Key change:**
```bash
ros2 launch teleop_twist_joy teleop-launch.py \
    config_filepath:="${SCRIPT_DIR}/xbox_rover.config.yaml"
```

### 3. Updated Documentation
**File:** `LOCAL_SIMULATION_GUIDE.md`
**Changes:**
- Updated controller mapping documentation
- Added speed specifications
- Removed deadman switch instructions

---

## Testing Results

**Verified:**
- ✅ Controller publishes to `/cmd_vel` without button press
- ✅ Angular axis (yaw) mapped to axis 0 (left stick horizontal)
- ✅ Linear axis mapped to axis 1 (left stick vertical)
- ✅ Angular scale increased from 0.4 to 0.8 rad/s
- ✅ Turbo mode works with right bumper

**Expected Behavior:**
- Move left stick up/down → Rover moves forward/backward
- Move left stick left/right → Rover rotates left/right
- Hold right bumper → Faster movement

---

## Troubleshooting

### If rover still doesn't turn:
1. Check controller is detected: `ls /dev/input/js0`
2. Check joy messages: `ros2 topic echo /joy`
3. Move left stick left/right and verify axis 0 changes
4. Check cmd_vel output: `ros2 topic echo /cmd_vel`
   - `angular.z` should be non-zero when turning

### If rover moves unexpectedly:
- May need to invert axes in config
- Adjust deadzone if stick drifts

---

**Status:** Complete and tested
**Ready for use!**
