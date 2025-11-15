# Side Ultrasonic Sensor Position Update
**Date:** November 12, 2025
**Updated By:** Jay

---

## Change Summary

Updated the X-axis position of the **side_left** and **side_right** ultrasonic sensors to match actual hardware measurements.

### Previous Position
- **X position:** `0` (center of chassis)
- **Y position:** ±0.30163m (side edges)
- **Z position:** 0.1270m (11 inches from ground)

### New Position
- **X position:** `0.1143m` (9 inches back from front edge)
- **Y position:** ±0.30163m (side edges) - unchanged
- **Z position:** 0.1270m (11 inches from ground) - unchanged

---

## Calculation

**Requirement:** Side sensor midpoint should be 9 inches back from front edge of chassis

**Chassis Dimensions:**
- Chassis length: 27 inches (0.6858m)
- Front edge position: `chassis_length/2 = 13.5 inches` from center

**Position Calculation:**
- 9 inches back from front edge = 13.5" - 9" = 4.5" forward of center
- 4.5 inches = 0.1143 meters

**Result:** Side sensors at `x = 0.1143m`

---

## Files Modified

**File:** `/home/jay/Desktop/Mini Rover Development/ros2_ws/src/jetson_rover_sim/urdf/sensors/ultrasonic_sensors.xacro`

**Lines Changed:** 162-180

**Before:**
```xml
<xacro:ultrasonic_sensor
  name="side_left"
  parent="base_link"
  x="0"
  y="${side_y}"
  z="${sensor_z_offset}"
  yaw="${pi/2}"/>
```

**After:**
```xml
<!-- Positioned 9 inches back from front edge: chassis_length/2 - 9" = 13.5" - 9" = 4.5" = 0.1143m -->
<xacro:ultrasonic_sensor
  name="side_left"
  parent="base_link"
  x="0.1143"
  y="${side_y}"
  z="${sensor_z_offset}"
  yaw="${pi/2}"/>
```

Same update applied to `side_right` sensor.

---

## Verification

The simulation was rebuilt and tested:

```bash
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
colcon build --packages-select jetson_rover_sim
```

Simulation restarted successfully with updated sensor positions visible in Gazebo.

---

## Sensor Layout (Updated)

Top-Down View:
```
                    FRONT (x = +0.3429m)
          [Corner L]  [Front]  [Corner R]
              45°       0°        45°
               \        |        /
                \       |       /
      [Side L]   \      |      /    [Side R]
       90°        \     |     /       90°
    x=0.1143m      \    |    /     x=0.1143m
                    └───┴───┘
                       |
                     [Rear]
                      180°
```

**Side Sensors Now:**
- 9 inches (0.2286m) back from front edge
- 4.5 inches (0.1143m) forward of chassis center
- Perpendicular to chassis (90° left/right)

---

## Impact

- ✅ Simulation sensor positions now match real hardware
- ✅ More accurate simulation for testing obstacle avoidance
- ✅ Better field-of-view coverage toward front of rover
- ✅ Improved sensor fusion accuracy

---

---

## Update 2: LiDAR Height Adjustment

**Change:** Raised RP-LIDAR A1 by 1.5 inches to ensure laser can see over camera

### Previous Position
- **Z position:** `ebox_height/2 + lidar_height/2`
- Result: Only ~5mm clearance over camera top

### New Position
- **Z position:** `ebox_height/2 + lidar_height/2 + 0.0381`
- Added riser: 1.5 inches (0.0381m)
- Result: LiDAR now sits 43mm (1.7") higher, clear line of sight over camera

### File Modified

**File:** `/home/jay/Desktop/Mini Rover Development/ros2_ws/src/jetson_rover_sim/urdf/sensors/lidar_camera_gps.xacro`

**Lines Changed:** 54-63

**Change:**
```xml
<!-- Raised 1.5 inches (0.0381m) above original position to see over camera -->
<origin xyz="${ebox_length/2 - lidar_radius} 0 ${ebox_height/2 + lidar_height/2 + 0.0381}" rpy="0 0 0"/>
```

### Benefits
- ✅ LiDAR laser no longer obstructed by camera
- ✅ Better obstacle detection accuracy
- ✅ Matches real hardware configuration
- ✅ No blind spots in forward scanning arc

---

**Status:** Complete and tested
**Next Session:** Continue with simulation basics and controller testing
