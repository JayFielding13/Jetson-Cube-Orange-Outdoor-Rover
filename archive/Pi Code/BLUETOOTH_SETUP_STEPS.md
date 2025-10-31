# Incremental Bluetooth BlueCharm Tracking Setup

## Prerequisites

### 1. Install Bluetooth Library
```bash
# In your virtual environment
source ~/rover_project/venv/bin/activate
pip install bleak==0.21.1

# Verify installation
python3 -c "import bleak; print('✅ Bleak installed successfully')"
```

### 2. Test Basic Bluetooth Scanning
```bash
# Quick test to see available Bluetooth devices
python3 -c "
import asyncio
from bleak import BleakScanner

async def scan():
    devices = await BleakScanner.discover(timeout=5.0)
    for device in devices:
        print(f'{device.name}: {device.address} ({device.rssi} dBm)')

asyncio.run(scan())
"
```

## Incremental Development Steps

### Step 1: Basic BlueCharm Detection (PASSIVE)
**Goal**: Detect and log BlueCharm presence without affecting navigation
- Add BlueCharm scanning to existing navigator
- Log detections with RSSI and estimated distance
- Navigation remains 100% ultrasonic
- Verify detection reliability over time

**Test**: Run rover and verify BlueCharm detection logs while it navigates normally

### Step 2: Distance-Based Information Display (MONITORING)
**Goal**: Show BlueCharm distance in status without navigation changes
- Display BlueCharm distance in navigation logs
- Track detection statistics
- Still 0% navigation influence
- Build confidence in distance accuracy

**Test**: Move BlueCharm closer/farther and verify distance estimates match reality

### Step 3: Gentle Approach Behavior (5% INFLUENCE)
**Goal**: Very subtle bias toward BlueCharm when safe
- 5% navigation weight toward BlueCharm
- Only when ultrasonic distance > 100cm (very safe)
- Easy to override and disable
- Monitor for any unexpected behaviors

**Test**: Place BlueCharm in open area, verify gentle approach without compromising safety

### Step 4: Moderate Following Behavior (15% INFLUENCE)
**Goal**: Noticeable but safe following behavior
- Increase influence to 15%
- Add distance-based speed adjustment
- Maintain all safety overrides
- Add manual BlueCharm mode toggle

**Test**: Test following in open area, verify safety overrides in obstacle scenarios

### Step 5: Active Beacon Following (30% INFLUENCE)
**Goal**: Primary following with safety backup
- Full beacon following capability
- Smart blending of ultrasonic safety + Bluetooth tracking
- Advanced stuck recovery with Bluetooth awareness
- Real-time parameter adjustment

**Test**: Complete mission scenarios with BlueCharm following

## Testing Protocol for Each Step

### Before Each Step:
1. **Backup current working code**
2. **Test ultrasonic-only mode** to ensure no regression
3. **Verify Arduino communication** is stable
4. **Check BlueCharm battery** and signal strength

### During Each Step:
1. **Start with short test runs** (30-60 seconds)
2. **Monitor console output** for unexpected behavior
3. **Keep emergency stop ready** (manual mode switch)
4. **Log all behaviors** for analysis

### Success Criteria for Each Step:
- **No navigation regression** in ultrasonic mode
- **Stable BlueCharm detection** (>80% scan success)
- **Predictable behavior** in both open and obstacle scenarios
- **Safe emergency stops** when needed

## Hardware Setup Tips

### BlueCharm Positioning:
- **Initial testing**: Place BlueCharm on table at waist height
- **Range testing**: Test at 1m, 3m, 5m, 10m distances
- **Obstacle testing**: Test with walls, furniture between rover and beacon
- **Movement testing**: Carry BlueCharm while rover follows

### Environment Considerations:
- **Start in open area** (garage, large room)
- **Avoid WiFi interference** zones if possible
- **Test both day and night** (temperature affects Bluetooth)
- **Note metal objects** that might affect signal

## Troubleshooting Common Issues

### BlueCharm Not Detected:
```bash
# Check if BlueCharm is advertising
sudo bluetoothctl
scan on
# Look for BlueCharm device
```

### Detection Intermittent:
- Check BlueCharm battery level
- Reduce scan timeout if too long
- Check for interference sources
- Move closer to verify signal

### Navigation Feels Sluggish:
- Reduce Bluetooth scan frequency
- Decrease Bluetooth influence weight
- Check Arduino communication timing

### Rover Gets Confused:
- Increase ultrasonic priority weight
- Add more conservative safety zones
- Reduce Bluetooth update rate

## Configuration Files for Each Step

I can create specific configuration files for each step to make testing easier and safer. Each step would have its own parameter set that you can quickly switch between.

Would you like me to create these step-by-step configuration files?