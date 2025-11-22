#!/bin/bash
# GPS Acquisition Monitor for HERE 3+ on Cube Orange
# Watches for GPS fix and satellite count via MAVROS2

export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash

echo "========================================="
echo "GPS Acquisition Monitor - HERE 3+"
echo "========================================="
echo ""
echo "Monitoring GPS status every 5 seconds..."
echo "Press Ctrl+C to stop"
echo ""
echo "Waiting for HERE 3+ to acquire satellites..."
echo "(This can take 1-5 minutes for cold start)"
echo ""

count=0
while true; do
    count=$((count + 1))
    timestamp=$(date '+%H:%M:%S')

    echo "[$timestamp] Check #$count:"

    # Check if GPS topic is publishing
    timeout 2 ros2 topic hz /mavros/global_position/global 2>&1 | grep -q "average rate"
    if [ $? -eq 0 ]; then
        echo "  ✅ GPS ACQUIRED! Topic is publishing!"

        # Get GPS data
        gps_data=$(timeout 2 ros2 topic echo /mavros/global_position/global --once 2>/dev/null)

        # Get satellite count
        sat_count=$(timeout 2 ros2 topic echo /mavros/global_position/raw/satellites --once 2>/dev/null | grep "data:" | awk '{print $2}')

        if [ ! -z "$sat_count" ]; then
            echo "  📡 Satellites: $sat_count"
        fi

        # Check RTK status
        rtk_data=$(timeout 2 ros2 topic echo /mavros/mavros/gps1/rtk --once 2>/dev/null | grep "rtk_health:" | awk '{print $2}')
        if [ ! -z "$rtk_data" ]; then
            echo "  🛰️  RTK Status: Health=$rtk_data (0=healthy)"
        fi

        echo ""
        echo "GPS is active! Check the dashboard or run:"
        echo "  curl http://192.168.254.100:5001/api/gps | python3 -m json.tool"
        echo ""
        break
    else
        echo "  ⏳ No GPS fix yet (HERE 3+ still searching for satellites...)"
        echo "     LED Status: Solid white = searching"
        echo "     Expected: Blinking = receiving satellites"
    fi

    echo ""
    sleep 5
done

echo "========================================="
echo "GPS acquisition complete!"
echo "========================================="
