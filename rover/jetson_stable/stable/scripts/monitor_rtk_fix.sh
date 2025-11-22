#!/bin/bash
# RTK Fix Type Monitor
# Watches GPS fix type progression: 3 (3D) → 5 (RTK Float) → 6 (RTK Fixed)

echo "========================================="
echo "RTK Fix Type Monitor - HERE 3+"
echo "========================================="
echo ""
echo "Fix Types:"
echo "  3 = 3D GPS (meter-level accuracy)"
echo "  4 = DGPS (sub-meter)"
echo "  5 = RTK Float (decimeter, converging...)"
echo "  6 = RTK Fixed (centimeter! 🎯)"
echo ""
echo "Monitoring every 5 seconds. Press Ctrl+C to stop."
echo ""

count=0
while true; do
    count=$((count + 1))
    timestamp=$(date '+%H:%M:%S')

    # Get GPS data from API
    gps_data=$(curl -s http://localhost:5001/api/gps 2>/dev/null)

    if [ ! -z "$gps_data" ]; then
        fix_type=$(echo "$gps_data" | python3 -c "import json, sys; data = json.load(sys.stdin); print(data.get('fix_type', 'N/A'))")
        satellites=$(echo "$gps_data" | python3 -c "import json, sys; data = json.load(sys.stdin); print(data.get('satellites', 'N/A'))")
        hdop=$(echo "$gps_data" | python3 -c "import json, sys; data = json.load(sys.stdin); print(data.get('hdop', 'N/A'))")
        h_acc=$(echo "$gps_data" | python3 -c "import json, sys; data = json.load(sys.stdin); print(data.get('h_acc', 'N/A'))")

        # Convert h_acc from mm to meters
        if [ "$h_acc" != "N/A" ]; then
            h_acc_m=$(echo "scale=2; $h_acc / 1000" | bc)
        else
            h_acc_m="N/A"
        fi

        # Determine status
        case $fix_type in
            3)
                status="3D GPS"
                icon="📍"
                ;;
            4)
                status="DGPS"
                icon="📡"
                ;;
            5)
                status="RTK FLOAT"
                icon="⏳"
                ;;
            6)
                status="RTK FIXED"
                icon="🎯"
                ;;
            *)
                status="UNKNOWN"
                icon="❓"
                ;;
        esac

        echo "[$timestamp] Check #$count:"
        echo "  $icon Status: $status (fix_type=$fix_type)"
        echo "  📡 Satellites: $satellites"
        echo "  📊 HDOP: $hdop"
        echo "  📏 H. Accuracy: ${h_acc_m}m (${h_acc}mm)"

        # Check if RTK Fixed achieved
        if [ "$fix_type" == "6" ]; then
            echo ""
            echo "========================================="
            echo "🎯 RTK FIXED ACHIEVED! 🎯"
            echo "Centimeter-level accuracy!"
            echo "========================================="
            echo ""
            break
        fi
    else
        echo "[$timestamp] Check #$count: No GPS data available"
    fi

    echo ""
    sleep 5
done
