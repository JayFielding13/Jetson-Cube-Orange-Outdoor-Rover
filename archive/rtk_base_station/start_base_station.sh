#!/bin/bash
echo "🏠 Starting RTK Base Station..."
echo "📍 GPS: /dev/ttyACM0 (u-blox receiver)"
echo "📡 NTRIP Server: localhost:2101"
echo "⏱️  Survey mode: Collecting position data..."

# Start RTK receiver in base mode
~/RTKLIB/app/rtkrcv/gcc/rtkrcv -o base_station.conf
