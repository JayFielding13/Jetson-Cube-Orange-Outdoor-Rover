#!/bin/bash
echo "🚀 Quick RTK Base Station (RTCM Server)"
echo "📍 GPS: /dev/ttyACM0 (u-blox)"  
echo "📡 Server: localhost:2101"
echo "📊 Streaming RTCM3 corrections..."

# Temporary base position (will improve with survey)
LAT=45.43037  # Portland area - adjust to your location
LON=-122.84203
ALT=50.0

# Start RTCM3 server
~/RTKLIB/app/str2str/gcc/str2str \
  -in serial://ttyACM0:38400#ubx \
  -out tcpsvr://:2101#rtcm3 \
  -p $LAT $LON $ALT \
  -msg 1074,1084,1094,1124,1004 \
  -sta 1001
