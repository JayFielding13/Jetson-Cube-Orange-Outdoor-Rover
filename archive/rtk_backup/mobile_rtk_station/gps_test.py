#!/usr/bin/env python3
"""
Basic GPS test script for SparkFun GPS-RTK-SMA
Tests UART communication and NMEA message parsing
"""

import serial
import pynmea2
import time
from datetime import datetime

def test_gps_connection():
    """Test basic GPS connection and message parsing"""
    print("=== Mobile RTK Station GPS Test ===")
    print("Connecting to GPS on /dev/serial0 at 38400 baud...")
    
    try:
        # Open serial connection to GPS
        gps_port = serial.Serial('/dev/serial0', 38400, timeout=1)
        print("✅ Serial connection established")
        
        message_count = 0
        position_count = 0
        satellites = 0
        fix_quality = 0
        
        print("\nListening for GPS messages... (Press Ctrl+C to stop)\n")
        
        while True:
            try:
                # Read line from GPS
                line = gps_port.readline().decode('ascii', errors='ignore').strip()
                
                if line:
                    message_count += 1
                    
                    # Parse different NMEA message types
                    if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                        # GGA: Position and fix data
                        try:
                            msg = pynmea2.parse(line)
                            if msg.latitude and msg.longitude:
                                position_count += 1
                                fix_quality = int(msg.gps_qual) if msg.gps_qual else 0
                                satellites = int(msg.num_sats) if msg.num_sats else 0
                                
                                # Determine fix type
                                fix_types = {
                                    0: "NO_FIX",
                                    1: "GPS_FIX", 
                                    2: "DGPS_FIX",
                                    4: "RTK_FIXED",
                                    5: "RTK_FLOAT"
                                }
                                fix_type = fix_types.get(fix_quality, "UNKNOWN")
                                
                                # Calculate accuracy estimate
                                if fix_quality == 4:  # RTK Fixed
                                    accuracy = "±2cm"
                                elif fix_quality == 5:  # RTK Float  
                                    accuracy = "±10cm"
                                elif fix_quality == 2:  # DGPS
                                    accuracy = "±1m"
                                elif fix_quality == 1:  # GPS
                                    accuracy = "±3m"
                                else:
                                    accuracy = "Unknown"
                                
                                print(f"📍 Position #{position_count:3d} | "
                                      f"Fix: {fix_type:10s} | "
                                      f"Sats: {satellites:2d} | "
                                      f"Lat: {float(msg.latitude):10.6f} | "
                                      f"Lon: {float(msg.longitude):11.6f} | "
                                      f"Alt: {float(msg.altitude):6.1f}m | "
                                      f"Acc: {accuracy}")
                                      
                        except pynmea2.ParseError:
                            pass  # Skip malformed messages
                            
                    # Show periodic summary
                    if message_count % 50 == 0:
                        print(f"\n📊 Summary: {message_count} messages received, "
                              f"{position_count} position fixes")
                        fix_types = {0: "NO_FIX", 1: "GPS_FIX", 2: "DGPS_FIX", 4: "RTK_FIXED", 5: "RTK_FLOAT"}
                        if fix_quality > 0:
                            print(f"Current status: {fix_types.get(fix_quality, 'UNKNOWN')} "
                                  f"with {satellites} satellites\n")
                        else:
                            print("Waiting for GPS fix...\n")
                            
            except KeyboardInterrupt:
                print(f"\n\n=== Test Summary ===")
                print(f"Total messages received: {message_count}")
                print(f"Position fixes received: {position_count}")
                fix_types = {0: "NO_FIX", 1: "GPS_FIX", 2: "DGPS_FIX", 4: "RTK_FIXED", 5: "RTK_FLOAT"}
                print(f"Final GPS status: {fix_types.get(fix_quality, 'UNKNOWN')} "
                      f"with {satellites} satellites")
                break
                
            except Exception as e:
                print(f"Error parsing message: {e}")
                continue
                
    except serial.SerialException as e:
        print(f"❌ Serial connection error: {e}")
        print("Check wiring and UART configuration")
        return False
        
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False
        
    finally:
        if 'gps_port' in locals():
            gps_port.close()
            print("GPS connection closed")
            
    return True

if __name__ == "__main__":
    test_gps_connection()
