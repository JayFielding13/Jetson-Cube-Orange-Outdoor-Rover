#!/usr/bin/env python3
"""
LiDAR Cleanup and Reset Script
Cleanly disconnects from LiDAR and resets the connection
"""

import time
import serial
from rplidar import RPLidar

def cleanup_lidar_port(port='/dev/ttyUSB1'):
    """Clean up LiDAR port connection"""
    print(f"🧹 Cleaning up LiDAR on {port}")
    
    # Try to connect and properly disconnect
    try:
        print("🔗 Attempting LiDAR connection...")
        lidar = RPLidar(port, baudrate=115200, timeout=1)
        
        try:
            print("🛑 Stopping any running scans...")
            lidar.stop()
            time.sleep(0.5)
        except:
            print("⚠️ No active scan to stop")
        
        try:
            print("🔄 Stopping motor...")
            lidar.stop_motor()
            time.sleep(1)
        except:
            print("⚠️ Motor stop failed")
        
        try:
            print("🔌 Disconnecting...")
            lidar.disconnect()
            time.sleep(0.5)
        except:
            print("⚠️ Disconnect failed")
        
        print("✅ LiDAR cleanup completed")
        
    except Exception as e:
        print(f"❌ Cleanup failed: {e}")
    
    # Try raw serial reset
    try:
        print("🔧 Attempting raw serial reset...")
        ser = serial.Serial(port, 115200, timeout=1)
        ser.close()
        time.sleep(0.5)
        print("✅ Serial reset completed")
    except Exception as e:
        print(f"❌ Serial reset failed: {e}")

def test_lidar_connection(port='/dev/ttyUSB1'):
    """Test basic LiDAR connection"""
    print(f"🧪 Testing LiDAR connection on {port}")
    
    try:
        lidar = RPLidar(port, baudrate=115200, timeout=3)
        
        print("📋 Getting device info...")
        info = lidar.get_info()
        print(f"✅ Device info: {info}")
        
        print("💚 Getting health status...")
        health = lidar.get_health()
        print(f"✅ Health: {health}")
        
        print("✅ LiDAR connection test successful")
        
        lidar.disconnect()
        return True
        
    except Exception as e:
        print(f"❌ LiDAR connection test failed: {e}")
        return False

def main():
    print("🔧 LiDAR Cleanup and Test Utility")
    print("=" * 40)
    
    # Step 1: Cleanup
    cleanup_lidar_port('/dev/ttyUSB1')
    
    # Step 2: Wait
    print("⏱️ Waiting 3 seconds...")
    time.sleep(3)
    
    # Step 3: Test
    if test_lidar_connection('/dev/ttyUSB1'):
        print("🎉 LiDAR is ready for use!")
    else:
        print("💥 LiDAR connection still has issues")

if __name__ == "__main__":
    main()