#!/usr/bin/env python3
"""
LiDAR Diagnostic Tool
Comprehensive testing and troubleshooting for RPLidar A1M8
"""

import time
import sys

try:
    import rplidar
    print("✅ RPLidar library imported successfully")
    print(f"📦 Library version: {rplidar.__version__ if hasattr(rplidar, '__version__') else 'Unknown'}")
except ImportError as e:
    print(f"❌ Failed to import RPLidar library: {e}")
    sys.exit(1)

def test_lidar_connection(port='/dev/ttyUSB1'):
    """Test basic LiDAR connection and info"""
    print(f"\n🔌 Testing LiDAR connection on {port}...")
    
    try:
        lidar = rplidar.RPLidar(port)
        print("✅ LiDAR object created successfully")
        
        # Get device info
        print("\n📋 Getting device information...")
        try:
            info = lidar.get_info()
            print(f"✅ Device info: {info}")
        except Exception as e:
            print(f"⚠️ Failed to get device info: {e}")
        
        # Get device health
        print("\n🏥 Checking device health...")
        try:
            health = lidar.get_health()
            print(f"✅ Device health: {health}")
        except Exception as e:
            print(f"⚠️ Failed to get device health: {e}")
        
        # Start motor
        print("\n🔄 Starting motor...")
        try:
            lidar.start_motor()
            print("✅ Motor started successfully")
            time.sleep(2)  # Let motor spin up
        except Exception as e:
            print(f"⚠️ Failed to start motor: {e}")
        
        return lidar
        
    except Exception as e:
        print(f"❌ Failed to create LiDAR connection: {e}")
        return None

def test_basic_scan(lidar, duration=5):
    """Test basic scanning functionality"""
    print(f"\n📡 Testing basic scan for {duration} seconds...")
    
    try:
        scan_count = 0
        point_count = 0
        start_time = time.time()
        
        print("📊 Starting scan data collection...")
        
        for scan in lidar.iter_scans():
            scan_count += 1
            current_points = len(list(scan))
            point_count += current_points
            
            print(f"   Scan #{scan_count}: {current_points} points")
            
            if time.time() - start_time > duration:
                break
        
        print(f"\n✅ Scan test completed:")
        print(f"   Total scans: {scan_count}")
        print(f"   Total points: {point_count}")
        print(f"   Average points per scan: {point_count/scan_count if scan_count > 0 else 0:.1f}")
        
        return True
        
    except Exception as e:
        print(f"❌ Scan test failed: {e}")
        return False

def test_scan_with_buffer_sizes(lidar):
    """Test different buffer sizes to find optimal setting"""
    print(f"\n🔧 Testing different buffer sizes...")
    
    buffer_sizes = [50, 100, 150, 200, 250, 300]
    
    for buf_size in buffer_sizes:
        print(f"\n   Testing buffer size: {buf_size}")
        try:
            scan_generator = lidar.iter_scans(max_buf_meas=buf_size)
            scan = next(scan_generator)
            points = len(list(scan))
            print(f"   ✅ Buffer {buf_size}: {points} points - SUCCESS")
            
        except Exception as e:
            print(f"   ❌ Buffer {buf_size}: FAILED - {e}")

def test_scan_quality_analysis(lidar, duration=3):
    """Analyze scan quality and data integrity"""
    print(f"\n🔍 Analyzing scan quality for {duration} seconds...")
    
    try:
        total_points = 0
        valid_points = 0
        quality_sum = 0
        distance_sum = 0
        angle_coverage = set()
        
        start_time = time.time()
        
        for scan in lidar.iter_scans(max_buf_meas=100):
            for quality, angle, distance in scan:
                total_points += 1
                
                if distance > 0 and quality > 0:
                    valid_points += 1
                    quality_sum += quality
                    distance_sum += distance
                    angle_coverage.add(int(angle))
            
            if time.time() - start_time > duration:
                break
        
        print(f"\n📊 Quality Analysis Results:")
        print(f"   Total points: {total_points}")
        print(f"   Valid points: {valid_points} ({100*valid_points/total_points if total_points > 0 else 0:.1f}%)")
        print(f"   Average quality: {quality_sum/valid_points if valid_points > 0 else 0:.1f}")
        print(f"   Average distance: {distance_sum/valid_points/10 if valid_points > 0 else 0:.1f} cm")
        print(f"   Angle coverage: {len(angle_coverage)} degrees out of 360")
        
        return True
        
    except Exception as e:
        print(f"❌ Quality analysis failed: {e}")
        return False

def test_simple_obstacle_detection(lidar, duration=5):
    """Test basic obstacle detection in front sector"""
    print(f"\n🚨 Testing obstacle detection for {duration} seconds...")
    print("   Place an object in front of the LiDAR to test detection")
    
    try:
        start_time = time.time()
        detections = []
        
        for scan in lidar.iter_scans(max_buf_meas=100):
            front_distances = []
            
            for quality, angle, distance in scan:
                if distance > 0 and quality > 5:
                    # Front sector: 0° ± 45°
                    if angle <= 45 or angle >= 315:
                        front_distances.append(distance / 10.0)  # Convert to cm
            
            if front_distances:
                min_front = min(front_distances)
                detections.append(min_front)
                
                if min_front < 100:  # Object detected within 1 meter
                    print(f"   🎯 Object detected: {min_front:.1f}cm")
            
            if time.time() - start_time > duration:
                break
        
        if detections:
            print(f"\n✅ Detection test completed:")
            print(f"   Measurements taken: {len(detections)}")
            print(f"   Closest detection: {min(detections):.1f}cm")
            print(f"   Farthest detection: {max(detections):.1f}cm")
        else:
            print(f"\n⚠️ No valid detections recorded")
        
        return True
        
    except Exception as e:
        print(f"❌ Obstacle detection test failed: {e}")
        return False

def cleanup_lidar(lidar):
    """Safely cleanup LiDAR connection"""
    if lidar:
        try:
            print("\n🧹 Cleaning up LiDAR connection...")
            lidar.stop_motor()
            lidar.disconnect()
            print("✅ LiDAR cleanup completed")
        except Exception as e:
            print(f"⚠️ Cleanup warning: {e}")

def main():
    print("=" * 60)
    print("🔬 LIDAR DIAGNOSTIC TOOL")
    print("=" * 60)
    print("Purpose: Comprehensive LiDAR testing and troubleshooting")
    print()
    
    # Test 1: Basic connection
    lidar = test_lidar_connection()
    if not lidar:
        print("\n❌ Cannot proceed - LiDAR connection failed")
        return
    
    try:
        # Test 2: Basic scanning
        if test_basic_scan(lidar, duration=3):
            print("✅ Basic scanning works")
        else:
            print("❌ Basic scanning failed")
            return
        
        # Test 3: Buffer size optimization
        test_scan_with_buffer_sizes(lidar)
        
        # Test 4: Quality analysis
        if test_scan_quality_analysis(lidar, duration=3):
            print("✅ Quality analysis completed")
        
        # Test 5: Obstacle detection
        if test_simple_obstacle_detection(lidar, duration=5):
            print("✅ Obstacle detection test completed")
        
        print(f"\n🎉 DIAGNOSTIC COMPLETE")
        print("📋 Summary: Check results above to identify optimal settings")
        
    except KeyboardInterrupt:
        print("\n🛑 Diagnostic interrupted by user")
    
    finally:
        cleanup_lidar(lidar)

if __name__ == "__main__":
    main()