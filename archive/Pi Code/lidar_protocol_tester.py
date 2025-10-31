#!/usr/bin/env python3
"""
LiDAR Protocol Compatibility Tester
Test different scanning methods to find working protocol
"""

import time
import rplidar

def test_protocol_method_1(lidar):
    """Test Method 1: Basic iter_scans with no parameters"""
    print("🧪 Method 1: Basic iter_scans()")
    try:
        scan_generator = lidar.iter_scans()
        scan = next(scan_generator)
        points = list(scan)
        print(f"   ✅ SUCCESS: {len(points)} points")
        return True, len(points)
    except Exception as e:
        print(f"   ❌ FAILED: {e}")
        return False, 0

def test_protocol_method_2(lidar):
    """Test Method 2: iter_scans with small buffer"""
    print("🧪 Method 2: iter_scans(max_buf_meas=50)")
    try:
        scan_generator = lidar.iter_scans(max_buf_meas=50)
        scan = next(scan_generator)
        points = list(scan)
        print(f"   ✅ SUCCESS: {len(points)} points")
        return True, len(points)
    except Exception as e:
        print(f"   ❌ FAILED: {e}")
        return False, 0

def test_protocol_method_3(lidar):
    """Test Method 3: iter_measurements"""
    print("🧪 Method 3: iter_measurements()")
    try:
        measurement_generator = lidar.iter_measurements()
        measurements = []
        
        # Collect 100 measurements (partial scan)
        for i, measurement in enumerate(measurement_generator):
            measurements.append(measurement)
            if i >= 100:
                break
        
        print(f"   ✅ SUCCESS: {len(measurements)} measurements")
        return True, len(measurements)
    except Exception as e:
        print(f"   ❌ FAILED: {e}")
        return False, 0

def test_protocol_method_4(lidar):
    """Test Method 4: iter_measurements with max_buf_meas"""
    print("🧪 Method 4: iter_measurements(max_buf_meas=50)")
    try:
        measurement_generator = lidar.iter_measurements(max_buf_meas=50)
        measurements = []
        
        # Collect 100 measurements (partial scan)
        for i, measurement in enumerate(measurement_generator):
            measurements.append(measurement)
            if i >= 100:
                break
        
        print(f"   ✅ SUCCESS: {len(measurements)} measurements")
        return True, len(measurements)
    except Exception as e:
        print(f"   ❌ FAILED: {e}")
        return False, 0

def test_protocol_method_5(lidar):
    """Test Method 5: Force standard scan mode"""
    print("🧪 Method 5: Force standard scan mode")
    try:
        # Try to force standard scan mode
        lidar.start_scan()
        time.sleep(1)  # Let scan start
        
        # Try to read raw data
        scan_generator = lidar.iter_scans()
        scan = next(scan_generator)
        points = list(scan)
        print(f"   ✅ SUCCESS: {len(points)} points")
        return True, len(points)
    except Exception as e:
        print(f"   ❌ FAILED: {e}")
        return False, 0

def test_working_method(lidar, method_func, duration=3):
    """Test a working method for sustained operation"""
    print(f"\n🔄 Testing {method_func.__name__} for {duration} seconds...")
    
    try:
        start_time = time.time()
        total_points = 0
        scan_count = 0
        
        if "measurement" in method_func.__name__:
            # Test iter_measurements method
            measurement_generator = lidar.iter_measurements(max_buf_meas=50) if "method_4" in method_func.__name__ else lidar.iter_measurements()
            
            while time.time() - start_time < duration:
                measurement = next(measurement_generator)
                total_points += 1
                
                if total_points % 100 == 0:
                    print(f"   📊 {total_points} measurements collected...")
        
        else:
            # Test iter_scans method
            if "method_2" in method_func.__name__:
                scan_generator = lidar.iter_scans(max_buf_meas=50)
            elif "method_5" in method_func.__name__:
                lidar.start_scan()
                time.sleep(0.5)
                scan_generator = lidar.iter_scans()
            else:
                scan_generator = lidar.iter_scans()
            
            while time.time() - start_time < duration:
                scan = next(scan_generator)
                points = list(scan)
                total_points += len(points)
                scan_count += 1
                
                print(f"   📊 Scan #{scan_count}: {len(points)} points")
        
        print(f"   ✅ Sustained test completed: {total_points} total points")
        return True
        
    except Exception as e:
        print(f"   ❌ Sustained test failed: {e}")
        return False

def main():
    print("=" * 60)
    print("🔬 LIDAR PROTOCOL COMPATIBILITY TESTER")
    print("=" * 60)
    
    try:
        # Connect to LiDAR
        print("🔌 Connecting to LiDAR...")
        lidar = rplidar.RPLidar('/dev/ttyUSB1')
        lidar.start_motor()
        time.sleep(2)
        print("✅ LiDAR connected and motor started")
        
        # Test different protocol methods
        methods = [
            test_protocol_method_1,
            test_protocol_method_2, 
            test_protocol_method_3,
            test_protocol_method_4,
            test_protocol_method_5
        ]
        
        working_methods = []
        
        print(f"\n🧪 Testing {len(methods)} different protocol methods...")
        
        for method in methods:
            success, points = method(lidar)
            if success:
                working_methods.append((method, points))
        
        # Report results
        print(f"\n📊 RESULTS:")
        if working_methods:
            print(f"✅ Found {len(working_methods)} working methods:")
            for method, points in working_methods:
                print(f"   • {method.__name__}: {points} points")
            
            # Test the best working method
            best_method = max(working_methods, key=lambda x: x[1])
            print(f"\n🏆 Best method: {best_method[0].__name__} ({best_method[1]} points)")
            
            # Extended test of best method
            if test_working_method(lidar, best_method[0]):
                print(f"✅ {best_method[0].__name__} works reliably!")
                print(f"\n💡 SOLUTION FOUND:")
                print(f"   Use {best_method[0].__name__.replace('test_protocol_', '').replace('_', ' ')} in your navigation code")
            
        else:
            print("❌ No working protocol methods found")
            print("🔧 This may require firmware update or different library version")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
    
    finally:
        try:
            lidar.stop_motor()
            lidar.disconnect()
            print("\n🧹 LiDAR disconnected")
        except:
            pass

if __name__ == "__main__":
    main()