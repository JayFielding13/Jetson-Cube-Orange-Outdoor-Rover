#!/usr/bin/env python3
"""
LiDAR Scan Debugger
Diagnose why scan data appears to be stuck/not updating
"""

import time
import rplidar

def debug_scan_freshness():
    """Test if we're getting fresh scan data each cycle"""
    print("=" * 60)
    print("🔬 LIDAR SCAN FRESHNESS DEBUGGER")
    print("=" * 60)
    
    try:
        # Connect to LiDAR
        print("🔌 Connecting to LiDAR...")
        lidar = rplidar.RPLidar('/dev/ttyUSB1')
        lidar.start_motor()
        time.sleep(2)
        print("✅ LiDAR connected and motor started")
        
        print("\n📊 Testing scan data freshness...")
        print("Collecting 10 consecutive scans to check for variations...")
        
        scan_data_history = []
        
        for scan_num in range(10):
            try:
                print(f"\n🔄 Scan #{scan_num + 1}:")
                
                # Get a scan
                scan_generator = lidar.iter_scans()
                scan = next(scan_generator)
                
                # Process the scan
                front_distances = []
                all_distances = []
                points_count = 0
                angle_samples = []
                
                for quality, angle, distance in scan:
                    if distance > 0 and quality > 5:
                        dist_cm = distance / 10.0
                        if 10 <= dist_cm <= 500:
                            all_distances.append(dist_cm)
                            angle_samples.append(angle)
                            points_count += 1
                            
                            # Front sector for comparison
                            if angle <= 45 or angle >= 315:
                                front_distances.append(dist_cm)
                
                # Calculate statistics
                front_min = min(front_distances) if front_distances else 0
                overall_min = min(all_distances) if all_distances else 0
                overall_max = max(all_distances) if all_distances else 0
                overall_avg = sum(all_distances) / len(all_distances) if all_distances else 0
                
                scan_stats = {
                    'points': points_count,
                    'front_min': front_min,
                    'overall_min': overall_min,
                    'overall_max': overall_max,
                    'overall_avg': overall_avg,
                    'angle_range': f"{min(angle_samples):.0f}° to {max(angle_samples):.0f}°" if angle_samples else "No angles",
                    'timestamp': time.time()
                }
                
                scan_data_history.append(scan_stats)
                
                print(f"   Points: {points_count}")
                print(f"   Front min: {front_min:.1f}cm")
                print(f"   Overall: min={overall_min:.1f}cm, max={overall_max:.1f}cm, avg={overall_avg:.1f}cm")
                print(f"   Angle range: {scan_stats['angle_range']}")
                
                # Check for data variation
                if scan_num > 0:
                    prev_scan = scan_data_history[scan_num - 1]
                    front_change = abs(scan_stats['front_min'] - prev_scan['front_min'])
                    avg_change = abs(scan_stats['overall_avg'] - prev_scan['overall_avg'])
                    
                    if front_change < 0.5 and avg_change < 1.0:
                        print(f"   ⚠️  Data appears STATIC (change: front={front_change:.1f}cm, avg={avg_change:.1f}cm)")
                    else:
                        print(f"   ✅ Data is DYNAMIC (change: front={front_change:.1f}cm, avg={avg_change:.1f}cm)")
                
                time.sleep(0.5)  # Wait between scans
                
            except Exception as e:
                print(f"   ❌ Scan {scan_num + 1} failed: {e}")
        
        # Analysis
        print(f"\n📈 ANALYSIS RESULTS:")
        if len(scan_data_history) >= 2:
            # Check overall variation
            front_values = [s['front_min'] for s in scan_data_history if s['front_min'] > 0]
            avg_values = [s['overall_avg'] for s in scan_data_history if s['overall_avg'] > 0]
            
            if front_values:
                front_variation = max(front_values) - min(front_values)
                avg_variation = max(avg_values) - min(avg_values)
                
                print(f"✅ Successfully collected {len(scan_data_history)} scans")
                print(f"📊 Front distance variation: {front_variation:.1f}cm")
                print(f"📊 Average distance variation: {avg_variation:.1f}cm")
                
                if front_variation < 2.0 and avg_variation < 5.0:
                    print("❌ PROBLEM: Scan data appears to be STATIC/CACHED")
                    print("💡 This suggests the LiDAR is returning the same scan repeatedly")
                    print("🔧 SOLUTION: Need to force fresh scans or clear buffers")
                else:
                    print("✅ GOOD: Scan data is dynamic and updating properly")
            else:
                print("❌ No valid front distance readings obtained")
        else:
            print("❌ Insufficient scan data collected")
    
    except Exception as e:
        print(f"❌ Debug test failed: {e}")
    
    finally:
        try:
            lidar.stop_motor()
            lidar.disconnect()
            print("\n🧹 LiDAR disconnected")
        except:
            pass

def debug_scan_timing():
    """Test scan timing and buffer behavior"""
    print("\n" + "=" * 60)
    print("⏱️  SCAN TIMING DEBUGGER")
    print("=" * 60)
    
    try:
        lidar = rplidar.RPLidar('/dev/ttyUSB1')
        lidar.start_motor()
        time.sleep(2)
        
        print("🔄 Testing scan acquisition timing...")
        
        scan_times = []
        
        for i in range(5):
            start_time = time.time()
            
            try:
                scan_generator = lidar.iter_scans()
                scan = next(scan_generator)
                
                # Count points quickly
                point_count = len(list(scan))
                
                scan_time = time.time() - start_time
                scan_times.append(scan_time)
                
                print(f"   Scan {i+1}: {scan_time:.3f}s, {point_count} points")
                
            except Exception as e:
                print(f"   Scan {i+1}: FAILED - {e}")
        
        if scan_times:
            avg_time = sum(scan_times) / len(scan_times)
            print(f"\n📊 Average scan time: {avg_time:.3f}s")
            print(f"📊 Scan rate: {1/avg_time:.1f} Hz")
            
            if avg_time > 1.0:
                print("⚠️  WARNING: Scans are very slow (>1s each)")
                print("💡 This could cause apparent 'stuck' readings")
            elif avg_time < 0.1:
                print("⚠️  WARNING: Scans are unusually fast (<0.1s)")
                print("💡 This might indicate cached/buffered data")
            else:
                print("✅ Scan timing appears normal")
    
    except Exception as e:
        print(f"❌ Timing test failed: {e}")
    
    finally:
        try:
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass

def main():
    print("🔬 Starting LiDAR scan debugging...")
    print("Purpose: Diagnose why scan distances appear stuck")
    print()
    
    # Test 1: Check if scan data is actually changing
    debug_scan_freshness()
    
    # Test 2: Check scan timing
    debug_scan_timing()
    
    print(f"\n💡 NEXT STEPS:")
    print("1. If data is STATIC: LiDAR may be caching scans - need different approach")
    print("2. If data is DYNAMIC: Problem is in navigation code processing")
    print("3. If timing is off: May need to adjust scan acquisition method")

if __name__ == "__main__":
    main()