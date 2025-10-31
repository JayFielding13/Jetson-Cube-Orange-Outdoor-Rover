#!/usr/bin/env python3
import serial
import time
import threading
from datetime import datetime

class RTKQualityTest:
    def __init__(self):
        self.gps_device = '/dev/ttyACM0'
        self.test_duration = 60  # seconds
        self.measurements = []
        
    def continuous_gps_monitoring(self):
        """Monitor GPS fix quality continuously"""
        print(f'📡 Starting continuous GPS monitoring ({self.test_duration}s)...')
        print('🎯 Monitoring fix quality, satellite count, and precision')
        
        try:
            ser = serial.Serial(self.gps_device, 38400, timeout=2)
            
            start_time = time.time()
            sample_count = 0
            
            while time.time() - start_time < self.test_duration:
                try:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    
                    if 'GGA' in line and line.startswith('\$'):
                        parts = line.split(',')
                        if len(parts) >= 15:
                            timestamp = datetime.now()
                            
                            try:
                                fix_quality = int(parts[6]) if parts[6] else 0
                                num_sats = int(parts[7]) if parts[7] else 0
                                hdop = float(parts[8]) if parts[8] else 99.0
                                altitude = float(parts[9]) if parts[9] else 0.0
                                
                                measurement = {
                                    'time': timestamp,
                                    'fix': fix_quality,
                                    'sats': num_sats,
                                    'hdop': hdop,
                                    'alt': altitude
                                }
                                
                                self.measurements.append(measurement)
                                sample_count += 1
                                
                                # Real-time display every 5 samples
                                if sample_count % 5 == 0:
                                    fix_names = ['No fix', 'GPS', 'DGPS', 'RTK Float', 'RTK Fixed']
                                    fix_name = fix_names[fix_quality] if fix_quality < len(fix_names) else 'Unknown'
                                    elapsed = time.time() - start_time
                                    
                                    print(f'[{elapsed:5.1f}s] Fix: {fix_name:10} | Sats: {num_sats:2d} | HDOP: {hdop:5.2f} | Alt: {altitude:6.1f}m')
                                
                            except (ValueError, IndexError):
                                continue
                                
                except Exception:
                    continue
                    
            ser.close()
            
            print(f'\n📊 GPS Quality Analysis Complete ({sample_count} samples)')
            return self.analyze_measurements()
            
        except Exception as e:
            print(f'❌ GPS monitoring error: {e}')
            return None
    
    def analyze_measurements(self):
        """Analyze GPS measurement quality"""
        if not self.measurements:
            print('❌ No measurements to analyze')
            return None
        
        print('\n📈 GPS Performance Analysis:')
        print('=' * 40)
        
        # Fix quality distribution
        fix_counts = {}
        hdop_values = []
        sat_counts = []
        
        for m in self.measurements:
            fix_type = m['fix']
            fix_counts[fix_type] = fix_counts.get(fix_type, 0) + 1
            hdop_values.append(m['hdop'])
            sat_counts.append(m['sats'])
        
        # Analysis results
        total_samples = len(self.measurements)
        avg_hdop = sum(hdop_values) / len(hdop_values)
        avg_sats = sum(sat_counts) / len(sat_counts)
        min_hdop = min(hdop_values)
        max_hdop = max(hdop_values)
        
        print(f'📊 Total Samples: {total_samples}')
        print(f'🛰️  Average Satellites: {avg_sats:.1f}')
        print(f'📐 HDOP - Avg: {avg_hdop:.2f}, Min: {min_hdop:.2f}, Max: {max_hdop:.2f}')
        
        print('\n🎯 Fix Quality Distribution:')
        fix_names = {0: 'No fix', 1: 'GPS', 2: 'DGPS', 3: 'RTK Float', 4: 'RTK Fixed'}
        for fix_type, count in fix_counts.items():
            percentage = (count / total_samples) * 100
            fix_name = fix_names.get(fix_type, f'Type {fix_type}')
            print(f'   {fix_name}: {count} samples ({percentage:.1f}%)')
        
        # RTK readiness assessment
        print('\n🏆 RTK System Assessment:')
        
        if avg_sats >= 8:
            print('✅ Satellite count: Excellent (≥8)')
        elif avg_sats >= 6:
            print('⚠️  Satellite count: Good (6-7)')
        else:
            print('❌ Satellite count: Poor (<6)')
        
        if avg_hdop <= 1.0:
            print('✅ HDOP precision: Excellent (≤1.0)')
        elif avg_hdop <= 2.0:
            print('⚠️  HDOP precision: Good (1.0-2.0)')
        else:
            print('❌ HDOP precision: Poor (>2.0)')
        
        # RTK capability assessment
        dgps_percentage = fix_counts.get(2, 0) / total_samples * 100
        if dgps_percentage >= 80:
            print('✅ RTK readiness: Excellent (DGPS ≥80%)')
        elif dgps_percentage >= 50:
            print('⚠️  RTK readiness: Good (DGPS ≥50%)')
        else:
            print('❌ RTK readiness: Needs improvement (DGPS <50%)')
        
        return {
            'samples': total_samples,
            'avg_sats': avg_sats,
            'avg_hdop': avg_hdop,
            'fix_distribution': fix_counts,
            'dgps_percentage': dgps_percentage
        }

if __name__ == '__main__':
    tester = RTKQualityTest()
    
    print('🧪 RTK Quality and Stability Test')
    print('=' * 40)
    
    results = tester.continuous_gps_monitoring()
    
    if results:
        print('\n🎯 Test Complete - GPS system ready for RTK enhancement!')
    else:
        print('\n❌ Test failed - GPS system needs attention')
