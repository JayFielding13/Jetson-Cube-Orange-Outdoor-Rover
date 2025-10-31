#!/usr/bin/env python3
"""
Quick test to verify the target detection issue
"""

import time
import threading
from collections import deque

class TargetDetectionTest:
    def __init__(self):
        self.bluetooth_data = {
            'target_detected': False,
            'target_distance': 0.0,
            'target_direction': 0.0
        }
        self.robot_state = {'mode': 2}  # Autonomous
        self.running = True
        
    def bluetooth_thread(self):
        """Simulate Bluetooth detections"""
        time.sleep(2)
        print("🔍 Starting Bluetooth detection simulation...")
        
        for i in range(5):
            time.sleep(3)
            print(f"📡 Bluetooth scan {i+1} - Setting target_detected = True")
            self.bluetooth_data.update({
                'target_detected': True,
                'target_distance': 5.0,
                'target_direction': 0.0
            })
            time.sleep(2)
    
    def navigation_thread(self):
        """Simulate navigation reading"""
        nav_cycles = 0
        while self.running and nav_cycles < 50:
            nav_cycles += 1
            
            # Read bluetooth data (like the real navigation thread)
            target_detected = self.bluetooth_data['target_detected']
            target_distance = self.bluetooth_data['target_distance']
            
            if target_detected:
                print(f"🎯 NAV SEES TARGET: distance={target_distance:.1f}m (cycle {nav_cycles})")
            else:
                print(f"🔍 NAV: no target (cycle {nav_cycles})")
            
            time.sleep(0.5)
        
        self.running = False
    
    def run_test(self):
        print("🔧 Testing target detection timing...")
        
        bt_thread = threading.Thread(target=self.bluetooth_thread, daemon=True)
        nav_thread = threading.Thread(target=self.navigation_thread, daemon=True)
        
        bt_thread.start()
        nav_thread.start()
        
        nav_thread.join()
        print("✅ Test complete")

if __name__ == "__main__":
    test = TargetDetectionTest()
    test.run_test()