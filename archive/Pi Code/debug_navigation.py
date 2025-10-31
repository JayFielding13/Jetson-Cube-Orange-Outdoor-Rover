#!/usr/bin/env python3
"""
Debug navigation issue - simplified test
"""

import sys
import time

# Add the navigation logic from the main file
class MockNavigator:
    def __init__(self):
        self.nav_state = 'SEARCHING'
        self.nav_start_time = time.time()
        self.turn_direction = 'left'
        self.stuck_counter = 0
        
        # Movement parameters
        self.cruise_speed = 80
        self.search_speed = 60
        self.approach_speed = 50
        self.turn_speed = 70
        
        # Navigation thresholds
        self.obstacle_threshold = 40.0
        self.target_distance = 0.3
        self.target_tolerance = 0.2
        self.stats = {'obstacle_avoidances': 0, 'target_approaches': 0}
        
    def compute_tracking_navigation(self, obstacle_distance, target_detected, target_distance, target_bearing, current_time):
        """Compute navigation combining Bluetooth tracking with obstacle avoidance"""
        time_in_state = current_time - self.nav_start_time
        left_speed = 0
        right_speed = 0
        
        print(f"🔍 NAV INPUT: obstacle={obstacle_distance:.1f}cm, target={target_detected}, dist={target_distance:.1f}m, bearing={target_bearing:.1f}°")
        
        # Priority 1: Obstacle avoidance
        if obstacle_distance < self.obstacle_threshold:
            print("🚨 OBSTACLE DETECTED!")
            if self.nav_state != 'AVOIDING':
                self.nav_state = 'AVOIDING'
                self.nav_start_time = current_time
                self.turn_direction = 'left' if target_bearing >= 0 else 'right'
                self.stats['obstacle_avoidances'] += 1
                print(f"🚨 OBSTACLE! Avoiding {obstacle_distance:.1f}cm - turning {self.turn_direction}")
            
            # Execute avoidance
            if self.turn_direction == 'left':
                left_speed = -self.turn_speed
                right_speed = self.turn_speed
            else:
                left_speed = self.turn_speed
                right_speed = -self.turn_speed
        
        # Priority 2: Target tracking - PARK ON TARGET
        elif target_detected:
            print(f"🎯 TARGET LOGIC: distance={target_distance:.1f}m, tolerance={self.target_tolerance:.1f}m")
            if target_distance <= self.target_tolerance:
                # MISSION COMPLETE - Parked on target!
                if self.nav_state != 'MISSION_COMPLETE':
                    self.nav_state = 'MISSION_COMPLETE'
                    self.nav_start_time = current_time
                    print(f"🎉 MISSION COMPLETE! Parked on BlueCharm ({target_distance:.1f}m)")
                
                # Celebration behavior
                if time_in_state < 3.0:
                    left_speed = -self.turn_speed * 0.5
                    right_speed = self.turn_speed * 0.5
                else:
                    left_speed = 0
                    right_speed = 0
            else:
                # Approach target to park on it
                if self.nav_state != 'APPROACHING':
                    self.nav_state = 'APPROACHING'
                    self.nav_start_time = current_time
                    self.stats['target_approaches'] += 1
                    print(f"🏃 APPROACHING to park: {target_distance:.1f}m → {self.target_tolerance:.1f}m")
                
                # Calculate approach speed based on distance
                if target_distance > 2.0:
                    approach_speed = self.cruise_speed
                elif target_distance > 1.0:
                    approach_speed = self.approach_speed
                else:
                    approach_speed = self.approach_speed * 0.6
                
                # Navigate toward target with bearing correction
                if abs(target_bearing) > 20.0:
                    # Large bearing error - turn more aggressively
                    turn_factor = min(0.9, abs(target_bearing) / 45.0)
                    if target_bearing > 0:  # Target to the left
                        left_speed = approach_speed * (1.0 - turn_factor)
                        right_speed = approach_speed
                    else:  # Target to the right
                        left_speed = approach_speed
                        right_speed = approach_speed * (1.0 - turn_factor)
                else:
                    # Good heading - move straight toward target
                    left_speed = approach_speed
                    right_speed = approach_speed
        
        # Priority 3: Search for target
        else:
            if self.nav_state != 'SEARCHING':
                self.nav_state = 'SEARCHING'
                self.nav_start_time = current_time
                print("🔍 SEARCHING for Bluetooth target...")
            
            # Search pattern - slow rotation
            left_speed = -self.search_speed
            right_speed = self.search_speed
            print(f"🔍 SEARCH PATTERN: L={left_speed}, R={right_speed} | Time in state: {time_in_state:.1f}s")
        
        print(f"🚗 OUTPUT: L={left_speed}, R={right_speed} | State: {self.nav_state}")
        return left_speed, right_speed

def test_navigation():
    """Test navigation logic with the exact scenario from the logs"""
    nav = MockNavigator()
    
    print("=" * 60)
    print("🔍 TESTING NAVIGATION LOGIC")
    print("=" * 60)
    
    # Test case 1: Target detected at 7.9m (from logs)
    print("\n📋 TEST 1: Target detected at 7.9m")
    current_time = time.time()
    left, right = nav.compute_tracking_navigation(
        obstacle_distance=200.0,   # No obstacle
        target_detected=True,      # Target detected
        target_distance=7.9,       # 7.9m away
        target_bearing=0.0,        # Straight ahead
        current_time=current_time
    )
    print(f"✅ Result: L={left}, R={right}")
    
    # Test case 2: Target detected at 8.4m (from logs)
    print("\n📋 TEST 2: Target detected at 8.4m")
    current_time = time.time()
    left, right = nav.compute_tracking_navigation(
        obstacle_distance=200.0,   # No obstacle
        target_detected=True,      # Target detected
        target_distance=8.4,       # 8.4m away
        target_bearing=0.0,        # Straight ahead
        current_time=current_time
    )
    print(f"✅ Result: L={left}, R={right}")
    
    # Test case 3: No target detected (search mode)
    print("\n📋 TEST 3: No target detected (search mode)")
    nav.nav_state = 'SEARCHING'  # Reset to search
    nav.nav_start_time = time.time()
    current_time = time.time()
    left, right = nav.compute_tracking_navigation(
        obstacle_distance=200.0,   # No obstacle
        target_detected=False,     # No target
        target_distance=0.0,       # No distance
        target_bearing=0.0,        # No bearing
        current_time=current_time
    )
    print(f"✅ Result: L={left}, R={right}")
    
    print("\n" + "=" * 60)
    print("📊 NAVIGATION STATS:")
    print(f"   Obstacle avoidances: {nav.stats['obstacle_avoidances']}")
    print(f"   Target approaches: {nav.stats['target_approaches']}")
    print("=" * 60)

if __name__ == "__main__":
    test_navigation()