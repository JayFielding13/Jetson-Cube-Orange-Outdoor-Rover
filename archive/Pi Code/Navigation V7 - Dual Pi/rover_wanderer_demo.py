#!/usr/bin/env python3
"""
Rover Wanderer Demo - Complete Dual Pi Demonstration
Orchestrates the complete dual-Pi intelligent wandering system

This script helps coordinate:
1. Navigation Pi: Intelligent wandering behavior
2. Companion Pi: Real-time sensor visualization  
3. Development PC: Professional monitoring interface

Perfect for demonstrations and testing your dual sensor rover!
"""

import subprocess
import threading
import time
import signal
import sys
import socket
import json
from datetime import datetime

# Network configuration
NAVIGATION_PI_IP = '192.168.254.65'
COMPANION_PI_IP = '192.168.254.70'
VISUALIZATION_PORT = 5555

class RoverWandererDemo:
    def __init__(self):
        self.running = False
        self.processes = {}
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\n🛑 Demo stopped by signal {signum}")
        self.stop_demo()
        sys.exit(0)
    
    def print_header(self):
        """Print impressive demo header"""
        print("\n" + "="*70)
        print("🤖 INTELLIGENT ROVER WANDERER DEMONSTRATION")
        print("🎯 Dual Pi Architecture with Advanced Object Avoidance")
        print("="*70)
        print("📡 Navigation Pi (192.168.254.65): Intelligent behavior engine")
        print("🎨 Companion Pi (192.168.254.70): Real-time visualization")
        print("🖥️  Development PC: Professional monitoring interface")
        print("="*70)
    
    def check_network_connectivity(self):
        """Check if both Pis are accessible"""
        print("🔍 Checking network connectivity...")
        
        nav_pi_ok = self.ping_host(NAVIGATION_PI_IP)
        companion_pi_ok = self.ping_host(COMPANION_PI_IP)
        
        print(f"   Navigation Pi ({NAVIGATION_PI_IP}): {'✅ Reachable' if nav_pi_ok else '❌ Not reachable'}")
        print(f"   Companion Pi ({COMPANION_PI_IP}): {'✅ Reachable' if companion_pi_ok else '❌ Not reachable'}")
        
        if not nav_pi_ok:
            print("⚠️  Warning: Navigation Pi not reachable - demo will run in local mode")
        if not companion_pi_ok:
            print("⚠️  Warning: Companion Pi not reachable - no remote visualization")
        
        return nav_pi_ok, companion_pi_ok
    
    def ping_host(self, host):
        """Simple connectivity check"""
        try:
            result = subprocess.run(['ping', '-c', '1', '-W', '2', host], 
                                  capture_output=True, timeout=5)
            return result.returncode == 0
        except:
            return False
    
    def check_visualization_server(self):
        """Check if Companion Pi visualization server is running"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3)
            result = sock.connect_ex((COMPANION_PI_IP, VISUALIZATION_PORT))
            sock.close()
            return result == 0
        except:
            return False
    
    def start_local_components(self):
        """Start components that should run locally"""
        print("\n🚀 Starting local demo components...")
        
        # Start development PC visualization receiver
        print("   🖥️  Starting development PC visualization receiver...")
        try:
            receiver_process = subprocess.Popen([
                'python3', 'dev_pc_visualization_receiver.py'
            ], cwd='../../../Dashboard/Dashboard V3.5 - Dual Pi Management/')
            self.processes['receiver'] = receiver_process
            print("   ✅ Visualization receiver started")
        except Exception as e:
            print(f"   ⚠️  Could not start receiver: {e}")
        
        time.sleep(2)  # Let receiver initialize
    
    def display_instructions(self, nav_pi_ok, companion_pi_ok):
        """Display setup instructions for remote Pis"""
        print("\n📋 SETUP INSTRUCTIONS:")
        
        if nav_pi_ok:
            print(f"\n🎯 On Navigation Pi ({NAVIGATION_PI_IP}):")
            print("   ssh pi@192.168.254.65")
            print("   cd '/path/to/Pi Code/Navigation V7 - Dual Pi/'")
            print("   python3 intelligent_rover_wanderer.py")
        else:
            print(f"\n🎯 Navigation Pi ({NAVIGATION_PI_IP}) - LOCAL MODE:")
            print("   Run locally: python3 intelligent_rover_wanderer.py")
        
        if companion_pi_ok:
            print(f"\n🎨 On Companion Pi ({COMPANION_PI_IP}):")
            print("   ssh pi@192.168.254.70")
            print("   cd '/path/to/Pi Code/Navigation V7 - Dual Pi/'") 
            print("   python3 companion_pi_sensor_visualizer.py")
        else:
            print(f"\n🎨 Companion Pi ({COMPANION_PI_IP}) - NOT AVAILABLE:")
            print("   Visualization will not be available")
        
        print(f"\n🖥️  Development PC (this machine):")
        print("   ✅ Visualization receiver should be starting automatically")
        print("   📡 Connect to Companion Pi when both rover scripts are running")
    
    def monitor_demo(self):
        """Monitor the demo and provide status updates"""
        print("\n📊 DEMO MONITORING STARTED")
        print("   Monitoring system status every 30 seconds...")
        print("   🛑 Press Ctrl+C to stop demo\n")
        
        start_time = time.time()
        
        while self.running:
            try:
                # Check process health
                self.check_process_health()
                
                # Check visualization server
                viz_running = self.check_visualization_server()
                
                # Display status
                runtime = time.time() - start_time
                print(f"\n📈 DEMO STATUS - {datetime.now().strftime('%H:%M:%S')}")
                print(f"   Runtime: {runtime:.0f} seconds")
                print(f"   Visualization Server: {'✅ Running' if viz_running else '❌ Not detected'}")
                print(f"   Local Processes: {len([p for p in self.processes.values() if p and p.poll() is None])} active")
                
                if viz_running:
                    print("   🎉 Full system operational! Check development PC for live visualization.")
                else:
                    print("   💡 Start Companion Pi visualizer to see real-time sensor display")
                
                time.sleep(30)  # Status update every 30 seconds
                
            except Exception as e:
                print(f"⚠️  Monitor error: {e}")
                time.sleep(10)
    
    def check_process_health(self):
        """Check health of local processes"""
        for name, process in list(self.processes.items()):
            if process and process.poll() is not None:
                print(f"⚠️  Process {name} has stopped (exit code: {process.returncode})")
                # Could restart process here if desired
    
    def create_demo_summary(self):
        """Create a summary of what the demo showcases"""
        summary = """
🎯 INTELLIGENT ROVER WANDERER DEMO FEATURES:

🧠 Advanced Behaviors:
   • Curiosity-driven exploration with adaptive learning
   • Smart obstacle avoidance using dual 30° sensors  
   • Object investigation and circling behaviors
   • Stuck detection with intelligent escape maneuvers
   • Path memory to avoid repetitive loops

🎨 Real-time Visualization:
   • Live radar display showing sensor detection cones
   • Obstacle positions plotted with directional awareness
   • Color-coded alerts for emergency situations
   • Professional development PC interface

🚀 Dual Pi Performance:
   • Navigation Pi: 100% focused on intelligent decisions
   • Companion Pi: Dedicated visualization processing
   • Development PC: Enhanced monitoring and analysis

📊 Learning Analytics:
   • Behavioral adaptation based on environment
   • Obstacle encounter tracking and analysis
   • Curiosity level adjustments over time
   • Performance statistics and insights

🎮 Interactive Elements:
   • Try placing objects at different angles (30° left/right)
   • Watch rover investigate interesting stationary objects
   • Observe escape behaviors when rover gets cornered
   • See curiosity-driven exploration in open areas
        """
        
        print(summary)
    
    def stop_demo(self):
        """Stop all demo processes"""
        print("\n🛑 Stopping rover wanderer demo...")
        self.running = False
        
        # Stop local processes
        for name, process in self.processes.items():
            if process and process.poll() is None:
                print(f"   Stopping {name}...")
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
        
        print("✅ Demo stopped cleanly")
    
    def run_interactive_demo(self):
        """Run interactive demo mode"""
        print("\n🎮 INTERACTIVE DEMO MODE")
        print("This mode provides guidance for running the complete system")
        
        while self.running:
            print("\n" + "="*50)
            print("DEMO OPTIONS:")
            print("1. 📊 Show system status")
            print("2. 🎯 Display rover behavior features") 
            print("3. 📡 Check network connectivity")
            print("4. 🎨 Check visualization server")
            print("5. 📋 Show setup instructions")
            print("6. 🛑 Exit demo")
            
            try:
                choice = input("\nSelect option (1-6): ").strip()
                
                if choice == '1':
                    self.show_system_status()
                elif choice == '2':
                    self.create_demo_summary()
                elif choice == '3':
                    self.check_network_connectivity()
                elif choice == '4':
                    viz_status = self.check_visualization_server()
                    print(f"Visualization server: {'✅ Running' if viz_status else '❌ Not running'}")
                elif choice == '5':
                    nav_ok, comp_ok = self.check_network_connectivity()
                    self.display_instructions(nav_ok, comp_ok)
                elif choice == '6':
                    break
                else:
                    print("Invalid option. Please try again.")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"⚠️  Error: {e}")
    
    def show_system_status(self):
        """Show detailed system status"""
        nav_ok, comp_ok = self.check_network_connectivity()
        viz_running = self.check_visualization_server()
        
        print("\n📊 COMPLETE SYSTEM STATUS:")
        print(f"   Navigation Pi: {'🟢 Online' if nav_ok else '🔴 Offline'}")
        print(f"   Companion Pi: {'🟢 Online' if comp_ok else '🔴 Offline'}")
        print(f"   Visualization: {'🟢 Running' if viz_running else '🔴 Not running'}")
        print(f"   Demo Processes: {len([p for p in self.processes.values() if p and p.poll() is None])} active")
        
        if nav_ok and comp_ok and viz_running:
            print("\n🎉 FULL SYSTEM READY FOR DEMONSTRATION!")
        elif nav_ok or comp_ok:
            print("\n💡 PARTIAL SYSTEM - Some features available")
        else:
            print("\n⚠️  LOCAL MODE - Run scripts manually for full demo")
    
    def run(self):
        """Main demo execution"""
        self.print_header()
        self.create_demo_summary()
        
        # Check system readiness
        nav_pi_ok, companion_pi_ok = self.check_network_connectivity()
        
        # Start local components
        self.start_local_components()
        
        # Display setup instructions
        self.display_instructions(nav_pi_ok, companion_pi_ok)
        
        # Set running flag
        self.running = True
        
        print("\n🚀 ROVER WANDERER DEMO ACTIVE!")
        print("   💡 Follow the setup instructions above to start the complete system")
        print("   🎯 This demo coordinator will monitor and guide the process")
        
        try:
            # Choose between interactive and monitoring mode
            mode = input("\nDemo mode - (i)nteractive or (m)onitor? [i]: ").lower().strip()
            
            if mode == 'm' or mode == 'monitor':
                self.monitor_demo()
            else:
                self.run_interactive_demo()
                
        except KeyboardInterrupt:
            print("\n🛑 Demo interrupted by user")
        finally:
            self.stop_demo()

def main():
    print("🤖 Starting Rover Wanderer Demo Coordinator...")
    
    demo = RoverWandererDemo()
    demo.run()
    
    print("✅ Demo coordinator finished")
    return 0

if __name__ == "__main__":
    sys.exit(main())