#!/usr/bin/env python3
"""
LiDAR Library Version Check and Upgrade Guide
"""

import sys
import subprocess
import pkg_resources

def check_current_version():
    """Check current RPLidar library version"""
    print("🔍 Checking current RPLidar library version...")
    
    try:
        import rplidar
        
        # Try different ways to get version
        version_found = False
        
        if hasattr(rplidar, '__version__'):
            print(f"✅ Version: {rplidar.__version__}")
            version_found = True
        
        # Check via pkg_resources
        try:
            version = pkg_resources.get_distribution('rplidar-roboticia').version
            print(f"✅ Package version: {version}")
            version_found = True
        except:
            pass
        
        # Check available methods
        print(f"\n📋 Available methods in RPLidar class:")
        methods = [method for method in dir(rplidar.RPLidar) if not method.startswith('_')]
        for method in sorted(methods):
            print(f"   • {method}")
        
        # Check critical methods
        critical_methods = ['iter_scans', 'iter_measurements', 'start_scan', 'get_info', 'get_health']
        print(f"\n🔧 Critical method availability:")
        for method in critical_methods:
            available = hasattr(rplidar.RPLidar, method)
            status = "✅" if available else "❌"
            print(f"   {status} {method}")
        
        if not version_found:
            print("⚠️ Version information not available")
        
        return True
        
    except ImportError:
        print("❌ RPLidar library not installed")
        return False

def show_upgrade_instructions():
    """Show upgrade instructions"""
    print(f"\n🔧 UPGRADE INSTRUCTIONS:")
    print(f"=" * 50)
    
    print(f"\n1️⃣ Check current installation:")
    print(f"   pip list | grep rplidar")
    
    print(f"\n2️⃣ Uninstall old version:")
    print(f"   pip uninstall rplidar")
    print(f"   pip uninstall rplidar-roboticia")
    
    print(f"\n3️⃣ Install latest version:")
    print(f"   pip install rplidar-roboticia")
    
    print(f"\n4️⃣ Alternative: Install from source:")
    print(f"   git clone https://github.com/SkoltechRobotics/rplidar.git")
    print(f"   cd rplidar")
    print(f"   pip install .")
    
    print(f"\n5️⃣ Verify installation:")
    print(f"   python3 -c \"import rplidar; print('Success')\"")

def test_simple_alternative():
    """Test a very simple alternative approach"""
    print(f"\n🧪 Testing simple alternative approach...")
    
    try:
        import rplidar
        
        print("📡 Attempting basic connection...")
        lidar = rplidar.RPLidar('/dev/ttyUSB1')
        
        print("🔄 Starting motor...")
        lidar.start_motor()
        
        print("📊 Trying direct scan access...")
        # Try to access scan data directly without iter_scans
        try:
            # Check if we can at least connect and get basic info
            info = lidar.get_info()
            health = lidar.get_health()
            
            print(f"✅ Connection successful:")
            print(f"   Model: {info.get('model')}")
            print(f"   Firmware: {info.get('firmware')}")
            print(f"   Health: {health}")
            
            # Try raw scan data access
            print("🔍 Attempting raw data access...")
            
            # This is a very basic approach that might work with older libraries
            for i in range(5):
                try:
                    # Try to get any scan data
                    raw_data = next(iter(lidar.iter_scans()))
                    print(f"   Attempt {i+1}: Got some data!")
                    break
                except Exception as e:
                    print(f"   Attempt {i+1}: {e}")
            
        except Exception as e:
            print(f"❌ Raw access failed: {e}")
        
        finally:
            lidar.stop_motor()
            lidar.disconnect()
    
    except Exception as e:
        print(f"❌ Alternative test failed: {e}")

def main():
    print("=" * 60)
    print("🔍 LIDAR LIBRARY VERSION CHECK")
    print("=" * 60)
    
    if check_current_version():
        show_upgrade_instructions()
        test_simple_alternative()
    else:
        print("❌ RPLidar library not found")
        show_upgrade_instructions()
    
    print(f"\n💡 RECOMMENDED NEXT STEPS:")
    print(f"1. Upgrade the RPLidar library using instructions above")
    print(f"2. Test again with the protocol tester")
    print(f"3. The newer library should have iter_measurements and better protocol support")

if __name__ == "__main__":
    main()