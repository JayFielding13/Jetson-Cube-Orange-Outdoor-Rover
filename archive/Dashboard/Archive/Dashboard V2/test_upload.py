#!/usr/bin/env python3
"""
Test file for SSH Dashboard upload functionality
This file verifies the upload works and environment is properly activated
"""

import sys
import os
from datetime import datetime

def main():
    print("=" * 50)
    print("SSH Dashboard Upload Test - SUCCESS!")
    print("=" * 50)
    print(f"Current time: {datetime.now()}")
    print(f"Python version: {sys.version}")
    print(f"Python executable: {sys.executable}")
    print(f"Current working directory: {os.getcwd()}")
    print(f"Script location: {os.path.abspath(__file__)}")
    
    # Check if we're in a virtual environment
    if hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix):
        print("✅ Virtual environment is ACTIVE")
    else:
        print("❌ Virtual environment is NOT active")
    
    # List some environment info
    print("\nEnvironment variables:")
    for key in ['PATH', 'VIRTUAL_ENV', 'PWD']:
        value = os.environ.get(key, 'Not set')
        print(f"  {key}: {value}")
    
    print("\nFile upload and execution test completed successfully!")
    print("=" * 50)

if __name__ == "__main__":
    main()