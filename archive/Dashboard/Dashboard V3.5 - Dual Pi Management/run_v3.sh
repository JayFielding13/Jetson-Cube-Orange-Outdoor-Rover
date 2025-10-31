#!/bin/bash
# Linux launcher script for SSH Dashboard V3.5 - Dual Pi Management Edition

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Install/upgrade requirements
echo "Installing requirements..."
pip install -r requirements.txt

# Ensure X11 forwarding is enabled (optional setup)
echo "Setting up X11 forwarding permissions..."
xhost +local: 2>/dev/null || echo "X11 setup skipped (running without display)"

# Launch the dual Pi management dashboard
echo "Starting SSH Dashboard V3.5 - Dual Pi Management Edition..."
echo "Navigation Pi: 192.168.254.65"
echo "Companion Pi: 192.168.254.70"
python3 main_enhanced_v3.py

# Deactivate virtual environment when done
deactivate