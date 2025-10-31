#!/bin/bash
# Linux launcher script for SSH Dashboard V3 - Enhanced File Management

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

# Launch the enhanced dashboard
echo "Starting SSH Dashboard V3 - Enhanced File Management Edition..."
python3 main_enhanced_v3.py

# Deactivate virtual environment when done
deactivate