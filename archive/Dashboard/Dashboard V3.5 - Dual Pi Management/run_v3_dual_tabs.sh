#!/bin/bash
# Linux launcher script for SSH Dashboard V3.6 - Dual Pi Dedicated Tabs Edition

echo "🤖 Starting Rover SSH Dashboard V3.6 - Dual Pi Dedicated Tabs Edition"
echo "=================================================================="
echo ""

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "📦 Creating Python virtual environment..."
    python3 -m venv venv
    echo "✅ Virtual environment created"
fi

# Activate virtual environment
echo "🔄 Activating virtual environment..."
source venv/bin/activate

# Install/upgrade requirements
echo "📋 Installing/updating requirements..."
pip install -q --upgrade pip
pip install -q -r requirements.txt

# Ensure X11 forwarding is enabled (optional setup)
echo "🖥️  Setting up X11 forwarding permissions..."
xhost +local: 2>/dev/null || echo "   (X11 setup skipped - running without display)"

echo ""
echo "🚀 Launching Dashboard V3.6 - Dual Pi Dedicated Tabs Edition"
echo ""
echo "Key Features:"
echo "📡 Dedicated Navigation Pi tab (192.168.254.65)"
echo "🎨 Dedicated Companion Pi tab (192.168.254.70)" 
echo "📊 Dual Pi system monitoring"
echo "🔄 Independent file management per Pi"
echo ""
echo "💡 Use this dashboard to:"
echo "   • Upload your intelligent wanderer script to Navigation Pi"
echo "   • Upload your sensor visualizer to Companion Pi"
echo "   • Monitor both Pi systems in real-time"
echo "   • Manage files independently on each Pi"
echo ""

# Launch the enhanced dual Pi dashboard
python3 main_enhanced_v3_dual_tabs.py

# Deactivate virtual environment when done
echo ""
echo "🧹 Cleaning up..."
deactivate
echo "✅ Dashboard closed successfully"