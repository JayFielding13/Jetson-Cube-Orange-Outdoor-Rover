#!/bin/bash
# Linux launcher script for SSH Dashboard V3.7 - Real-time Terminal Fixed Edition

echo "🚀 Starting Rover SSH Dashboard V3.7 - Real-time Terminal Fixed Edition"
echo "========================================================================"
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
echo "🎯 REAL-TIME TERMINAL FIX APPLIED!"
echo "⚡ Based on working Dashboard V2 terminal implementation"
echo ""
echo "Key Features:"
echo "📡 Dedicated Navigation Pi tab (192.168.254.65)"
echo "🎨 Dedicated Companion Pi tab (192.168.254.70)" 
echo "📊 REAL-TIME terminal updates with channel-based reading"
echo "🔄 Independent file management per Pi"
echo "▶️ Run Python scripts directly from file browser"
echo ""
echo "✨ TERMINAL IMPROVEMENTS:"
echo "   • PTY allocation with get_pty=True"
echo "   • Non-blocking channel reading with recv_ready()"
echo "   • Real-time output polling loop"
echo "   • Immediate GUI updates with root.after(0, ...)"
echo ""
echo "💡 Perfect for testing your rover scripts with live output!"
echo ""

# Launch the real-time fixed dashboard
python3 main_enhanced_v3_dual_tabs_realtime_fixed.py

# Deactivate virtual environment when done
echo ""
echo "🧹 Cleaning up..."
deactivate
echo "✅ Dashboard closed successfully"