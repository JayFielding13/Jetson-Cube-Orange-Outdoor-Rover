#!/bin/bash
# Linux launcher script for SSH Dashboard V3.6 - Full Dual Pi Tabs with Real-time Terminal

echo "🚀 Starting Rover SSH Dashboard V3.6 - Dual Pi Tabs with Real-time Terminal"
echo "=========================================================================="
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
echo "🎯 FULL RESTORATION COMPLETE!"
echo "⚡ All dual Pi tabs + working real-time terminal"
echo ""
echo "Key Features:"
echo "📡 Navigation Pi tab (192.168.254.65) with independent file management"
echo "🎨 Companion Pi tab (192.168.254.70) with independent file management"
echo "💻 REAL-TIME terminal execution with channel-based reading"
echo "📊 System monitoring for both Pis"
echo "▶️ Execute/Stop/Clear controls for each Pi terminal"
echo ""
echo "✨ TERMINAL IMPROVEMENTS:"
echo "   • PTY allocation with get_pty=True"
echo "   • Channel-based reading with recv_ready()"
echo "   • Real-time output polling loop"
echo "   • Immediate GUI updates with root.after(0, ...)"
echo ""
echo "💡 Perfect for dual Pi rover development with live output!"
echo ""

# Launch the fully restored dual Pi dashboard with real-time terminal
python3 main_enhanced_v3_dual_tabs_with_terminal.py

# Deactivate virtual environment when done
echo ""
echo "🧹 Cleaning up..."
deactivate
echo "✅ Dashboard closed successfully"