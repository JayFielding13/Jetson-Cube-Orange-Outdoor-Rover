#!/bin/bash
# Linux launcher script for SSH Dashboard V3.6 - Minimal Terminal Fix

echo "🚀 Starting Rover SSH Dashboard V3.6 - Minimal Terminal Fix"
echo "==========================================================="
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

echo ""
echo "🎯 MINIMAL TERMINAL FIX APPLIED!"
echo "⚡ Conservative approach with working dual Pi tabs"
echo ""
echo "Key Features:"
echo "📡 Navigation Pi tab (192.168.254.65) - Full file management"
echo "🎨 Companion Pi tab (192.168.254.70) - Full file management"
echo "💻 Terminal tab with real-time execution for both Pis"
echo "⚙️ Settings tab with configuration options"
echo ""
echo "✨ TERMINAL FEATURES:"
echo "   • PTY allocation for real-time output"
echo "   • Channel-based reading with recv_ready()"
echo "   • Execute/Stop/Clear controls per Pi"
echo "   • Separate terminals for each Pi"
echo ""
echo "💡 Minimal changes to ensure stability!"
echo ""

# Launch the minimal terminal fix dashboard
python3 main_enhanced_v3_dual_tabs_minimal_terminal.py

# Deactivate virtual environment when done
echo ""
echo "🧹 Cleaning up..."
deactivate
echo "✅ Dashboard closed successfully"