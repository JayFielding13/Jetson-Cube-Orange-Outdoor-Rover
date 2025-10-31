#!/bin/bash
# Conservative terminal fix launcher for SSH Dashboard V3.5

echo "🔧 Starting Rover SSH Dashboard V3.5 - Conservative Terminal Fix"
echo "==============================================================="
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
echo "🎯 CONSERVATIVE TERMINAL FIX APPLIED"
echo "⚡ Safer implementation with better error handling"
echo ""
echo "Features:"
echo "📡 Dual Pi connection management (select from dropdown)"
echo "📁 File management with upload/download"
echo "💻 Real-time terminal with channel-based reading"
echo "📊 System monitoring and health checks"
echo "▶️ Python script execution with live output"
echo ""
echo "💡 This version should be more stable while fixing terminal output!"
echo ""

# Launch the conservative terminal fix dashboard
python3 main_enhanced_v3_terminal_fix.py

# Deactivate virtual environment when done
echo ""
echo "🧹 Cleaning up..."
deactivate
echo "✅ Dashboard closed successfully"