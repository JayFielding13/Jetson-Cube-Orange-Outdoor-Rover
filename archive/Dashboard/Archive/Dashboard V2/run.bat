@echo off
echo Installing dependencies...
pip install -r requirements.txt
echo.
echo Starting Raspberry Pi SSH Dashboard...
python main.py
pause