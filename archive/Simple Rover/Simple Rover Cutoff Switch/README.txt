================================================================================
                    SIMPLE ROVER - LoRa CUTOFF SWITCH
================================================================================

PROJECT OVERVIEW:
This folder contains everything needed to set up a wireless LoRa-based cutoff
switch for your rover using two Heltec LoRa WiFi 32 V3 boards.

FUNCTIONALITY:
  - Press a momentary button on the TRANSMITTER (remote control)
  - Toggle between STOP and AUTONOMOUS modes
  - RECEIVER on the rover receives the command
  - Raspberry Pi gets notified via Serial (USB)
  - Rover responds accordingly

================================================================================
FOLDER STRUCTURE:
================================================================================

Simple Rover Cutoff Switch/
├── README.txt                          (This file)
├── LoRa_Integration_Guide.txt          (Detailed integration guide)
├── WIRING_GUIDE.txt                    (Hardware connection details)
├── Transmitter/
│   └── LoRa_KillSwitch_Transmitter_Channel.ino  (Upload to remote board)
└── Receiver/
    └── LoRa_KillSwitch_Receiver_Channel.ino     (Upload to rover board)

================================================================================
QUICK START:
================================================================================

STEP 1: UPLOAD ARDUINO CODE
  1. Open Arduino IDE
  2. Install Heltec ESP32 board support (if not already installed)
  3. Upload Transmitter code to one Heltec board
  4. Upload Receiver code to another Heltec board

STEP 2: VERIFY SYNC WORDS MATCH
  - Open both .ino files
  - Find this line: #define SYNC_WORD   0x12
  - Make sure BOTH files have the SAME sync word value
  - This prevents interference with other rovers

STEP 3: WIRE THE HARDWARE

  TRANSMITTER (Remote Control):
    - Momentary Button: One side to GPIO 21, other side to GND
    - Power: USB cable or LiPo battery

  RECEIVER (On Rover):
    - USB cable to Raspberry Pi (provides power + serial communication)
    - Optional: Status LED on GPIO 21

STEP 4: TEST THE CONNECTION
  1. Power on both boards
  2. You should see "Waiting..." on the receiver display
  3. Press the button on transmitter
  4. Receiver display should update showing "STOP" or "AUTONOMOUS"

STEP 5: TEST RASPBERRY PI SERIAL
  1. Connect receiver to Pi via USB
  2. Find serial port: ls /dev/ttyUSB* /dev/ttyACM*
  3. Monitor serial: screen /dev/ttyUSB0 115200
  4. Press transmitter button - you should see:
     MODE:STOP or MODE:AUTONOMOUS
     {"mode":"...","rssi":...,"time":...}

STEP 6: INTEGRATE WITH ROVER CODE
  - See LoRa_Integration_Guide.txt for Python integration examples
  - Add serial reading to your existing rover controller
  - React to MODE:STOP and MODE:AUTONOMOUS commands

================================================================================
HARDWARE REQUIRED:
================================================================================

✓ 2x Heltec LoRa WiFi 32 V3 boards
✓ 1x Momentary push button (normally open)
✓ 2x USB cables (for programming and power)
✓ Antennas for both boards (usually included with Heltec boards)
✓ Optional: LiPo battery for portable transmitter
✓ Optional: LED + 220Ω resistor for status indicator

================================================================================
ARDUINO IDE SETUP:
================================================================================

BOARD MANAGER:
  1. Open Arduino IDE
  2. Go to: File > Preferences
  3. Add to "Additional Board Manager URLs":
     https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/releases/download/0.0.7/package_heltec_esp32_index.json
  4. Go to: Tools > Board > Boards Manager
  5. Search for "Heltec ESP32"
  6. Install "Heltec ESP32 Dev-Boards"

LIBRARY INSTALLATION:
  1. Go to: Sketch > Include Library > Manage Libraries
  2. Search and install:
     - "RadioLib" by Jan Gromeš
     - "Heltec ESP32 Dev-Boards" (should be installed with board package)

BOARD SELECTION:
  - Tools > Board > Heltec WiFi Series > "WiFi LoRa 32(V3)"
  - Tools > Port > Select appropriate COM port (COM3, COM4, etc. on Windows)
                   (/dev/ttyUSB0, /dev/ttyACM0 on Linux)

================================================================================
CHANNEL CONFIGURATION (SYNC WORDS):
================================================================================

To run multiple rovers without interference, each rover pair needs a unique
sync word. Think of it like a "channel" on a walkie-talkie.

CONFIGURATION:
In both Transmitter and Receiver code, find:
  #define SYNC_WORD   0x12

EXAMPLES FOR MULTIPLE ROVERS:
  Rover #1: 0x12
  Rover #2: 0x34
  Rover #3: 0x56
  Rover #4: 0x78
  Rover #5: 0xAB
  Rover #6: 0xCD

IMPORTANT: Transmitter and receiver MUST have matching sync words!

================================================================================
OPERATING MODES:
================================================================================

STOP MODE:
  - Rover is disabled
  - No autonomous movement
  - Safe state
  - Default mode on startup

AUTONOMOUS MODE:
  - Rover is enabled
  - Can execute autonomous navigation
  - Motors active
  - Sensors active

TOGGLING:
  - Press button once: Changes from current mode to the other
  - STOP → AUTONOMOUS → STOP → AUTONOMOUS...

================================================================================
DISPLAY INFORMATION:
================================================================================

TRANSMITTER DISPLAY:
  Line 1: "TX - Ch:0x12" (shows sync word/channel)
  Line 2: Current mode (STOP or AUTONOMOUS) - large text
  Line 3: Status (Sent OK or SEND FAILED)
  Line 4: Message count and [OK]/[FAIL] indicator

RECEIVER DISPLAY:
  Line 1: "RX - Ch:0x12" (shows sync word/channel)
  Line 2: Current mode (STOP or AUTONOMOUS) - large text
  Line 3: Last command received
  Line 4: RSSI signal strength and message count

================================================================================
TROUBLESHOOTING:
================================================================================

PROBLEM: Arduino IDE doesn't see the board
SOLUTION:
  - Install CH340/CP210x USB driver
  - Try different USB cable (must be data cable, not charge-only)
  - Press and hold BOOT button while connecting USB

PROBLEM: Receiver not getting messages
SOLUTION:
  - Check that sync words match in both codes
  - Check that frequency matches (915.0 MHz for US)
  - Verify antennas are connected to both boards
  - Move boards closer together for testing
  - Check Serial Monitor on both boards for error messages

PROBLEM: Raspberry Pi not seeing serial port
SOLUTION:
  - Run: ls /dev/ttyUSB* /dev/ttyACM*
  - Run: dmesg | tail (to see USB connection messages)
  - Try different USB port on Pi
  - Add user to dialout group: sudo usermod -a -G dialout $USER

PROBLEM: Garbled serial output on Pi
SOLUTION:
  - Verify baud rate is 115200 in your Python code
  - Check USB cable quality
  - Restart receiver board

PROBLEM: Range is poor
SOLUTION:
  - Make sure antennas are properly connected
  - Antennas should be vertical and not touching metal
  - Move away from WiFi routers and other RF sources
  - Check that output power is set to 22 dBm (maximum)

================================================================================
RANGE EXPECTATIONS:
================================================================================

Line of Sight:
  - Up to 2-3 km with good antennas and clear line of sight
  - Settings in code are optimized for range (SF=10, BW=125kHz)

Indoor/Urban:
  - 100-500 meters depending on obstacles
  - Walls and metal structures reduce range significantly

For Your Indoor Rover:
  - Should work throughout a typical house/building
  - May penetrate multiple walls/floors

================================================================================
TECHNICAL SPECIFICATIONS:
================================================================================

LoRa Configuration:
  - Frequency: 915.0 MHz (US) - Change for your region
  - Spreading Factor: 10 (good balance of range and speed)
  - Bandwidth: 125 kHz
  - Coding Rate: 4/8
  - Output Power: 22 dBm (maximum)
  - Sync Word: Configurable (default 0x12)

Serial Configuration (Receiver to Pi):
  - Baud Rate: 115200
  - Data Bits: 8
  - Parity: None
  - Stop Bits: 1

================================================================================
SAFETY FEATURES:
================================================================================

1. SAFE DEFAULT: Receiver starts in STOP mode
2. VISUAL FEEDBACK: OLED displays show current mode on both boards
3. SERIAL LOGGING: All commands logged to Serial Monitor for debugging
4. RSSI MONITORING: Signal strength displayed to verify connection quality
5. ERROR HANDLING: Failed transmissions are indicated on display

================================================================================
NEXT STEPS:
================================================================================

1. ✓ Upload code to both Heltec boards
2. ✓ Test basic LoRa communication
3. ✓ Verify serial output on Raspberry Pi
4. ⃞ Integrate with existing rover control code
5. ⃞ Test in real rover environment
6. ⃞ Tune range and reliability as needed
7. ⃞ Add additional features if desired (e.g., battery monitoring)

================================================================================
SUPPORT & RESOURCES:
================================================================================

Heltec Documentation:
  https://heltec.org/project/wifi-lora-32-v3/

RadioLib Documentation:
  https://jgromes.github.io/RadioLib/

For issues or questions:
  - Check Serial Monitor on both boards for debug messages
  - Refer to LoRa_Integration_Guide.txt for detailed Pi integration
  - Check WIRING_GUIDE.txt for hardware connections

================================================================================
